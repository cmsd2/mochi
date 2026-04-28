;;;; mochi-loader.lisp -- Lisp side of the mochi package.
;;;;
;;;; Provides $mod_load: parse a Modelica .mo file via rumoca and build a
;;;; Maxima struct directly, with no Python helper or temp file.
;;;;
;;;; Pipeline:
;;;;   .mo -> rumoca compile --json -> cl-json:decode-json-from-string
;;;;       -> mochi--ast-to-maxima walks the AST -> Maxima list returned.
;;;;
;;;; Symbols used in the Modelica model become Maxima symbols ($R, $L, ...)
;;;; constructed directly in Lisp -- no Maxima-side eval pass, so existing
;;;; user bindings are untouched.

(in-package :maxima)

;; --- Bootstrap Quicklisp + cl-json -------------------------------------

(unless (find-package :ql)
  (let ((setup (merge-pathnames "quicklisp/setup.lisp"
                                (user-homedir-pathname))))
    (when (probe-file setup)
      (load setup))))

(unless (find-package :ql)
  (error "mochi: Quicklisp not found. Run `mxpm setup quicklisp`."))

(funcall (intern "QUICKLOAD" :ql) :cl-json :silent t)

;; --- Locating rumoca ----------------------------------------------------

(defun mochi--rumoca-bin ()
  "Resolve the rumoca executable.  Honours $RUMOCA_BIN, then ~/.cargo/bin,
then assumes it's on $PATH."
  (or (uiop:getenv "RUMOCA_BIN")
      (let ((cargo (merge-pathnames ".cargo/bin/rumoca"
                                    (user-homedir-pathname))))
        (when (probe-file cargo)
          (namestring cargo)))
      "rumoca"))

(defun mochi--run-rumoca (mo-path)
  "Run rumoca on MO-PATH and return the JSON output as a string."
  (let* ((abs (truename mo-path))
         (dir (directory-namestring abs))
         (name (file-namestring abs)))
    (uiop:run-program (list (mochi--rumoca-bin) "compile" "--json" name)
                      :output :string
                      :directory dir)))

;; --- Maxima expression construction helpers ----------------------------

(defun mochi--maxima-case-invert (s)
  "Maxima's reader inverts case for all-uppercase or all-lowercase identifiers
(it's how Common Lisp's case-folding reader hides itself from users).  Apply
the same convention here so a Modelica name like `R' produces a Maxima symbol
that *displays* as `R' rather than `r'."
  (let ((has-upper nil) (has-lower nil))
    (loop for ch across s
          do (cond ((upper-case-p ch) (setf has-upper t))
                   ((lower-case-p ch) (setf has-lower t))))
    (cond ((and has-upper has-lower) s)
          (has-upper (string-downcase s))
          (has-lower (string-upcase s))
          (t s))))

(defun mochi--mxsym (name)
  "Intern NAME (a string in the original Modelica casing) as a Maxima
symbol whose display matches NAME."
  (intern (concatenate 'string "$" (mochi--maxima-case-invert name)) :maxima))

(defun mochi--mlist (items)
  "Build a Maxima list ((MLIST) item1 item2 ...) from a Lisp list."
  (cons '(mlist) items))

(defun mochi--mequal (lhs rhs)
  "Build ((MEQUAL) lhs rhs)."
  (list '(mequal) lhs rhs))

;; --- AST walker --------------------------------------------------------

(defun mochi--get (alist key)
  (cdr (assoc key alist)))

(defun mochi--unwrap-tagged (node)
  "A tagged variant value from cl-json is a one-element alist
((:*TAG . body)).  Peel that wrapper and return (values TAG BODY)."
  (values (caar node) (cdar node)))

(defun mochi--ast-to-maxima (node)
  "Convert a rumoca JSON AST node (cl-json's parsed form) into a Maxima
expression in Lisp form."
  (cond
    ((numberp node) node)
    ((stringp node) node)
    ((eq node t) t)
    ((eq node nil) nil)
    ((not (consp node))
     (error "mochi: don't know how to convert node ~S" node))
    (t
     (multiple-value-bind (tag body) (mochi--unwrap-tagged node)
       (case tag
         (:*var-ref
          (mochi--mxsym (mochi--get body :name)))
         (:*literal
          (mochi--literal-value body))
         (:*binary
          (mochi--binary-to-maxima body))
         (:*unary
          (mochi--unary-to-maxima body))
         (:*builtin-call
          (mochi--builtin-call-to-maxima body))
         (:*paren
          (mochi--ast-to-maxima (mochi--get body :expr)))
         (:*function-call
          (let* ((fname (mochi--get body :name))
                 (args (mapcar #'mochi--ast-to-maxima
                               (mochi--get body :args))))
            (cons (list (mochi--mxsym fname)) args)))
         (otherwise
          (error "mochi: unsupported AST tag ~S" tag)))))))

(defun mochi--literal-value (body)
  "BODY is a list with one of (:*REAL . val), (:*INTEGER . val), etc.
cl-json wraps Pascal-case tags in `*' to preserve case."
  (cond
    ((assoc :*real body)    (cdr (assoc :*real body)))
    ((assoc :*integer body) (cdr (assoc :*integer body)))
    ((assoc :*boolean body) (if (cdr (assoc :*boolean body)) t nil))
    ((assoc :*string body)  (cdr (assoc :*string body)))
    (t (error "mochi: unknown literal shape ~S" body))))

(defun mochi--binary-to-maxima (body)
  (let* ((op-node (mochi--get body :op))
         (op-tag (caar op-node))   ; op-node is a one-key alist like ((:*MUL ...))
         (lhs (mochi--ast-to-maxima (mochi--get body :lhs)))
         (rhs (mochi--ast-to-maxima (mochi--get body :rhs))))
    (case op-tag
      (:*add (list '(mplus) lhs rhs))
      (:*sub (list '(mplus) lhs (list '(mtimes) -1 rhs)))
      (:*mul (list '(mtimes) lhs rhs))
      (:*div (list '(mtimes) lhs (list '(mexpt) rhs -1)))
      (:*pow (list '(mexpt) lhs rhs))
      (:*eq  (list '(mequal) lhs rhs))
      (otherwise
       (error "mochi: unsupported binary op ~S" op-tag)))))

(defun mochi--unary-to-maxima (body)
  (let* ((op-node (mochi--get body :op))
         (op-tag (caar op-node))
         (arg (mochi--ast-to-maxima (mochi--get body :arg))))
    (case op-tag
      (:*neg (list '(mtimes) -1 arg))
      (:*pos arg)
      (otherwise
       (error "mochi: unsupported unary op ~S" op-tag)))))

(defun mochi--builtin-call-to-maxima (body)
  (let* ((fn (mochi--get body :function))
         (raw-args (mochi--get body :args))
         (args (mapcar #'mochi--ast-to-maxima raw-args)))
    (cond
      ((string= fn "Der")
       ;; der(x) → der_x as a single Maxima symbol.  Reach into the raw AST
       ;; for the VarRef name (rather than going through the converted Maxima
       ;; symbol) so case-inversion is applied exactly once -- otherwise
       ;; this `der_*' symbol would diverge from the one in (mod_get m 'derivs).
       (let ((var-name (mochi--get (cdar (first raw-args)) :name)))
         (mochi--mxsym (concatenate 'string "der_" var-name))))
      ((string= fn "Sin") (cons '(%sin) args))
      ((string= fn "Cos") (cons '(%cos) args))
      ((string= fn "Tan") (cons '(%tan) args))
      ((string= fn "Asin") (cons '(%asin) args))
      ((string= fn "Acos") (cons '(%acos) args))
      ((string= fn "Atan") (cons '(%atan) args))
      ((string= fn "Atan2") (cons '(%atan2) args))
      ((string= fn "Sinh") (cons '(%sinh) args))
      ((string= fn "Cosh") (cons '(%cosh) args))
      ((string= fn "Tanh") (cons '(%tanh) args))
      ((string= fn "Exp") (cons '(%exp) args))
      ((string= fn "Log") (cons '(%log) args))
      ((string= fn "Sqrt") (cons '(%sqrt) args))
      ((string= fn "Abs") (cons '(mabs) args))
      ((string= fn "Min") (cons '($min) args))
      ((string= fn "Max") (cons '($max) args))
      ((string= fn "Floor") (cons '($floor) args))
      ((string= fn "Ceil") (cons '($ceiling) args))
      (t
       (cons (list (mochi--mxsym (string-downcase fn))) args)))))

;; --- Inputs/outputs from the .mo source --------------------------------
;; rumoca's --json output doesn't tag inputs/outputs separately, but they
;; appear as `input Real <name>;` / `output Real <name>;` in the source.

(defun mochi--strip-comments (text)
  "Remove /* ... */ block and // line comments from TEXT.  Crude but
sufficient for scanning declarations."
  (let* ((step1 (mochi--strip-blocks text))
         (step2 (mochi--strip-lines step1)))
    step2))

(defun mochi--strip-blocks (text)
  (with-output-to-string (out)
    (loop with len = (length text)
          with i = 0
          while (< i len)
          do (cond
               ((and (< i (1- len))
                     (char= (char text i) #\/)
                     (char= (char text (1+ i)) #\*))
                ;; consume until "*/"
                (let ((end (search "*/" text :start2 (+ i 2))))
                  (setf i (if end (+ end 2) len))))
               (t
                (write-char (char text i) out)
                (incf i))))))

(defun mochi--strip-lines (text)
  (with-output-to-string (out)
    (loop with len = (length text)
          with i = 0
          while (< i len)
          do (cond
               ((and (< i (1- len))
                     (char= (char text i) #\/)
                     (char= (char text (1+ i)) #\/))
                ;; consume to end-of-line
                (let ((end (position #\Newline text :start (+ i 2))))
                  (setf i (if end end len))))
               (t
                (write-char (char text i) out)
                (incf i))))))

(defun mochi--scan-keyword (text keyword)
  "Find every occurrence of `<keyword> Real <name>' or `<keyword> Real <name>(...)';
return a list of <name> strings."
  (let ((pat (concatenate 'string keyword " Real "))
        (names '())
        (start 0))
    (loop
      (let ((pos (search pat text :start2 start)))
        (unless pos (return (nreverse names)))
        (setf start (+ pos (length pat)))
        ;; Read identifier at start: alpha/underscore then alnum/_
        (let ((id-start start))
          (loop while (and (< start (length text))
                           (or (alphanumericp (char text start))
                               (char= (char text start) #\_)))
                do (incf start))
          (when (> start id-start)
            (push (subseq text id-start start) names)))))))

(defun mochi--read-file (path)
  (with-open-file (s path :direction :input :external-format :utf-8)
    (with-output-to-string (out)
      (loop for line = (read-line s nil nil)
            while line do (write-line line out)))))

(defun mochi--scan-io (mo-path)
  "Return (values inputs outputs) from the source file."
  (let* ((text (mochi--strip-comments (mochi--read-file mo-path))))
    (values (mochi--scan-keyword text "input")
            (mochi--scan-keyword text "output"))))

(defun mochi--model-name-from-source (mo-path)
  "Best-effort extraction of the top-level model name from the .mo file."
  (let* ((text (mochi--strip-comments (mochi--read-file mo-path))))
    (or (mochi--first-keyword-name text "model")
        (mochi--first-keyword-name text "class")
        (mochi--first-keyword-name text "block")
        "Unnamed")))

(defun mochi--first-keyword-name (text keyword)
  (let* ((pat (concatenate 'string keyword " "))
         (pos (search pat text)))
    (when pos
      (let* ((start (+ pos (length pat)))
             (id-start start))
        (loop while (and (< start (length text))
                         (or (alphanumericp (char text start))
                             (char= (char text start) #\_)))
              do (incf start))
        (when (> start id-start)
          (subseq text id-start start))))))

;; --- Build the Maxima struct -------------------------------------------

(defun mochi--start-value (info)
  "INFO is a parameter or state info alist.  Return the start-value as a Lisp
number (defaulting to 0)."
  (let ((start (mochi--get info :start)))
    ;; start is a one-element alist ((:*LITERAL . body)).
    (if (and (consp start) (consp (car start)) (eq (caar start) :*literal))
        (mochi--literal-value (cdar start))
        0)))

(defun mochi--name-from-info (info)
  "INFO is a parameter or state info alist with a :NAME field."
  (mochi--get info :name))

(defun mochi--params-list (raw)
  "RAW is the alist under :p from rumoca."
  (mochi--mlist
   (mapcar (lambda (entry)
             (let* ((info (cdr entry))
                    (name (mochi--name-from-info info))
                    (val (mochi--start-value info)))
               (mochi--mlist (list (mochi--mxsym name) val))))
           raw)))

(defun mochi--state-symbols (raw)
  "RAW is the alist under :x from rumoca; return ((MLIST) $name1 $name2 ...)."
  (mochi--mlist
   (mapcar (lambda (entry)
             (mochi--mxsym (mochi--name-from-info (cdr entry))))
           raw)))

(defun mochi--deriv-symbols (raw)
  (mochi--mlist
   (mapcar (lambda (entry)
             (mochi--mxsym (concatenate 'string "der_"
                                        (mochi--name-from-info (cdr entry)))))
           raw)))

(defun mochi--initial-list (raw)
  (mochi--mlist
   (mapcar (lambda (entry)
             (let* ((info (cdr entry))
                    (name (mochi--name-from-info info))
                    (val (mochi--start-value info)))
               (mochi--mlist (list (mochi--mxsym name) val))))
           raw)))

(defun mochi--io-list (names)
  (mochi--mlist (mapcar #'mochi--mxsym names)))

(defun mochi--residuals (raw)
  "RAW is the list under :f--x from rumoca."
  (mochi--mlist
   (mapcar (lambda (eq-entry)
             (mochi--ast-to-maxima (mochi--get eq-entry :residual)))
           raw)))

;; --- Public entry point ------------------------------------------------

(defun $mod_load (path)
  "Parse a Modelica .mo file and return a Maxima model struct."
  (let* ((mo-path (if (stringp path) path (string path)))
         (json-text (mochi--run-rumoca mo-path))
         (model (cl-json:decode-json-from-string json-text))
         (params (cdr (assoc :p model)))
         (states (cdr (assoc :x model)))
         (eqs (cdr (assoc :f--x model))))
    (multiple-value-bind (inputs outputs) (mochi--scan-io mo-path)
      (let ((name (mochi--model-name-from-source mo-path)))
        (mochi--mlist
         (list (mochi--mequal '$name name)
               (mochi--mequal '$params (mochi--params-list params))
               (mochi--mequal '$states (mochi--state-symbols states))
               (mochi--mequal '$derivs (mochi--deriv-symbols states))
               (mochi--mequal '$inputs (mochi--io-list inputs))
               (mochi--mequal '$outputs (mochi--io-list outputs))
               (mochi--mequal '$initial (mochi--initial-list states))
               (mochi--mequal '$residuals (mochi--residuals eqs))))))))
