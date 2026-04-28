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

(defun mochi--run-rumoca (mo-path &optional model-name)
  "Run rumoca on MO-PATH and return the JSON output as a string.
If MODEL-NAME is given, pass --model so rumoca picks that class as the
top-level (required when the file contains more than one model, e.g.
when reusable component classes are defined alongside the top-level
system)."
  (let* ((abs (truename mo-path))
         (dir (directory-namestring abs))
         (name (file-namestring abs))
         (cmd (append (list (mochi--rumoca-bin) "compile" "--json")
                      (when model-name (list "--model" model-name))
                      (list name))))
    (uiop:run-program cmd :output :string :directory dir)))

;; --- Locating the diagram renderers (`dot' / `mmdc') -------------------

(defun mochi--dot-bin ()
  "Resolve the GraphViz `dot' executable.  Honours $DOT_BIN, then probes
the common Homebrew / system locations, then falls back to plain `dot'
on $PATH (which may not be inherited from the GUI launcher on macOS)."
  (or (uiop:getenv "DOT_BIN")
      (loop for cand in '("/opt/homebrew/bin/dot"
                          "/usr/local/bin/dot"
                          "/usr/bin/dot")
            when (probe-file cand)
              return cand)
      "dot"))

(defun mochi--mmdc-bin ()
  "Resolve the Mermaid CLI executable (`mmdc').  Honours $MMDC_BIN, then
probes the common Homebrew / system locations, then falls back to plain
`mmdc' on $PATH."
  (or (uiop:getenv "MMDC_BIN")
      (loop for cand in '("/opt/homebrew/bin/mmdc"
                          "/usr/local/bin/mmdc"
                          "/usr/bin/mmdc")
            when (probe-file cand)
              return cand)
      "mmdc"))

(defun mochi--temp-dir ()
  "Resolve a temp directory ending in a path separator.  Prefers Maxima's
$maxima_tempdir (which the maxima-extension sets up), falling back to
the OS default."
  (namestring
   (uiop:ensure-directory-pathname
    (or (and (boundp '$maxima_tempdir)
             (stringp $maxima_tempdir)
             (not (string= $maxima_tempdir ""))
             $maxima_tempdir)
        (namestring (uiop:temporary-directory))))))

(defun mochi--write-temp (source extension)
  "Write SOURCE (a string) to a fresh file in the Maxima temp directory
with the given EXTENSION (without dot).  Returns the path."
  (let* ((temp-dir (mochi--temp-dir))
         (stamp (format nil "~A_~A" (get-universal-time) (random 1000000)))
         (path (format nil "~Amochi_~A.~A" temp-dir stamp extension)))
    (with-open-file (s path :direction :output
                            :if-exists :supersede
                            :if-does-not-exist :create)
      (write-string source s))
    path))

(defun mochi--svg-path-for (input-path)
  "Mirror INPUT-PATH but with a .svg extension (uniqueness comes from the
input path's stamp)."
  (let ((dot (position #\. input-path :from-end t)))
    (concatenate 'string (subseq input-path 0 dot) ".svg")))

(defun mochi--emit-svg-path (svg-path)
  "Print SVG-PATH quoted on its own line so the maxima-extension / Aximar
parser recognises it as an inline image/svg+xml display output."
  (format t "\"~A\"~%" svg-path))

(defun $mod__render_dot_to_svg (dot-source)
  "Render DOT source to a temp .svg file via the GraphViz `dot' binary.
Prints the path quoted so Aximar picks it up as image/svg+xml, and
returns the path as a Maxima string."
  (unless (stringp dot-source)
    (merror "mod__render_dot_to_svg: expected a string, got: ~M" dot-source))
  (let* ((dot-path (mochi--write-temp dot-source "dot"))
         (svg-path (mochi--svg-path-for dot-path)))
    (handler-case
        (uiop:run-program (list (mochi--dot-bin) "-Tsvg" dot-path "-o" svg-path)
                          :output :string
                          :error-output :string)
      (error (e)
        (merror "mod__render_dot_to_svg: dot failed (~A).  Is GraphViz installed and on $PATH or $DOT_BIN?"
                e)))
    (mochi--emit-svg-path svg-path)
    svg-path))

(defun $mod__render_mermaid_to_svg (mermaid-source)
  "Render Mermaid source to a temp .svg file via the Mermaid CLI (`mmdc').
Prints the path quoted so Aximar picks it up as image/svg+xml, and
returns the path as a Maxima string."
  (unless (stringp mermaid-source)
    (merror "mod__render_mermaid_to_svg: expected a string, got: ~M" mermaid-source))
  (let* ((mmd-path (mochi--write-temp mermaid-source "mmd"))
         (svg-path (mochi--svg-path-for mmd-path)))
    (handler-case
        (uiop:run-program (list (mochi--mmdc-bin)
                                "-i" mmd-path
                                "-o" svg-path
                                "--quiet")
                          :output :string
                          :error-output :string)
      (error (e)
        (merror "mod__render_mermaid_to_svg: mmdc failed (~A).  Install with `npm install -g @mermaid-js/mermaid-cli', or set $MMDC_BIN."
                e)))
    (mochi--emit-svg-path svg-path)
    svg-path))

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

(defun mochi--flatten-dots (name)
  "Translate hierarchical Modelica names like `tank1.h' into Maxima-safe
identifiers `tank1_h' (Maxima symbols can't contain dots)."
  (substitute #\_ #\. name))

(defun mochi--mxsym (name)
  "Intern NAME (a string in the original Modelica casing, possibly with
hierarchical dots from connector composition) as a Maxima symbol whose
display matches NAME with dots translated to underscores."
  (intern (concatenate 'string "$"
                       (mochi--maxima-case-invert (mochi--flatten-dots name)))
          :maxima))

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
  ;; rumoca's Unary node carries the operand under :rhs (not :arg).
  ;; Fall back to :arg in case a future rumoca version normalises it.
  (let* ((op-node (mochi--get body :op))
         (op-tag (caar op-node))
         (arg-node (or (cdr (assoc :rhs body))
                       (cdr (assoc :arg body))))
         (arg (mochi--ast-to-maxima arg-node)))
    (case op-tag
      ;; rumoca emits :*MINUS / :*PLUS for unary signs (matching the
      ;; binary-op tag names).  :*NEG / :*POS are tolerated as aliases.
      ((:*minus :*neg) (list '(mtimes) -1 arg))
      ((:*plus  :*pos) arg)
      (:*not (list '(mnot) arg))
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

(defun mochi--top-model-block (text)
  "Return (values name body) for the *last* `model NAME ... end NAME;'
block in TEXT.  Modelica's convention is that the top-level model is the
last one declared; reusable building-block classes come earlier."
  (let ((best-name nil)
        (best-body nil)
        (search-from 0))
    (loop
      (let ((kw-pos (search "model " text :start2 search-from)))
        (unless kw-pos (return (values best-name best-body)))
        ;; Pull the model's name
        (let* ((name-start (+ kw-pos 6))
               (name-end name-start))
          (loop while (and (< name-end (length text))
                           (or (alphanumericp (char text name-end))
                               (char= (char text name-end) #\_)))
                do (incf name-end))
          (when (> name-end name-start)
            (let* ((name (subseq text name-start name-end))
                   (terminator (concatenate 'string "end " name ";"))
                   (end-pos (search terminator text :start2 name-end)))
              (when end-pos
                (setf best-name name
                      best-body (subseq text name-end end-pos))
                (setf search-from (+ end-pos (length terminator)))))))
        (when (>= search-from (length text)) (return (values best-name best-body)))
        (setf search-from (max search-from (1+ kw-pos)))))))

(defun mochi--collect-class-io (text)
  "Walk every `model NAME ... end NAME;' block in TEXT and return an alist
mapping the class name (string) to (list inputs outputs), each a list of
declared identifier names."
  (let ((classes '())
        (search-from 0)
        (len (length text)))
    (loop
      (let ((kw-pos (search "model " text :start2 search-from)))
        (unless kw-pos (return (nreverse classes)))
        (let* ((name-start (+ kw-pos 6))
               (name-end name-start))
          (loop while (and (< name-end len)
                           (or (alphanumericp (char text name-end))
                               (char= (char text name-end) #\_)))
                do (incf name-end))
          (cond
            ((> name-end name-start)
             (let* ((cname (subseq text name-start name-end))
                    (terminator (concatenate 'string "end " cname ";"))
                    (end-pos (search terminator text :start2 name-end)))
               (cond
                 (end-pos
                  (let ((body (subseq text name-end end-pos)))
                    (push (list cname
                                (mochi--scan-keyword body "input")
                                (mochi--scan-keyword body "output"))
                          classes))
                  (setf search-from (+ end-pos (length terminator))))
                 (t (setf search-from name-end)))))
            (t (setf search-from (1+ kw-pos)))))
        (when (>= search-from len) (return (nreverse classes)))))))

(defun mochi--read-ident-at (text start)
  "Read a (possibly empty) identifier from TEXT starting at START.
Return the substring or nil if no identifier."
  (let ((end start))
    (loop while (and (< end (length text))
                     (or (alphanumericp (char text end))
                         (char= (char text end) #\_)))
          do (incf end))
    (when (> end start) (subseq text start end))))

(defun mochi--skip-spaces (text pos)
  "Advance past spaces and tabs."
  (loop while (and (< pos (length text))
                   (member (char text pos) '(#\Space #\Tab)))
        do (incf pos))
  pos)

(defun mochi--bound-lhses (body)
  "Walk the equation section of BODY (the top-level model body) and
collect identifier names that appear on the LHS of an equation.
Identifiers may include dots (`tank1.q_in').  Used to exclude these
from the candidate-inputs list -- if a variable is set by a top-level
equation, it's not a free input.  Skips `connect(a, b);' calls and
anything inside parentheses (so parameter modifications like
`Resistor r1(R = 1.0);' don't fool us)."
  (let* ((eq-start (search "equation" body))
         (scan (if eq-start
                   (subseq body (+ eq-start (length "equation")))
                   ""))
         (len (length scan))
         (i 0)
         (depth 0)
         (results '()))
    (loop while (< i len)
          do (let ((ch (char scan i)))
               (cond
                 ((or (char= ch #\() (char= ch #\[)) (incf depth) (incf i))
                 ((or (char= ch #\)) (char= ch #\])) (decf depth) (incf i))
                 ((and (zerop depth)
                       (or (alpha-char-p ch) (char= ch #\_)))
                  ;; identifier-or-instance.var
                  (let ((start i))
                    (loop while (and (< i len)
                                     (or (alphanumericp (char scan i))
                                         (char= (char scan i) #\_)
                                         (char= (char scan i) #\.)))
                          do (incf i))
                    (let* ((id (subseq scan start i))
                           (j (mochi--skip-spaces scan i)))
                      ;; Look ahead for `= NOT-=' at the top level
                      (when (and (< j len) (char= (char scan j) #\=)
                                 (not (and (< (1+ j) len)
                                           (char= (char scan (1+ j)) #\=))))
                        ;; Skip pure connect() / smooth() calls etc.; identifier
                        ;; followed by `(' is a function call, not LHS.  We
                        ;; already advanced past the identifier so check the
                        ;; original position's lookahead for `('.
                        (push id results))
                      (setf i j))))
                 (t (incf i)))))
    (nreverse results)))

(defun mochi--scan-instances (body class-table)
  "BODY is the top-level model's body.  CLASS-TABLE is the alist returned
by mochi--collect-class-io.  Find component instance declarations of the
form `ClassName instance_name'; for each instance, namespace the class's
input/output names as `instance.var' and return (values inputs outputs).

Inputs that are bound by an explicit equation in BODY (e.g. `tank1.q_in
= source;') are excluded -- they're algebraic, not free top-level
inputs."
  (let ((inputs '())
        (outputs '())
        (bound (mochi--bound-lhses body)))
    (dolist (entry class-table)
      (let* ((cname (first entry))
             (cinputs (second entry))
             ;; Instance OUTPUTS deliberately not used: see note below.
             (search-from 0))
        (loop
          (let ((pos (search cname body :start2 search-from)))
            (unless pos (return))
            ;; Require word boundary before
            (let ((before (if (zerop pos) #\Space (char body (1- pos))))
                  (after-name (mochi--skip-spaces body (+ pos (length cname)))))
              (cond
                ((or (alphanumericp before) (char= before #\_))
                 ;; Embedded inside another identifier; skip
                 (setf search-from (1+ pos)))
                (t
                 (let ((inst-name (mochi--read-ident-at body after-name)))
                   (cond
                     ((and inst-name (> (length inst-name) 0))
                      ;; Only instance INPUTS are surfaced: an instance's
                      ;; declared `input Real X' is a top-level free
                      ;; variable unless bound by an equation here.
                      ;; Instance OUTPUTS are not surfaced -- they're
                      ;; internal signals; the user can read them via
                      ;; mod_get(m, 'algebraics) or compute via a
                      ;; user-typed top-level output equation.
                      (dolist (v cinputs)
                        (let ((qualified (concatenate 'string inst-name "." v)))
                          (unless (member qualified bound :test #'string=)
                            (push qualified inputs))))
                      (setf search-from (+ after-name (length inst-name))))
                     (t (setf search-from (1+ pos))))))))))))
    (values (nreverse inputs) (nreverse outputs))))

(defun mochi--scan-io (mo-path)
  "Return (values inputs outputs) for the top-level model.  Combines:
  (a) `input Real X' and `output Real X' declarations in the top-level
      model's own body, and
  (b) variables exposed via component instances: each `Type instance' line
      contributes that class's inputs as `instance.var' and similarly for
      outputs."
  (let* ((text (mochi--strip-comments (mochi--read-file mo-path)))
         (top-body (or (nth-value 1 (mochi--top-model-block text)) text))
         (class-table (mochi--collect-class-io text))
         (own-inputs (mochi--scan-keyword top-body "input"))
         (own-outputs (mochi--scan-keyword top-body "output")))
    (multiple-value-bind (inst-inputs inst-outputs)
        (mochi--scan-instances top-body class-table)
      (values (append own-inputs inst-inputs)
              (append own-outputs inst-outputs)))))

(defun mochi--model-name-from-source (mo-path)
  "Top-level model name (the *last* `model NAME' in the file)."
  (let* ((text (mochi--strip-comments (mochi--read-file mo-path))))
    (or (nth-value 0 (mochi--top-model-block text)) "Unnamed")))

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

(defun mochi--algebraic-symbols (raw)
  "RAW is the alist under :y from rumoca (algebraic variables)."
  (mapcar (lambda (entry)
            (mochi--mxsym (mochi--name-from-info (cdr entry))))
          raw))

(defun mochi--walk-syms (expr)
  "Walk a Maxima Lisp expression and return all `$'-prefixed user symbols."
  (cond
    ((and (symbolp expr)
          (let ((n (symbol-name expr)))
            (and (plusp (length n)) (char= (char n 0) #\$))))
     (list expr))
    ((consp expr) (mapcan #'mochi--walk-syms expr))
    (t nil)))

(defun mochi--unclassified-syms (residual-exprs known-syms)
  "Return symbols appearing in RESIDUAL-EXPRS that aren't in KNOWN-SYMS.
These are typically connector-flattened outputs (e.g. tank1_q_out) that
rumoca didn't put in :y but which still need to be solved for."
  (let ((all (remove-duplicates (mapcan #'mochi--walk-syms residual-exprs))))
    (set-difference all known-syms)))

(defun $mod_load (path &rest args)
  "Parse a Modelica .mo file and return a Maxima model struct.
With one argument, auto-selects the top-level model (the last `model
NAME ... end NAME;' block in the file).  With a second string argument,
uses that as the explicit model name to compile."
  (let* ((mo-path (if (stringp path) path (string path)))
         (explicit-name (when args
                          (let ((a (first args)))
                            (cond ((stringp a) a)
                                  ((symbolp a) (subseq (symbol-name a) 1))
                                  (t (format nil "~A" a))))))
         (resolved-name (or explicit-name
                            (mochi--model-name-from-source mo-path)))
         (json-text (mochi--run-rumoca mo-path resolved-name))
         (model (cl-json:decode-json-from-string json-text))
         (raw-params (cdr (assoc :p model)))
         (raw-states (cdr (assoc :x model)))
         (raw-algebraics (cdr (assoc :y model)))
         (raw-eqs (cdr (assoc :f--x model)))
         (residual-exprs (mapcar (lambda (e)
                                   (mochi--ast-to-maxima (mochi--get e :residual)))
                                 raw-eqs)))
    (multiple-value-bind (inputs outputs) (mochi--scan-io mo-path)
      (let* ((name resolved-name)
             (param-syms (mapcar (lambda (e) (mochi--mxsym (mochi--name-from-info (cdr e))))
                                 raw-params))
             (state-syms (mapcar (lambda (e) (mochi--mxsym (mochi--name-from-info (cdr e))))
                                 raw-states))
             (deriv-syms (mapcar (lambda (e) (mochi--mxsym (concatenate 'string "der_"
                                                                       (mochi--name-from-info (cdr e)))))
                                 raw-states))
             (input-syms  (mapcar #'mochi--mxsym inputs))
             (output-syms (mapcar #'mochi--mxsym outputs))
             (alg-from-y  (mochi--algebraic-symbols raw-algebraics))
             (known-syms  (append param-syms state-syms deriv-syms
                                  input-syms output-syms alg-from-y))
             ;; Anything in residuals that isn't one of the above must be
             ;; an algebraic variable that rumoca didn't put in :y --
             ;; typically a connector-flattened output.
             (alg-extra   (mochi--unclassified-syms residual-exprs known-syms))
             (algebraics  (append alg-from-y alg-extra)))
        (mochi--mlist
         (list (mochi--mequal '$name name)
               (mochi--mequal '$params (mochi--params-list raw-params))
               (mochi--mequal '$states (mochi--mlist state-syms))
               (mochi--mequal '$derivs (mochi--mlist deriv-syms))
               (mochi--mequal '$algebraics (mochi--mlist algebraics))
               (mochi--mequal '$inputs (mochi--mlist input-syms))
               (mochi--mequal '$outputs (mochi--mlist output-syms))
               (mochi--mequal '$initial (mochi--initial-list raw-states))
               (mochi--mequal '$residuals (mochi--mlist residual-exprs))))))))
