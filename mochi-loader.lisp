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

(defvar *mochi-extra-source-roots* '()
  "Extra source-root directories pushed by mod_add_source_root / set by
mod_set_source_root.  Searched before MODELICAPATH and the auto-
discovery fallbacks.")

(defun mochi--openmodelica-source-roots ()
  "If OpenModelica is installed, return its versioned MSL directories
as candidate source roots.  OM's documented convention:
$OPENMODELICAHOME/lib/omlibrary/Modelica*/ (a `Modelica X.Y.Z' folder
per installed MSL version, the user-visible result of OM's package
manager).  Returns nil when OM isn't installed."
  (let* ((om-home (or (uiop:getenv "OPENMODELICAHOME")
                      "/opt/openmodelica"))
         (lib-dir (concatenate 'string om-home "/lib/omlibrary/")))
    (when (probe-file lib-dir)
      (loop for d in (directory (concatenate 'string lib-dir "Modelica*/"))
            when (probe-file (merge-pathnames "Modelica/package.mo" d))
              collect (namestring d)))))

(defun mochi--source-roots ()
  "Source-root directories that rumoca should search beyond the input
file's own directory.  Modelica Language Specification §13.2.4
defines MODELICAPATH as a colon-separated (Unix) / semicolon-
separated (Windows) list of directories that hold Modelica libraries
— it is multi-path by design, and `--source-root' is rumoca's way of
honouring the same idea.

Resolution rules — explicit configuration always wins:

  1. If the user has called mod_set_source_root or mod_add_source_root,
     use *mochi-extra-source-roots*.  Combined with MODELICAPATH if
     both are set (the API explicitly extends the env var).
  2. Otherwise, if MODELICAPATH is set, use it (parsed as the standard
     colon-separated list).
  3. Otherwise, fall back to OpenModelica's `$OPENMODELICAHOME/lib/
     omlibrary/Modelica*/' since that's the one tool-managed location
     widely deployed on Unix systems.  Returns nil if no MSL install
     is found anywhere — mochi continues to work for self-contained
     models, just without library imports."
  (let* ((extras *mochi-extra-source-roots*)
         (env-path (uiop:getenv "MODELICAPATH"))
         (env (when (and env-path (not (string= env-path "")))
                (uiop:split-string env-path :separator (list #\:)))))
    (cond
      ((or extras env)
       (remove-duplicates (append extras env) :test #'string=))
      (t
       (mochi--openmodelica-source-roots)))))

(defun $mod_set_source_root (&rest paths)
  "mod_set_source_root(path1, path2, ...) — replace mochi's source-root
list (the directories rumoca searches for `Modelica.X.Y.Z' references).
Pass zero arguments to clear and fall back to MODELICAPATH / auto-
discovery only.  Each PATH should point at the parent of a `Modelica/'
directory (the same convention as rumoca's --source-root)."
  (setf *mochi-extra-source-roots*
        (mapcar (lambda (p) (if (stringp p) p (string p))) paths))
  '$done)

(defun $mod_add_source_root (path)
  "mod_add_source_root(path) — push a directory onto mochi's source-root
list.  See mod_set_source_root for the path convention."
  (push (if (stringp path) path (string path))
        *mochi-extra-source-roots*)
  '$done)

(defun $mod_source_roots ()
  "Return the current source-root list as a Maxima list of strings."
  (mochi--mlist (mochi--source-roots)))

(defun mochi--run-rumoca (mo-path &optional model-name)
  "Run rumoca on MO-PATH and return the JSON output as a string.
If MODEL-NAME is given, pass --model so rumoca picks that class as the
top-level (required when the file contains more than one model, e.g.
when reusable component classes are defined alongside the top-level
system).  Source roots from MODELICAPATH are passed via --source-root
so the model can reference MSL or other library components by their
qualified names (`Modelica.Electrical.Analog.Basic.Resistor', etc.)."
  (let* ((abs (truename mo-path))
         (dir (directory-namestring abs))
         (name (file-namestring abs))
         (source-roots (mochi--source-roots))
         (cmd (append (list (mochi--rumoca-bin) "compile" "--json")
                      (when model-name (list "--model" model-name))
                      (mapcan (lambda (root) (list "--source-root" root))
                              source-roots)
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
         (:*array
          ;; Modelica's `{a, b, c}' literal — represent as a Maxima list.
          ;; (Modelica's matrices use the same node with is_matrix=true;
          ;; we don't try to distinguish here — Maxima lists are fine for
          ;; the use cases that show up via parameter expressions.)
          (cons '(mlist) (mapcar #'mochi--ast-to-maxima
                                  (mochi--get body :elements))))
         (:*if
          ;; Modelica `if cond then val ... else else_val'.  rumoca emits
          ;; an If with one or more (cond, value) branches plus an else.
          ;; Translate to Maxima's mcond form: `((mcond) c1 v1 ... t e)'.
          ;; When cond resolves to a constant (typical for parameter-
          ;; gated paths in MSL blocks like pid.D.zeroGain), the
          ;; substitution + simplifier collapses to the right branch.
          (let* ((branches (mochi--get body :branches))
                 (else-branch (mochi--get body :else--branch))
                 (pairs (mapcan (lambda (br)
                                  (list (mochi--ast-to-maxima (first br))
                                        (mochi--ast-to-maxima (second br))))
                                branches)))
            (append (list '(mcond))
                    pairs
                    (list t (mochi--ast-to-maxima else-branch)))))
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
      ;; Comparison operators that show up inside If branches in
      ;; MSL blocks (e.g. pid.D.zeroGain = (Td < 1e-12)).
      (:*lt  (list '(mlessp)    lhs rhs))
      (:*le  (list '(mleqp)     lhs rhs))
      (:*gt  (list '(mgreaterp) lhs rhs))
      (:*ge  (list '(mgeqp)     lhs rhs))
      (:*ne  (list '(mnotequal) lhs rhs))
      (:*and (list '(mand)      lhs rhs))
      (:*or  (list '(mor)       lhs rhs))
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
      ;; Modelica's `min(...)' / `max(...)' accept either a vararg list
      ;; or a single array argument (`max({a, b, c})').  Maxima's $min /
      ;; $max are vararg, so when we see a single array arg, splat its
      ;; elements into the function call.
      ((or (string= fn "Min") (string= fn "Max"))
       (let* ((maxima-fn (if (string= fn "Min") '($min) '($max)))
              (single-array-p
                (and (= (length args) 1)
                     (consp (first args))
                     (consp (car (first args)))
                     (eq (caar (first args)) 'maxima::mlist)))
              (effective-args
                (if single-array-p (cdr (first args)) args)))
         (cons maxima-fn effective-args)))
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

(defun mochi--whitespace-char-p (c)
  (member c '(#\Space #\Tab)))

(defun mochi--skip-ws (text pos)
  "Advance past horizontal whitespace (space and tab)."
  (loop while (and (< pos (length text))
                   (mochi--whitespace-char-p (char text pos)))
        do (incf pos))
  pos)

(defun mochi--scan-keyword (text keyword)
  "Find every occurrence of `<keyword> Real <name>' allowing arbitrary
horizontal whitespace between the tokens, and return the list of
<name> strings.  KEYWORD must be at a token boundary (preceded by
whitespace, start-of-string, or `;')."
  (let ((kw-len (length keyword))
        (names '())
        (start 0)
        (len (length text)))
    (loop
      (let ((pos (search keyword text :start2 start)))
        (unless pos (return (nreverse names)))
        (let ((boundary-ok
                (or (zerop pos)
                    (member (char text (1- pos))
                            '(#\Space #\Tab #\Newline #\Return #\;)))))
          (cond
            ((not boundary-ok)
             (setf start (1+ pos)))
            (t
             (let* ((after-kw (mochi--skip-ws text (+ pos kw-len)))
                    (real-end (+ after-kw 4))
                    (real-ok
                      (and (<= real-end len)
                           (string= text "Real"
                                    :start1 after-kw :end1 real-end)
                           (or (= real-end len)
                               (mochi--whitespace-char-p
                                (char text real-end))))))
               (cond
                 ((not real-ok)
                  (setf start (+ pos kw-len)))
                 (t
                  (let* ((id-start (mochi--skip-ws text real-end))
                         (id-end id-start))
                    (loop while (and (< id-end len)
                                     (or (alphanumericp (char text id-end))
                                         (char= (char text id-end) #\_)))
                          do (incf id-end))
                    (when (> id-end id-start)
                      (push (subseq text id-start id-end) names))
                    (setf start id-end))))))))))))

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

(defun mochi--scan-extends (body)
  "Return the list of class names that BODY's `extends' declarations
inherit from.  Modelica syntax is `extends ParentName;' or
`extends ParentName(modifications);'.  We don't care about the
modifications here (rumoca has already applied them when emitting the
flattened JSON) — we just need the parent class names so we can
inherit their `input'/`output' declarations."
  (let ((parents '())
        (search-from 0)
        (len (length body)))
    (loop
      (let ((pos (search "extends " body :start2 search-from)))
        (unless pos (return (nreverse parents)))
        ;; Make sure the previous character is whitespace / start-of-body
        ;; (so we don't match an identifier that happens to end in
        ;; `extends').
        (cond
          ((or (zerop pos)
               (member (char body (1- pos))
                       '(#\Space #\Tab #\Newline #\Return #\;)))
           (let* ((name-start (+ pos 8))
                  (name-end (or (loop for i from name-start below len
                                      while (or (alphanumericp (char body i))
                                                (char= (char body i) #\_)
                                                (char= (char body i) #\.))
                                      finally (return i))
                                len)))
             (when (> name-end name-start)
               (push (subseq body name-start name-end) parents))
             (setf search-from (max (1+ pos) name-end))))
          (t (setf search-from (1+ pos))))))))

(defun mochi--collect-class-io (text)
  "Walk every `model NAME ... end NAME;' block in TEXT and return an alist
mapping the class name (string) to (list inputs outputs extends-parents),
where each list element is a list of identifier name strings.  The
`extends-parents' list lets the IO scanner walk up the inheritance
chain to inherit `input'/`output' declarations from base classes."
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
                                (mochi--scan-keyword body "output")
                                (mochi--scan-extends body))
                          classes))
                  (setf search-from (+ end-pos (length terminator))))
                 (t (setf search-from name-end)))))
            (t (setf search-from (1+ kw-pos)))))
        (when (>= search-from len) (return (nreverse classes)))))))

(defun mochi--inherited-io (class-name class-table seen)
  "Recursively walk CLASS-NAME's extends parents in CLASS-TABLE,
returning (values inherited-inputs inherited-outputs).  SEEN is the
set of class names already visited (to break cycles)."
  (let ((entry (assoc class-name class-table :test #'string=))
        (inputs '())
        (outputs '()))
    (when (and entry (not (member class-name seen :test #'string=)))
      (let ((parents (fourth entry))
            (seen (cons class-name seen)))
        (dolist (parent parents)
          (let ((parent-entry (assoc parent class-table :test #'string=)))
            (when parent-entry
              (setf inputs  (append inputs  (second parent-entry)))
              (setf outputs (append outputs (third  parent-entry)))
              (multiple-value-bind (gi go)
                  (mochi--inherited-io parent class-table seen)
                (setf inputs  (append inputs  gi))
                (setf outputs (append outputs go))))))))
    (values inputs outputs)))

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
      model's own body,
  (b) inputs/outputs inherited from each parent listed in the top-level
      model's `extends' clauses (transitively), and
  (c) variables exposed via component instances: each `Type instance' line
      contributes that class's inputs as `instance.var' and similarly for
      outputs."
  (let* ((text (mochi--strip-comments (mochi--read-file mo-path)))
         (top-name (nth-value 0 (mochi--top-model-block text)))
         (top-body (or (nth-value 1 (mochi--top-model-block text)) text))
         (class-table (mochi--collect-class-io text))
         (own-inputs (mochi--scan-keyword top-body "input"))
         (own-outputs (mochi--scan-keyword top-body "output")))
    (multiple-value-bind (inh-inputs inh-outputs)
        (mochi--inherited-io top-name class-table nil)
      (multiple-value-bind (inst-inputs inst-outputs)
          (mochi--scan-instances top-body class-table)
        (values (append own-inputs inh-inputs inst-inputs)
                (append own-outputs inh-outputs inst-outputs))))))

(defun mochi--model-name-from-source (mo-path)
  "Top-level model name (the *last* `model NAME' in the file)."
  (let* ((text (mochi--strip-comments (mochi--read-file mo-path))))
    (or (nth-value 0 (mochi--top-model-block text)) "Unnamed")))

;; --- Event extraction (Modelica `when' clauses) ------------------------
;;
;; rumoca surfaces `when' clauses through two extra top-level keys:
;;   :f--z — list of discrete state-update equations.  Each entry is
;;           {lhs: var, rhs: If(branches=[[Edge(cond), new_val]], else=var)}
;;           ("set var to new_val on the rising edge of cond, otherwise
;;            keep var unchanged").
;;   :f--c — the boolean condition expressions (we don't strictly need
;;           these, since they're embedded in the f_z If branches).
;;
;; mochi-nonlinear's mod_simulate_nonlinear uses the events list at the
;; Maxima level: each entry is `[detector, reset_eqs, guard, cond_pretty]'
;; where DETECTOR is a real-valued expression whose zero crossings CVODE
;; watches for, RESET_EQS is the list of state-update equations applied
;; at the event time (with `pre(...)' simplified — at event time the
;; pre-event state is exactly what CVODE returns), GUARD is a Maxima
;; expression that's positive iff the original boolean condition holds
;; at the event time (so the loop can reject spurious detections), and
;; COND_PRETTY is the original boolean for display.
;;
;; A single `when (A and B)' clause produces TWO entries — one detector
;; per primitive inequality.  Both share the same reset and guard, so
;; whichever inequality CVODE detects first triggers a re-evaluation of
;; the full conjunction; only if the guard accepts does the reset fire.

(defun mochi--strip-edge (node)
  "If NODE is a BuiltinCall(Edge, [arg]), return ARG.  Otherwise return
   NODE unchanged."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged node)
    (cond
      ((and (eq tag :*builtin-call)
            (string= (mochi--get body :function) "Edge"))
       (first (mochi--get body :args)))
      (t node))))

(defun mochi--strip-pre (expr)
  "Walk a Maxima Lisp expression and replace `($pre x)' (Modelica's
   `pre(x)' builtin) with just `x'.  In an event-time substitution
   context the pre-event state is exactly what CVODE hands back, so
   `pre(v)' ≡ `v'."
  (cond
    ((atom expr) expr)
    ((and (consp (car expr))
          (eq (caar expr) '$pre)
          (consp (cdr expr)))
     (mochi--strip-pre (cadr expr)))
    (t (cons (car expr) (mapcar #'mochi--strip-pre (cdr expr))))))

(defun mochi--ast-binary-op-tag (node)
  "Return the binary-op tag (e.g. :*le, :*and) for a Binary AST node,
   or nil if NODE isn't a Binary."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged node)
    (when (eq tag :*binary)
      (caar (mochi--get body :op)))))

(defun mochi--cond-unary-not-body (node)
  "If NODE is a Unary with op = Not, return its body alist.  Else nil."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged node)
    (when (eq tag :*unary)
      (let ((op-tag (caar (mochi--get body :op))))
        (when (eq op-tag :*not) body)))))

(defun mochi--lhs-rhs-mx (node)
  "NODE is a Binary; return (values LHS-MX RHS-MX) — the operands as
   Maxima Lisp expressions."
  (let ((body (cdar node)))
    (values (mochi--ast-to-maxima (mochi--get body :lhs))
            (mochi--ast-to-maxima (mochi--get body :rhs)))))

(defun mochi--mx-sub (a b)
  "Maxima `a - b' in Lisp form."
  (list '(mplus) a (list '(mtimes) -1 b)))

(defun mochi--cond-to-detectors (cond-node)
  "Walk a boolean-condition AST and return a list of `(detector dir)'
   pairs covering every primitive comparison reachable from the root.
   DIR is -1 (falling crossing activates the inequality), +1 (rising),
   or 0 (any).  All conjuncts AND disjuncts are visited so any
   inequality firing triggers re-evaluation of the guard."
  (let ((tag (mochi--ast-binary-op-tag cond-node)))
    (case tag
      ((:*le :*lt)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list (list (mochi--mx-sub lhs rhs) -1))))
      ((:*ge :*gt)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list (list (mochi--mx-sub lhs rhs) +1))))
      ((:*eq)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list (list (mochi--mx-sub lhs rhs) 0))))
      ((:*and :*or)
       (let ((body (cdar cond-node)))
         (append (mochi--cond-to-detectors (mochi--get body :lhs))
                 (mochi--cond-to-detectors (mochi--get body :rhs)))))
      (otherwise
       (let ((not-body (mochi--cond-unary-not-body cond-node)))
         (cond
           (not-body
            ;; not(c): flip directions.
            (mapcar (lambda (det)
                      (list (first det) (- (second det))))
                    (mochi--cond-to-detectors (mochi--get not-body :rhs))))
           (t
            (error "mochi: unsupported event condition AST tag ~S"
                   (and (consp cond-node) (caar cond-node))))))))))

(defun mochi--cond-to-guard (cond-node)
  "Convert a boolean condition AST to a real-valued Maxima expression
   that is > 0 iff the condition holds.  Caller checks `> 0' to decide
   whether to fire the reset."
  (let ((tag (mochi--ast-binary-op-tag cond-node)))
    (case tag
      ((:*le :*lt)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         ;; lhs <= rhs  ⇔  rhs - lhs ≥ 0
         (mochi--mx-sub rhs lhs)))
      ((:*ge :*gt)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (mochi--mx-sub lhs rhs)))
      ((:*eq)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         ;; -|diff|^2 — zero only at equality, negative otherwise.
         (let ((diff (mochi--mx-sub lhs rhs)))
           (list '(mtimes) -1 (list '(mexpt) diff 2)))))
      ((:*and)
       (let ((body (cdar cond-node)))
         (list '($min)
               (mochi--cond-to-guard (mochi--get body :lhs))
               (mochi--cond-to-guard (mochi--get body :rhs)))))
      ((:*or)
       (let ((body (cdar cond-node)))
         (list '($max)
               (mochi--cond-to-guard (mochi--get body :lhs))
               (mochi--cond-to-guard (mochi--get body :rhs)))))
      (otherwise
       (let ((not-body (mochi--cond-unary-not-body cond-node)))
         (cond
           (not-body
            (list '(mtimes) -1
                  (mochi--cond-to-guard (mochi--get not-body :rhs))))
           (t
            (error "mochi: unsupported event condition AST tag ~S"
                   (and (consp cond-node) (caar cond-node))))))))))

(defun mochi--cond-pretty (cond-node)
  "Convert a boolean condition AST to a Maxima expression preserving
   the original op shape, for display via mod_print."
  (let ((tag (mochi--ast-binary-op-tag cond-node)))
    (case tag
      ((:*le)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mleqp) lhs rhs)))
      ((:*lt)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mlessp) lhs rhs)))
      ((:*ge)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mgeqp) lhs rhs)))
      ((:*gt)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mgreaterp) lhs rhs)))
      ((:*eq)
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mequal) lhs rhs)))
      ((:*and)
       (let ((body (cdar cond-node)))
         (list '(mand)
               (mochi--cond-pretty (mochi--get body :lhs))
               (mochi--cond-pretty (mochi--get body :rhs)))))
      ((:*or)
       (let ((body (cdar cond-node)))
         (list '(mor)
               (mochi--cond-pretty (mochi--get body :lhs))
               (mochi--cond-pretty (mochi--get body :rhs)))))
      (otherwise
       (let ((not-body (mochi--cond-unary-not-body cond-node)))
         (cond
           (not-body
            (list '(mnot) (mochi--cond-pretty (mochi--get not-body :rhs))))
           (t
            ;; Fall back to plain conversion (best effort).
            (mochi--ast-to-maxima cond-node))))))))

(defun mochi--fz-to-events (fz-entry)
  "Convert one rumoca f_z entry to a list of event tuples
   `(detector reset-eqs guard cond-pretty)' — one per primitive
   detector in the boolean condition.  All entries share the same
   reset and guard.  Returns nil if the entry doesn't match the
   expected If(branches=[[Edge(cond), new_val]], else=...) shape."
  (let* ((lhs-name (mochi--get fz-entry :lhs))
         (rhs-node (mochi--get fz-entry :rhs)))
    (multiple-value-bind (tag if-body) (mochi--unwrap-tagged rhs-node)
      (when (eq tag :*if)
        (let* ((branches (mochi--get if-body :branches))
               (first-branch (first branches))
               (raw-cond (first first-branch))
               (new-value-node (second first-branch))
               (cond-node (mochi--strip-edge raw-cond))
               (detectors (mochi--cond-to-detectors cond-node))
               (guard (mochi--cond-to-guard cond-node))
               (cond-pretty (mochi--cond-pretty cond-node))
               (new-value-mx (mochi--strip-pre
                              (mochi--ast-to-maxima new-value-node)))
               (reset-eq (list '(mequal) (mochi--mxsym lhs-name) new-value-mx))
               (reset-eqs (mochi--mlist (list reset-eq))))
          (mapcar (lambda (det)
                    (mochi--mlist (list (first det)
                                        reset-eqs
                                        guard
                                        cond-pretty)))
                  detectors))))))

(defun mochi--extract-events (raw-fz)
  "Build the events list for the model struct.  Maps every f_z entry
   through mochi--fz-to-events (which may produce multiple tuples per
   entry) and flattens into a single Maxima list."
  (mochi--mlist (mapcan #'mochi--fz-to-events raw-fz)))

;; --- Build the Maxima struct -------------------------------------------

(defun mochi--start-value (info)
  "INFO is a parameter or state info alist.  Return the start-value:
a number when rumoca emitted a literal default, or a Maxima expression
(in Lisp form) when the default is computed (e.g. MSL parameters
declared as `Add.k1 = +1' or `D.T = max({Td/Nd, 100*1e-15})').
Caller is responsible for resolving the expression — usually by
substituting other parameter values into it iteratively until it
reduces to a number."
  (let ((start (mochi--get info :start)))
    (cond
      ((null start) 0)
      ((and (consp start) (consp (car start)) (eq (caar start) :*literal))
       (mochi--literal-value (cdar start)))
      (t (mochi--ast-to-maxima start)))))

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
         (raw-fz (cdr (assoc :f--z model)))
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
             ;; rumoca's :y list is "everything else" from its perspective.
             ;; That sometimes includes variables that mochi has already
             ;; classified as inputs or outputs by walking the source --
             ;; typically `inst.Vin' style names in instance-composed
             ;; models, where the inner class declared `input Real Vin'.
             ;; Subtract so they don't end up double-classified (which
             ;; would make the residual system underdetermined when we
             ;; solve for the algebraics).
             (alg-from-y-raw (mochi--algebraic-symbols raw-algebraics))
             (alg-from-y  (set-difference alg-from-y-raw
                                          (append input-syms output-syms)))
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
               (mochi--mequal '$residuals (mochi--mlist residual-exprs))
               (mochi--mequal '$events (mochi--extract-events raw-fz))))))))
