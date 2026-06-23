;;;; mochi-loader.lisp -- Lisp side of the mochi package.
;;;;
;;;; Provides $mod_load: parse a Modelica .mo file via rumoca and build a
;;;; Maxima struct directly, with no Python helper or temp file.
;;;;
;;;; Pipeline:
;;;;   .mo -> rumoca --json -> cl-json:decode-json-from-string
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

(defun mochi--impact-source-roots ()
  "Return source roots from the impact / MoVE convention: `~/.modelica/
library/<libname>/' where each `<libname>' directory contains a
`Modelica/package.mo' (the actual library code).  Returns nil if
~/.modelica/library/ doesn't exist."
  (let ((lib-dir (merge-pathnames ".modelica/library/"
                                  (user-homedir-pathname))))
    (when (probe-file lib-dir)
      (loop for d in (directory (concatenate 'string
                                              (namestring lib-dir) "*/"))
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
  3. Otherwise, fall back to the two tool-managed locations widely
     deployed on Unix systems:
       a. OpenModelica's `$OPENMODELICAHOME/lib/omlibrary/Modelica*/'.
       b. The impact / MoVE convention `~/.modelica/library/<lib>/'.
     Both are scanned; their results are combined.  Returns nil if
     no MSL install is found anywhere — mochi continues to work for
     self-contained models, just without library imports."
  (let* ((extras *mochi-extra-source-roots*)
         (env-path (uiop:getenv "MODELICAPATH"))
         (env (when (and env-path (not (string= env-path "")))
                (uiop:split-string env-path :separator (list #\:)))))
    (cond
      ((or extras env)
       (remove-duplicates (append extras env) :test #'string=))
      (t
       (remove-duplicates (append (mochi--openmodelica-source-roots)
                                  (mochi--impact-source-roots))
                          :test #'string=)))))

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
system).  Source roots from MODELICAPATH are passed via --lib-path
so the model can reference MSL or other library components by their
qualified names (`Modelica.Electrical.Analog.Basic.Resistor', etc.)."
  (let* ((abs (truename mo-path))
         (dir (directory-namestring abs))
         (name (file-namestring abs))
         (source-roots (mochi--source-roots))
         (cmd (append (list (mochi--rumoca-bin) "--json")
                      (when model-name (list "--model" model-name))
                      (mapcan (lambda (root) (list "--lib-path" root))
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

(defun mochi--cref-name (cref-body)
  "CREF-BODY is the unwrapped body of a ComponentReference (i.e. a
plist with :PARTS and :LOCAL).  Each part is `((:IDENT . ((:TEXT . name)
...)) (:SUBS . ...))'.  Join the part texts with `.' to form the
hierarchical Modelica name (most flattened refs already arrive as a
single part whose text contains the dotted name)."
  (let ((parts (mochi--get cref-body :parts)))
    (format nil "~{~A~^.~}"
            (mapcar (lambda (part)
                      (mochi--get (mochi--get part :ident) :text))
                    parts))))

(defun mochi--cref-to-maxima (cref-body)
  "Map an unwrapped ComponentReference body to a Maxima `$name' symbol."
  (mochi--mxsym (mochi--cref-name cref-body)))

(defun mochi--terminal-to-maxima (body)
  "Decode a Terminal (literal) node.  rumoca tags the literal kind in
:TERMINAL--TYPE (a JSON string like \"UnsignedReal\" or \"UnsignedInteger\")
and stores the source text in :TOKEN.TEXT — we parse the text according
to the kind so downstream code gets a real Lisp number / boolean /
string rather than the raw source slice."
  (let* ((kind (mochi--get body :terminal--type))
         (text (mochi--get (mochi--get body :token) :text)))
    (cond
      ((or (string= kind "UnsignedReal") (string= kind "Real"))
       ;; read-from-string on a clean numeric literal returns a Lisp number;
       ;; force float so `1' inside a parameter still works arithmetically.
       (let ((*read-default-float-format* 'double-float))
         (read-from-string text)))
      ((or (string= kind "UnsignedInteger") (string= kind "Integer"))
       (parse-integer text))
      ((string= kind "Bool")
       (cond ((string-equal text "true") t)
             ((string-equal text "false") nil)
             (t (error "mochi: unrecognised Bool terminal text ~S" text))))
      ((string= kind "String")
       text)
      (t
       (error "mochi: unknown terminal_type ~S (text ~S)" kind text)))))

(defun mochi--ast-to-maxima (node)
  "Convert a rumoca JSON AST expression node (cl-json's parsed form)
into a Maxima expression in Lisp form."
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
         (:*component-reference (mochi--cref-to-maxima body))
         (:*terminal            (mochi--terminal-to-maxima body))
         (:*binary              (mochi--binary-to-maxima body))
         (:*unary               (mochi--unary-to-maxima body))
         (:*function-call       (mochi--function-call-to-maxima body))
         (:*parenthesized       (mochi--ast-to-maxima (mochi--get body :inner)))
         (:*array
          ;; Modelica's `{a, b, c}' literal — represent as a Maxima list.
          ;; Matrices use the same node with is_matrix=true; we don't try
          ;; to distinguish here — Maxima lists are fine for the parameter
          ;; / start-value use cases that exercise this path.
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

(defun mochi--simple-residual (fx-entry)
  "FX-ENTRY is `((:*SIMPLE . ((:LHS . lhs-ast) (:RHS . rhs-ast))))'.
Return the residual `lhs - rhs' as a Maxima Lisp expression."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged fx-entry)
    (unless (eq tag :*simple)
      (error "mochi: unexpected fx entry ~S" tag))
    (let ((lhs (mochi--ast-to-maxima (mochi--get body :lhs)))
          (rhs (mochi--ast-to-maxima (mochi--get body :rhs))))
      (list '(mplus) lhs (list '(mtimes) -1 rhs)))))

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
      (:*exp (list '(mexpt) lhs rhs))   ; rumoca tag for `^' in Modelica source.
                                        ; (Modelica's `^' is exponentiation; this
                                        ; isn't related to Maxima's `exp(...)' or
                                        ; matrix-exponential operators.)
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
  ;; rumoca's Unary node carries the operand under :RHS.
  (let* ((op-node (mochi--get body :op))
         (op-tag (caar op-node))
         (arg (mochi--ast-to-maxima (mochi--get body :rhs))))
    (case op-tag
      (:*minus (list '(mtimes) -1 arg))
      (:*plus  arg)
      (:*not   (list '(mnot) arg))
      (otherwise
       (error "mochi: unsupported unary op ~S" op-tag)))))

(defun mochi--function-call-to-maxima (body)
  "Translate a FunctionCall node.  COMP is an unwrapped ComponentReference
(carrying the function name in its first part) and ARGS is a list of
argument AST nodes.  rumoca passes the Modelica-source name through
verbatim — lower-case for builtins (`der', `sin', ...), original case
for user-defined classes."
  (let* ((comp (mochi--get body :comp))
         (fn (mochi--cref-name comp))
         (raw-args (mochi--get body :args))
         (args (mapcar #'mochi--ast-to-maxima raw-args)))
    (cond
      ((string= fn "der")
       ;; der(x) → der_x as a single Maxima symbol.  Reach into the raw AST
       ;; for the ComponentReference's part text so case-inversion is
       ;; applied exactly once — otherwise this `der_*' symbol would
       ;; diverge from the one in (mod_get m 'derivs).
       (let* ((arg-body (cdar (first raw-args)))
              (var-name (mochi--cref-name arg-body)))
         (mochi--mxsym (concatenate 'string "der_" var-name))))
      ((string= fn "pre")
       ;; pre(x) — Modelica's pre-event value.  Wrap in a Maxima function
       ;; symbol so event-time substitution (mochi--strip-pre) can rewrite
       ;; it back to plain x when materialising reset expressions.
       (cons '($pre) args))
      ((string= fn "sin")   (cons '(%sin) args))
      ((string= fn "cos")   (cons '(%cos) args))
      ((string= fn "tan")   (cons '(%tan) args))
      ((string= fn "asin")  (cons '(%asin) args))
      ((string= fn "acos")  (cons '(%acos) args))
      ((string= fn "atan")  (cons '(%atan) args))
      ((string= fn "atan2") (cons '(%atan2) args))
      ((string= fn "sinh")  (cons '(%sinh) args))
      ((string= fn "cosh")  (cons '(%cosh) args))
      ((string= fn "tanh")  (cons '(%tanh) args))
      ((string= fn "exp")   (cons '(%exp) args))
      ((string= fn "log")   (cons '(%log) args))
      ((string= fn "sqrt")  (cons '(%sqrt) args))
      ((string= fn "abs")   (cons '(mabs) args))
      ;; Modelica's `min(...)' / `max(...)' accept either a vararg list
      ;; or a single array argument (`max({a, b, c})').  Maxima's $min /
      ;; $max are vararg, so when we see a single array arg, splat its
      ;; elements into the function call.
      ((or (string= fn "min") (string= fn "max"))
       (let* ((maxima-fn (if (string= fn "min") '($min) '($max)))
              (single-array-p
                (and (= (length args) 1)
                     (consp (first args))
                     (consp (car (first args)))
                     (eq (caar (first args)) 'maxima::mlist)))
              (effective-args
                (if single-array-p (cdr (first args)) args)))
         (cons maxima-fn effective-args)))
      ((string= fn "floor") (cons '($floor) args))
      ((or (string= fn "ceil") (string= fn "ceiling")) (cons '($ceiling) args))
      (t
       (cons (list (mochi--mxsym fn)) args)))))

;; --- Model-name discovery from the .mo source --------------------------
;; rumoca's CLI requires `--model NAME'.  When a caller invokes mod_load
;; without an explicit name, we scan the .mo file for the last
;; `model NAME ... end NAME;' block and pass that as the top-level class.
;; (Causality discovery used to live here too, but rumoca >= 0.7.28 now
;; emits a top-level :u map for inputs and tags :y entries with
;; `causality.Output' — see $mod_load.)

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

(defun mochi--model-name-from-source (mo-path)
  "Top-level model name (the *last* `model NAME' in the file)."
  (let* ((text (mochi--strip-comments (mochi--read-file mo-path))))
    (or (nth-value 0 (mochi--top-model-block text)) "Unnamed")))

;; --- Event extraction (Modelica `when' clauses) ------------------------
;;
;; rumoca surfaces `when' clauses through three top-level keys keyed by
;; synthesised condition-variable names (c0, c1, ...):
;;   :c   — declares the boolean condition variables.
;;   :fc  — { cN: <cond-ast> } — the boolean expression that activates cN.
;;   :fr  — { cN: <Assignment-ast> } — the state assignment that runs when
;;          cN becomes true (the Modelica `reinit' / `when ... then var := ...').
;;
;; Conditions without a matching :fr entry are implicit boundaries from
;; bare `if'-in-equation relations (saturation, sign-flips, ideal diodes)
;; — we still emit a detector so CVODE stops cleanly at the discontinuity.
;;
;; mochi-nonlinear's mod_simulate_nonlinear uses the events list at the
;; Maxima level: each entry is `[detector, reset_eqs, guard, cond_pretty,
;; direction]' where DETECTOR is a real-valued expression whose zero
;; crossings CVODE watches for, RESET_EQS is the list of state-update
;; equations applied at the event time (with `pre(...)' simplified — at
;; event time the pre-event state is exactly what CVODE returns), GUARD
;; is a Maxima expression that's positive iff the original boolean
;; condition holds at the event time (so the loop can reject spurious
;; detections), COND_PRETTY is the original boolean for display, and
;; DIRECTION is -1 / 0 / +1 — the crossing direction implied by the
;; original inequality (`<=`/`<` → -1, `>=`/`>` → +1, `==` → 0), passed
;; through to CVODE's `rootdir' filter so the integrator only fires on
;; the matching direction.  Without this filter the post-reset state
;; (which sits exactly on the event surface — `h := 0' lands on `h <= 0')
;; would re-trigger immediately.
;;
;; A single `when (A and B)' clause produces TWO entries — one detector
;; per primitive inequality.  Both share the same reset and guard, so
;; whichever inequality CVODE detects first triggers a re-evaluation of
;; the full conjunction; only if the guard accepts does the reset fire.

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

(defun mochi--fr-reset-eq (fr-entry)
  "Convert an :fr Assignment node `((:*ASSIGNMENT . ((:COMP . cref)
(:VALUE . value-ast))))' into a Maxima equation `lhs = rhs', with
`pre(...)' references stripped (event-time pre-state equals the value
CVODE returns)."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged fr-entry)
    (unless (eq tag :*assignment)
      (error "mochi: unexpected fr entry tag ~S" tag))
    (let* ((comp (mochi--get body :comp))
           (lhs-sym (mochi--cref-to-maxima comp))
           (value-mx (mochi--strip-pre
                      (mochi--ast-to-maxima (mochi--get body :value)))))
      (list '(mequal) lhs-sym value-mx))))

(defun mochi--cond-to-events (cond-node reset-eqs)
  "Build event tuples for a single boolean condition AST.  Emits one
tuple per primitive comparison reachable from the root (so any leaf
inequality firing forces re-evaluation of the conjunction).  RESET-EQS
is the Maxima reset list to attach to each tuple; pass an empty list
for detector-only events."
  (let ((detectors (mochi--cond-to-detectors cond-node))
        (guard (mochi--cond-to-guard cond-node))
        (cond-pretty (mochi--cond-pretty cond-node)))
    (mapcar (lambda (det)
              (mochi--mlist (list (first det)
                                  reset-eqs
                                  guard
                                  cond-pretty
                                  (second det))))
            detectors)))

(defun errcatch-mochi (thunk)
  "Run THUNK, returning its result on success or NIL on any error.
Used to skip :fc entries whose condition AST isn't shaped like a
real-valued boundary detector (e.g. a bare Boolean discrete-mode flag
like PID's `local_reset')."
  (handler-case (funcall thunk)
    (error () nil)))

(defun mochi--cond-to-events-safe (cond-node reset-eqs)
  "Wrap mochi--cond-to-events to drop conditions that aren't comparison-
shaped — they can't be turned into a real-valued zero-crossing
detector."
  (errcatch-mochi (lambda () (mochi--cond-to-events cond-node reset-eqs))))

(defun mochi--extract-events (raw-c raw-fc raw-fr)
  "Build the events list for the model struct.  For each condition
variable cN declared in :c, look up its boolean expression in :fc.
If :fr has a matching Assignment, attach it as the reset.  Otherwise
emit a detector-only event so CVODE's rootfinder still stops cleanly
at the discontinuity (saturation, sign-flips, ideal diodes).

Returns a flattened Maxima list of `[detector, reset_eqs, guard,
cond_pretty, direction]' tuples."
  (declare (ignore raw-c))
  (let ((events '()))
    (dolist (fc-entry raw-fc)
      (let* ((cname (car fc-entry))
             (cond-node (cdr fc-entry))
             (fr-node (cdr (assoc cname raw-fr)))
             (reset-eqs (if fr-node
                            (mochi--mlist (list (mochi--fr-reset-eq fr-node)))
                            (mochi--mlist '())))
             (tuples (mochi--cond-to-events-safe cond-node reset-eqs)))
        (when tuples
          (setf events (append events tuples)))))
    (mochi--mlist events)))

;; --- Build the Maxima struct -------------------------------------------

(defun mochi--build-init-bindings (raw-fx-init)
  "Walk RAW-FX-INIT (the :fx--init list from rumoca) and return a hash
table mapping the LHS variable name (string, with `.' between component
parts) to its RHS AST node.  rumoca surfaces `final parameter T =
max(Td/Nd, ...)' style binding equations here rather than as the
parameter's `start' value, so mochi--start-value uses this table as a
fallback when `start' is `Empty'."
  (let ((h (make-hash-table :test 'equal)))
    (dolist (entry raw-fx-init h)
      (multiple-value-bind (tag body) (mochi--unwrap-tagged entry)
        (when (eq tag :*simple)
          (let* ((lhs-node (mochi--get body :lhs))
                 (lhs-body (cdar lhs-node))
                 (name (mochi--cref-name lhs-body))
                 (rhs (mochi--get body :rhs)))
            (setf (gethash name h) rhs)))))))

(defun mochi--start-value (info init-bindings)
  "INFO is a parameter or state info alist.  Return the start-value:
a number when rumoca emitted a literal default, or a Maxima expression
(in Lisp form) when the default is computed (e.g. MSL parameters
declared as `Add.k1 = +1' or `D.T = max({Td/Nd, 100*1e-15})').

INIT-BINDINGS is the hash table from mochi--build-init-bindings: when
`start' is `Empty' (rumoca's sentinel for `no literal default') but the
variable has a binding equation in :fx--init, we fall back to that —
this is how MSL's `final parameter T = max(...)' calculated parameters
reach the params list.

Caller is responsible for resolving the expression — usually by
substituting other parameter values into it iteratively until it
reduces to a number."
  (let ((start (mochi--get info :start)))
    (cond
      ((null start) 0)
      ;; \"Empty\" (a JSON string, not a tagged enum) means no literal
      ;; default; consult init-bindings before giving up to zero.
      ((and (stringp start) (string= start "Empty"))
       (let* ((name (mochi--name-from-info info))
              (binding (gethash name init-bindings)))
         (if binding (mochi--ast-to-maxima binding) 0)))
      (t (mochi--ast-to-maxima start)))))

(defun mochi--name-from-info (info)
  "INFO is a parameter or state info alist with a :NAME field."
  (mochi--get info :name))

(defun mochi--params-list (raw init-bindings)
  "RAW is the alist under :p from rumoca; INIT-BINDINGS comes from
mochi--build-init-bindings."
  (mochi--mlist
   (mapcar (lambda (entry)
             (let* ((info (cdr entry))
                    (name (mochi--name-from-info info))
                    (val (mochi--start-value info init-bindings)))
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

(defun mochi--initial-list (raw init-bindings)
  (mochi--mlist
   (mapcar (lambda (entry)
             (let* ((info (cdr entry))
                    (name (mochi--name-from-info info))
                    (val (mochi--start-value info init-bindings)))
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

(defun mochi--causality-tag (info)
  "Return the cl-json keyword for the causality tag of a variable info
entry (e.g. :*INPUT, :*OUTPUT), or NIL if causality is `Empty'.  rumoca
emits causality as either the JSON string \"Empty\" or a single-key
object like {\"Output\": {...}}; cl-json decodes the latter to an alist
((:*OUTPUT . inner)), which we inspect with caar."
  (let ((c (cdr (assoc :causality info))))
    (when (consp c) (caar c))))

(defun mochi--info-symbol (info)
  "Build the Maxima `$name' symbol for a variable info entry."
  (mochi--mxsym (mochi--name-from-info info)))

(defun mochi--top-level-name-p (name)
  "True iff NAME has no `.' — heuristic for `top-level model variable'
after rumoca's flattening pass.  Dotted names are inherited or instance-
scoped: `tank1.q_out' came from the inner Tank class, not the outer
TwoTanks user interface."
  (not (find #\. name)))

(defun mochi--partition-y-by-causality (raw-y)
  "Split RAW-Y (the alist under :y from rumoca) into (values outputs
algebraics), each a list of Maxima symbols.  An entry counts as an
output only if its causality tag is `Output' AND its (pre-flatten) name
is top-level (contains no `.'): rumoca preserves the `output' attribute
through `extends' and component instances, but instance-internal
outputs like `tank1.q_out' aren't part of the wrapping model's external
interface — they're algebraic from the caller's perspective.  Anything
not classified as an output is treated as algebraic."
  (let ((outputs '())
        (algebraics '()))
    (dolist (entry raw-y)
      (let* ((info (cdr entry))
             (name (mochi--name-from-info info))
             (sym (mochi--mxsym name)))
        (cond
          ((and (eq (mochi--causality-tag info) :*output)
                (mochi--top-level-name-p name))
           (push sym outputs))
          (t (push sym algebraics)))))
    (values (nreverse outputs) (nreverse algebraics))))

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
         (raw-inputs (cdr (assoc :u model)))
         (raw-y      (cdr (assoc :y model)))
         (raw-eqs     (cdr (assoc :fx model)))
         (raw-fx-init (cdr (assoc :fx--init model)))
         (raw-c       (cdr (assoc :c model)))
         (raw-fc      (cdr (assoc :fc model)))
         (raw-fr      (cdr (assoc :fr model)))
         (init-bindings (mochi--build-init-bindings raw-fx-init))
         ;; Each :fx entry is `((:*SIMPLE . ((:LHS . ...) (:RHS . ...))))'.
         ;; The residual is `lhs - rhs' as a Maxima expression — the rest
         ;; of mochi assumes `f(...) = 0' form.
         (residual-exprs (mapcar #'mochi--simple-residual raw-eqs)))
    (multiple-value-bind (output-syms alg-from-y)
        (mochi--partition-y-by-causality raw-y)
      (let* ((name resolved-name)
             (param-syms (mapcar (lambda (e) (mochi--info-symbol (cdr e)))
                                 raw-params))
             (state-syms (mapcar (lambda (e) (mochi--info-symbol (cdr e)))
                                 raw-states))
             (deriv-syms (mapcar (lambda (e) (mochi--mxsym (concatenate 'string "der_"
                                                                       (mochi--name-from-info (cdr e)))))
                                 raw-states))
             (input-syms (mapcar (lambda (e) (mochi--info-symbol (cdr e)))
                                 raw-inputs))
             (known-syms  (append param-syms state-syms deriv-syms
                                  input-syms output-syms alg-from-y))
             ;; Catch any algebraic symbol that appears in residuals but
             ;; isn't in :y (e.g. a connector-flattened internal signal
             ;; that didn't make it into the variable list).
             (alg-extra   (mochi--unclassified-syms residual-exprs known-syms))
             (algebraics  (append alg-from-y alg-extra)))
        (mochi--mlist
         (list (mochi--mequal '$name name)
               (mochi--mequal '$params (mochi--params-list raw-params init-bindings))
               (mochi--mequal '$states (mochi--mlist state-syms))
               (mochi--mequal '$derivs (mochi--mlist deriv-syms))
               (mochi--mequal '$algebraics (mochi--mlist algebraics))
               (mochi--mequal '$inputs (mochi--mlist input-syms))
               (mochi--mequal '$outputs (mochi--mlist output-syms))
               (mochi--mequal '$initial (mochi--initial-list raw-states init-bindings))
               (mochi--mequal '$residuals (mochi--mlist residual-exprs))
               (mochi--mequal '$events (mochi--extract-events raw-c raw-fc raw-fr))))))))
