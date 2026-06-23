;;;; mochi-loader.lisp -- Lisp side of the mochi package.
;;;;
;;;; Provides $mod_load: parse a Modelica .mo file via rumoca and build a
;;;; Maxima struct directly, with no Python helper or temp file.
;;;;
;;;; Pipeline:
;;;;   .mo -> rumoca compile --emit flat-json -> cl-json:decode-json-from-string
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
system).  Source roots from MODELICAPATH are passed via --source-root
so the model can reference MSL or other library components by their
qualified names (`Modelica.Electrical.Analog.Basic.Resistor', etc.).

The `compile' subcommand + `--emit flat-json' shape was settled in
rumoca v0.9.x — see README's Compatibility table for the current pin."
  (let* ((abs (truename mo-path))
         (dir (directory-namestring abs))
         (name (file-namestring abs))
         (source-roots (mochi--source-roots))
         (cmd (append (list (mochi--rumoca-bin) "compile" "--emit" "flat-json")
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

;; --- AST walker (rumoca v0.9.x flat-json shape) -----------------------
;;
;; Tagged-enum node wrappers use cl-json's `((:*TAG . body))' encoding.
;; Each body is an alist:
;;
;;   VarRef        body: ((:NAME . ((:NAME . "dotted") (:COMPONENT--REF . ...) ...))
;;                        (:SUBSCRIPTS . ...) (:SPAN . ...))
;;                 — `body.name.name' is the flat dotted Modelica name.
;;   Literal       body: ((:VALUE . ((:REAL . 1.0) | (:INTEGER . 0) |
;;                                   (:BOOL . t) | (:STRING . "..."))) (:SPAN . ...))
;;   Binary        body: ((:OP . "Mul") (:LHS . ...) (:RHS . ...) (:SPAN . ...))
;;   Unary         body: ((:OP . "Minus") (:RHS . ...) (:SPAN . ...))
;;   BuiltinCall   body: ((:FUNCTION . "Der") (:ARGS . (...)) (:SPAN . ...))
;;   If            body: ((:BRANCHES . ((cond1 val1) ...)) (:ELSE--BRANCH . ...))
;;   Array         body: ((:ELEMENTS . (...)) (:SPAN . ...))
;;   Reinit        body: ((:STATE . "name") (:VALUE . ...))
;;                 — only seen inside when_clauses[*].equations; handled by
;;                   the event extractor, not by mochi--ast-to-maxima.
;;
;; Notable v0.9.x changes vs v0.7.x: `:op' is a plain string (no
;; tagged-enum wrap), BuiltinCall function names are Pascal-cased
;; (\"Der\", \"Pre\", \"Sin\"), and there is no ComponentReference
;; wrapper node — flat dotted names live directly on VarRef.

(defun mochi--varref-name (body)
  "Pull the dotted Modelica name from a VarRef body."
  (mochi--get (mochi--get body :name) :name))

(defun mochi--varref-to-maxima (body)
  (mochi--mxsym (mochi--varref-name body)))

(defun mochi--literal-to-maxima (body)
  "Decode a Literal node.  body.value is a single-key tagged enum:
{Real: f}, {Integer: i}, {Boolean: b}, {String: s}."
  (let* ((value (mochi--get body :value))
         (tag (caar value))
         (raw (cdar value)))
    (case tag
      (:*real    (coerce raw 'double-float))
      (:*integer raw)
      (:*boolean raw)
      (:*string  raw)
      (otherwise (error "mochi: unknown literal value tag ~S" tag)))))

(defun mochi--binary-to-maxima (body)
  (let* ((op  (mochi--get body :op))
         (lhs (mochi--ast-to-maxima (mochi--get body :lhs)))
         (rhs (mochi--ast-to-maxima (mochi--get body :rhs))))
    (cond
      ((string= op "Add") (list '(mplus) lhs rhs))
      ((string= op "Sub") (list '(mplus) lhs (list '(mtimes) -1 rhs)))
      ((string= op "Mul") (list '(mtimes) lhs rhs))
      ((string= op "Div") (list '(mtimes) lhs (list '(mexpt) rhs -1)))
      ;; rumoca tags Modelica's `^' as either Pow or Exp historically;
      ;; both map to Maxima's mexpt.  Neither relates to Maxima's exp(x).
      ((or (string= op "Pow") (string= op "Exp"))
       (list '(mexpt) lhs rhs))
      ((string= op "Eq")  (list '(mequal) lhs rhs))
      ;; Comparison operators show up inside MSL If branches
      ;; (e.g. `pid.D.zeroGain = (Td < 1e-12)`).
      ((string= op "Lt")  (list '(mlessp)    lhs rhs))
      ((string= op "Le")  (list '(mleqp)     lhs rhs))
      ((string= op "Gt")  (list '(mgreaterp) lhs rhs))
      ((string= op "Ge")  (list '(mgeqp)     lhs rhs))
      ((string= op "Ne")  (list '(mnotequal) lhs rhs))
      ((string= op "And") (list '(mand)      lhs rhs))
      ((string= op "Or")  (list '(mor)       lhs rhs))
      (t (error "mochi: unsupported binary op ~S" op)))))

(defun mochi--unary-to-maxima (body)
  (let* ((op  (mochi--get body :op))
         (arg (mochi--ast-to-maxima (mochi--get body :rhs))))
    (cond
      ((string= op "Minus") (list '(mtimes) -1 arg))
      ((string= op "Plus")  arg)
      ((string= op "Not")   (list '(mnot) arg))
      (t (error "mochi: unsupported unary op ~S" op)))))

(defun mochi--builtin-call-to-maxima (body)
  "Translate a BuiltinCall node.  body.function is Pascal-cased
(`Der', `Pre', `Sin', `Max', ...)."
  (let* ((fn (mochi--get body :function))
         (raw-args (mochi--get body :args))
         (args (mapcar #'mochi--ast-to-maxima raw-args)))
    (cond
      ((string= fn "Der")
       ;; der(x) → der_x as a single Maxima symbol.  Read the VarRef name
       ;; directly so case-inversion runs once and matches mod_get(m,
       ;; 'derivs).
       (let* ((arg-body (cdar (first raw-args)))
              (var-name (mochi--varref-name arg-body)))
         (mochi--mxsym (concatenate 'string "der_" var-name))))
      ((string= fn "Pre")
       ;; pre(x) — Modelica's pre-event value.  Wrap so mochi--strip-pre
       ;; can collapse it to plain x at event time.
       (cons '($pre) args))
      ((string= fn "Sin")   (cons '(%sin) args))
      ((string= fn "Cos")   (cons '(%cos) args))
      ((string= fn "Tan")   (cons '(%tan) args))
      ((string= fn "Asin")  (cons '(%asin) args))
      ((string= fn "Acos")  (cons '(%acos) args))
      ((string= fn "Atan")  (cons '(%atan) args))
      ((string= fn "Atan2") (cons '(%atan2) args))
      ((string= fn "Sinh")  (cons '(%sinh) args))
      ((string= fn "Cosh")  (cons '(%cosh) args))
      ((string= fn "Tanh")  (cons '(%tanh) args))
      ((string= fn "Exp")   (cons '(%exp) args))
      ((string= fn "Log")   (cons '(%log) args))
      ((string= fn "Sqrt")  (cons '(%sqrt) args))
      ((string= fn "Abs")   (cons '(mabs) args))
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
      ((or (string= fn "Ceil") (string= fn "Ceiling")) (cons '($ceiling) args))
      (t (cons (list (mochi--mxsym (string-downcase fn))) args)))))

(defun mochi--ast-to-maxima (node)
  "Convert a rumoca flat-json AST expression node into a Maxima Lisp form."
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
         (:*var-ref       (mochi--varref-to-maxima body))
         (:*literal       (mochi--literal-to-maxima body))
         (:*binary        (mochi--binary-to-maxima body))
         (:*unary         (mochi--unary-to-maxima body))
         (:*builtin-call  (mochi--builtin-call-to-maxima body))
         (:*parenthesized (mochi--ast-to-maxima (mochi--get body :inner)))
         (:*array
          (cons '(mlist) (mapcar #'mochi--ast-to-maxima
                                 (mochi--get body :elements))))
         (:*if
          (let* ((branches    (mochi--get body :branches))
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
;; rumoca's flat-json surfaces each `when' clause as one entry in the
;; top-level :WHEN--CLAUSES list:
;;     ((:CONDITION . <bool-AST>)
;;      (:EQUATIONS . (<Reinit-node> ...))
;;      (:SPAN . ...))
;; where each Reinit body is `((:STATE . "name") (:VALUE . AST))' and
;; encodes a Modelica `reinit(name, value)' assignment.
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

(defun mochi--ast-binary-op (node)
  "Return the binary-op string (e.g. \"Le\", \"And\") for a Binary AST
node, or nil if NODE isn't a Binary."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged node)
    (when (eq tag :*binary)
      (mochi--get body :op))))

(defun mochi--cond-unary-not-body (node)
  "If NODE is a Unary with op = \"Not\", return its body alist.  Else nil."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged node)
    (when (eq tag :*unary)
      (let ((op (mochi--get body :op)))
        (when (and (stringp op) (string= op "Not")) body)))))

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
  (let ((op (mochi--ast-binary-op cond-node)))
    (cond
      ((null op)
       (let ((not-body (mochi--cond-unary-not-body cond-node)))
         (cond
           (not-body
            ;; not(c): flip directions.
            (mapcar (lambda (det)
                      (list (first det) (- (second det))))
                    (mochi--cond-to-detectors (mochi--get not-body :rhs))))
           (t
            (error "mochi: unsupported event condition AST tag ~S"
                   (and (consp cond-node) (caar cond-node)))))))
      ((or (string= op "Le") (string= op "Lt"))
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list (list (mochi--mx-sub lhs rhs) -1))))
      ((or (string= op "Ge") (string= op "Gt"))
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list (list (mochi--mx-sub lhs rhs) +1))))
      ((string= op "Eq")
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list (list (mochi--mx-sub lhs rhs) 0))))
      ((or (string= op "And") (string= op "Or"))
       (let ((body (cdar cond-node)))
         (append (mochi--cond-to-detectors (mochi--get body :lhs))
                 (mochi--cond-to-detectors (mochi--get body :rhs)))))
      (t
       (error "mochi: unsupported event condition binary op ~S" op)))))

(defun mochi--cond-to-guard (cond-node)
  "Convert a boolean condition AST to a real-valued Maxima expression
   that is > 0 iff the condition holds.  Caller checks `> 0' to decide
   whether to fire the reset."
  (let ((op (mochi--ast-binary-op cond-node)))
    (cond
      ((null op)
       (let ((not-body (mochi--cond-unary-not-body cond-node)))
         (cond
           (not-body
            (list '(mtimes) -1
                  (mochi--cond-to-guard (mochi--get not-body :rhs))))
           (t
            (error "mochi: unsupported event condition AST tag ~S"
                   (and (consp cond-node) (caar cond-node)))))))
      ((or (string= op "Le") (string= op "Lt"))
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         ;; lhs <= rhs  ⇔  rhs - lhs ≥ 0
         (mochi--mx-sub rhs lhs)))
      ((or (string= op "Ge") (string= op "Gt"))
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (mochi--mx-sub lhs rhs)))
      ((string= op "Eq")
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         ;; -|diff|^2 — zero only at equality, negative otherwise.
         (let ((diff (mochi--mx-sub lhs rhs)))
           (list '(mtimes) -1 (list '(mexpt) diff 2)))))
      ((string= op "And")
       (let ((body (cdar cond-node)))
         (list '($min)
               (mochi--cond-to-guard (mochi--get body :lhs))
               (mochi--cond-to-guard (mochi--get body :rhs)))))
      ((string= op "Or")
       (let ((body (cdar cond-node)))
         (list '($max)
               (mochi--cond-to-guard (mochi--get body :lhs))
               (mochi--cond-to-guard (mochi--get body :rhs)))))
      (t
       (error "mochi: unsupported event condition binary op ~S" op)))))

(defun mochi--cond-pretty (cond-node)
  "Convert a boolean condition AST to a Maxima expression preserving
   the original op shape, for display via mod_print."
  (let ((op (mochi--ast-binary-op cond-node)))
    (cond
      ((null op)
       (let ((not-body (mochi--cond-unary-not-body cond-node)))
         (cond
           (not-body
            (list '(mnot) (mochi--cond-pretty (mochi--get not-body :rhs))))
           (t
            ;; Fall back to plain conversion (best effort).
            (mochi--ast-to-maxima cond-node)))))
      ((string= op "Le")
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mleqp) lhs rhs)))
      ((string= op "Lt")
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mlessp) lhs rhs)))
      ((string= op "Ge")
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mgeqp) lhs rhs)))
      ((string= op "Gt")
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mgreaterp) lhs rhs)))
      ((string= op "Eq")
       (multiple-value-bind (lhs rhs) (mochi--lhs-rhs-mx cond-node)
         (list '(mequal) lhs rhs)))
      ((string= op "And")
       (let ((body (cdar cond-node)))
         (list '(mand)
               (mochi--cond-pretty (mochi--get body :lhs))
               (mochi--cond-pretty (mochi--get body :rhs)))))
      ((string= op "Or")
       (let ((body (cdar cond-node)))
         (list '(mor)
               (mochi--cond-pretty (mochi--get body :lhs))
               (mochi--cond-pretty (mochi--get body :rhs)))))
      (t
       (mochi--ast-to-maxima cond-node)))))

(defun mochi--reinit-to-eq (reinit-node)
  "Translate `((:*REINIT . ((:STATE . name) (:VALUE . ast))))' to a
Maxima equation `name = value' with `pre(...)' stripped (the pre-event
state at event time equals the value CVODE returns)."
  (multiple-value-bind (tag body) (mochi--unwrap-tagged reinit-node)
    (unless (eq tag :*reinit)
      (error "mochi: unexpected when-clause equation tag ~S" tag))
    (let* ((state-name (mochi--get body :state))
           (value-mx (mochi--strip-pre
                      (mochi--ast-to-maxima (mochi--get body :value)))))
      (list '(mequal) (mochi--mxsym state-name) value-mx))))

(defun errcatch-mochi (thunk)
  "Run THUNK, returning its result on success or NIL on any error.
Used to skip when_clauses whose condition isn't shaped like a real-
valued boundary detector (e.g. a bare Boolean discrete-mode flag)."
  (handler-case (funcall thunk)
    (error () nil)))

(defun mochi--when-clause-to-events (wc)
  "Convert one :WHEN--CLAUSES entry to a list of event tuples
`(detector reset-eqs guard cond-pretty direction)' — one per primitive
comparison in the condition.  All tuples share the same reset and guard
but each carries its own direction."
  (let* ((cond-node (mochi--get wc :condition))
         (reset-eqs (mochi--mlist
                     (mapcar #'mochi--reinit-to-eq
                             (mochi--get wc :equations)))))
    (or (errcatch-mochi
         (lambda ()
           (let ((detectors   (mochi--cond-to-detectors cond-node))
                 (guard       (mochi--cond-to-guard       cond-node))
                 (cond-pretty (mochi--cond-pretty         cond-node)))
             (mapcar (lambda (det)
                       (mochi--mlist (list (first det)
                                           reset-eqs
                                           guard
                                           cond-pretty
                                           (second det))))
                     detectors))))
        '())))

(defun mochi--collect-state-dependent-conds (raw-eqs state-names)
  "Walk RAW-EQS for `If' nodes; return the list of branch conditions
whose expressions reference any of STATE-NAMES.

Modelica models like SwitchedRC encode discontinuities as inline
`if x > 0 then A else B' in their equations rather than as `when'
clauses.  At simulation time CVODE needs an explicit zero-crossing
detector to stop cleanly at the boundary; otherwise it tries to step
adaptively across the discontinuity.  We synthesise detector-only
events from these conditions.

Conditions that reference only parameters (e.g. MSL PID's
`pid.D.zeroGain' = (Td < 1e-12)`) are skipped — those collapse to a
constant at linearisation time via mod__resolve_cond_ifs, so they don't
need a runtime detector."
  (let ((conds '()))
    (dolist (eq raw-eqs)
      (mochi--walk-ast
       (mochi--get eq :residual)
       (lambda (node)
         (when (eq (caar node) :*if)
           (let ((branches (mochi--get (cdar node) :branches)))
             (dolist (br branches)
               (let* ((cond-ast (first br))
                      (refs (mochi--ast-references cond-ast)))
                 (when (some (lambda (s) (gethash s refs)) state-names)
                   (push cond-ast conds)))))))))
    (nreverse conds)))

(defun mochi--bare-if-event-tuples (cond-ast covered-pretties)
  "Synthesise detector-only event tuples for a state-dependent boolean
condition that appears inside an inline `if'.  Returns nil if the cond
isn't comparison-shaped (mochi--cond-to-detectors errors out) or its
pretty form already appears in COVERED-PRETTIES (so a when_clause event
is already tracking the same boundary)."
  (errcatch-mochi
   (lambda ()
     (let ((pretty (mochi--cond-pretty cond-ast)))
       (unless (member pretty covered-pretties :test #'equal)
         (let ((detectors   (mochi--cond-to-detectors cond-ast))
               (empty-reset (mochi--mlist '())))
           (mapcar (lambda (det)
                     (mochi--mlist (list (first det)
                                         empty-reset
                                         t      ; guard = always true
                                         pretty
                                         (second det))))
                   detectors)))))))

(defun mochi--extract-events (raw-when-clauses extra-cond-asts)
  "Build the events list for the model struct.  WHEN_CLAUSE-derived
events (reset-bearing) come first; detector-only events synthesised
from state-dependent inline `if' conditions in EXTRA-COND-ASTS come
second, deduplicated against the when_clause set."
  (let* ((wc-events (mapcan #'mochi--when-clause-to-events raw-when-clauses))
         (covered (remove-duplicates
                   (mapcar (lambda (ev) (fourth (cdr ev))) wc-events)
                   :test #'equal))
         (extra-events
           (loop for c in extra-cond-asts
                 nconc (or (mochi--bare-if-event-tuples c covered) '()))))
    (mochi--mlist (append wc-events extra-events))))

;; --- Variable classification (flat-json) -------------------------------

(defun mochi--var-tag (info field)
  "Read a tagged-enum FIELD (e.g. :variability, :causality) from a
variable info alist.  rumoca emits either the string \"Empty\" or a
single-key alist like ((:*PARAMETER . inner))."
  (let ((v (mochi--get info field)))
    (when (consp v) (caar v))))

(defun mochi--top-level-name-p (name)
  "True iff NAME has no `.' — heuristic for `top-level model variable'
after rumoca's flattening pass.  Dotted names are inherited or instance-
scoped: `tank1.q_out' came from the inner Tank class, not the outer
TwoTanks user interface."
  (not (find #\. name)))

(defun mochi--walk-ast (node fn)
  "Recursively descend an AST NODE, calling FN on each tagged-enum
sub-node encountered.  Bodies are alists `((:KEY . VALUE) ...)' — we
recurse into each VALUE so nested AST nodes (themselves tagged-enum
alists) get visited."
  (when (and (consp node) (consp (car node)) (keywordp (caar node)))
    (funcall fn node)
    (let ((body (cdar node)))
      (when (consp body)
        (dolist (kv body)
          (let ((v (cdr kv)))
            (cond
              ((and (consp v) (consp (car v)) (keywordp (caar v)))
               (mochi--walk-ast v fn))
              ((listp v)
               (dolist (child v)
                 (when (consp child) (mochi--walk-ast child fn)))))))))))

(defun mochi--outer-or-connect-eq-p (eq)
  "True iff EQ comes from the top-level model body or from a connect()
synthesis.  rumoca tags each equation with :ORIGIN, one of:
  ComponentEquation{component: \"\"}        — outer-body user equation
  ComponentEquation{component: \"path.to\"}  — flattened inner-class
  Connection                                  — synthesised connect equality
  FlowSum                                     — sum-of-flow-vars = 0
Only the first three count as 'outer constraints' for input binding —
flattened inner equations are part of the inner class's interface and
don't bind the outer model's free inputs."
  (let ((origin (mochi--get eq :origin)))
    (when (consp origin)
      (let ((tag (caar origin)))
        (cond
          ((eq tag :*connection) t)
          ((eq tag :*flow-sum)   t)
          ((eq tag :*component-equation)
           (let* ((body (cdar origin))
                  (component (mochi--get body :component)))
             (and (stringp component) (string= component "")))))))))

(defun mochi--ast-references (node)
  "Walk an AST NODE and return a hash table of every VarRef name found."
  (let ((names (make-hash-table :test 'equal)))
    (mochi--walk-ast
     node
     (lambda (n)
       (when (eq (caar n) :*var-ref)
         (setf (gethash (mochi--varref-name (cdar n)) names) t))))
    names))

(defun mochi--bound-input-names (raw-eqs)
  "Set (as a list) of variable names touched by any outer-body or
connect-derived equation in RAW-EQS.  An Input-tagged variable in this
set is bound by the outer model and is therefore algebraic from the
caller's perspective, not a free input."
  (let ((touched (make-hash-table :test 'equal)))
    (dolist (eq raw-eqs)
      (when (mochi--outer-or-connect-eq-p eq)
        (let ((refs (mochi--ast-references (mochi--get eq :residual))))
          (loop for k being the hash-keys of refs do
                (setf (gethash k touched) t)))))
    (loop for k being the hash-keys of touched collect k)))

(defun mochi--collect-der-states (raw-eqs)
  "Walk RAW-EQS (list of `((:RESIDUAL . AST) ...)' entries) and return
the names of variables appearing inside `Der(...)' BuiltinCalls.
Modelica defines states as continuous variables whose derivative is
referenced — this is how mochi recovers the state list from flat-json,
which (unlike v0.7.x's :x dict) doesn't enumerate them separately."
  (let ((states (make-hash-table :test 'equal)))
    (dolist (eq raw-eqs)
      (mochi--walk-ast
       (mochi--get eq :residual)
       (lambda (node)
         (let ((tag (caar node)) (body (cdar node)))
           (when (and (eq tag :*builtin-call)
                      (stringp (mochi--get body :function))
                      (string= (mochi--get body :function) "Der"))
             (let* ((arg (first (mochi--get body :args))))
               (when (and (consp arg) (consp (car arg))
                          (eq (caar arg) :*var-ref))
                 (let ((name (mochi--varref-name (cdar arg))))
                   (setf (gethash name states) t)))))))))
    (loop for k being the hash-keys of states collect k)))

(defun mochi--start-value (info)
  "Return the variable's start-value as a Maxima expression (or number).
Prefer :BINDING (the resolved declaration expression — literal or
computed) over :START (rumoca's parsed-from-source default before
flattening).  Returns 0 if both are absent or the JSON sentinel
\"Empty\"."
  (labels ((empty-p (v) (or (null v)
                            (and (stringp v) (string= v "Empty")))))
    (let ((binding (mochi--get info :binding))
          (start   (mochi--get info :start)))
      (cond
        ((not (empty-p binding)) (mochi--ast-to-maxima binding))
        ((not (empty-p start))   (mochi--ast-to-maxima start))
        (t 0)))))

(defun mochi--classify-variables (raw-variables state-names
                                  top-level-inputs bound-names)
  "Partition RAW-VARIABLES (the :VARIABLES alist) into a (param-info,
state-name, input-name, output-name, algebraic-name) tuple list,
returned via multiple values in that order.

PARAM-INFO entries carry the full info alist (we need :binding /
:start at construction time); the other four are flat lists of dotted
name strings.

Classification rules:
  variability = Parameter or Constant       → param
  name appears in STATE-NAMES               → state
  name appears in TOP-LEVEL-INPUTS          → input (explicit top-level)
  causality = Input and name NOT in
            BOUND-NAMES                     → input (instance-promoted)
  causality = Output and dot-free name      → output (top-level only)
  otherwise                                 → algebraic

Rationale: rumoca emits `top_level_input_components' for the model's
explicit top-level `input' declarations, and propagates `causality.Input'
into every flattened instance member.  An instance member is free
(promoted to a model-level input) iff no outer-body or connect-derived
equation touches it.  BOUND-NAMES is the touched-by-outer set; we
exclude inputs that appear in it.  TOP-LEVEL-INPUTS always wins — an
explicit declaration is free even if it's referenced on the RHS of an
outer equation."
  (let ((params nil)
        (states nil)
        (inputs nil)
        (outputs nil)
        (algebraics nil))
    (dolist (entry raw-variables)
      (let* ((info (cdr entry))
             ;; cl-json mangles dotted dict keys (e.g. \"plant.y\" becomes
             ;; :PLANT.Y, \"R\" becomes :+R+), so the alist key isn't a
             ;; reliable name.  Pull the verbatim Modelica name from the
             ;; info's :NAME string field instead.
             (name (mochi--get info :name))
             (vtag (mochi--var-tag info :variability))
             (ctag (mochi--var-tag info :causality)))
        (cond
          ((or (eq vtag :*parameter) (eq vtag :*constant))
           (push (cons name info) params))
          ((member name state-names :test #'string=)
           (push name states))
          ((member name top-level-inputs :test #'string=)
           (push name inputs))
          ((and (eq ctag :*input)
                (not (member name bound-names :test #'string=)))
           (push name inputs))
          ((and (eq ctag :*output) (mochi--top-level-name-p name))
           (push name outputs))
          (t
           (push name algebraics)))))
    (values (nreverse params) (nreverse states)
            (nreverse inputs)  (nreverse outputs)
            (nreverse algebraics))))

(defun mochi--state-info-by-name (raw-variables name)
  "Look up a variable's info alist by NAME (string).  Match against the
info's :NAME field rather than the alist key, since cl-json mangles
dotted / uppercase JSON keys."
  (loop for entry in raw-variables
        for info = (cdr entry)
        when (string= (mochi--get info :name) name) return info))

;; --- Walk Maxima expressions for symbol cleanup ------------------------

(defun mochi--walk-syms (expr)
  "Walk a Maxima Lisp expression and return all `$'-prefixed user symbols."
  (cond
    ((and (symbolp expr)
          (let ((n (symbol-name expr)))
            (and (plusp (length n)) (char= (char n 0) #\$))))
     (list expr))
    ((consp expr) (mapcan #'mochi--walk-syms expr))
    (t nil)))

(defparameter *mochi-implicit-globals*
  '($time)
  "Maxima symbols that may appear in residuals without being declared
in :VARIABLES.  Modelica's `time' is the independent variable, supplied
by the integrator — it isn't a state or algebraic.  mochi--unclassified-
syms filters these out so mod__causalise doesn't try to solve for
them.")

(defun mochi--unclassified-syms (residual-exprs known-syms)
  "Return symbols appearing in RESIDUAL-EXPRS that aren't in KNOWN-SYMS.
Catches connector-flattened internal signals that didn't make it into
the variable list, while excluding Modelica primitives like `time'."
  (let* ((all (remove-duplicates (mapcan #'mochi--walk-syms residual-exprs)))
         (unknown (set-difference all known-syms)))
    (set-difference unknown *mochi-implicit-globals*)))

;; --- Public entry point ------------------------------------------------

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
         (raw-variables    (cdr (assoc :variables model)))
         (raw-eqs          (cdr (assoc :equations model)))
         (raw-when-clauses (cdr (assoc :when--clauses model)))
         (raw-top-inputs   (cdr (assoc :top--level--input--components model)))
         (residual-exprs (mapcar (lambda (eq)
                                   (mochi--ast-to-maxima
                                    (mochi--get eq :residual)))
                                 raw-eqs))
         (state-names (mochi--collect-der-states raw-eqs))
         (bound-names (mochi--bound-input-names raw-eqs))
         (extra-cond-asts (mochi--collect-state-dependent-conds
                           raw-eqs state-names)))
    (multiple-value-bind (params state-syms-str input-syms-str
                          output-syms-str alg-syms-str)
        (mochi--classify-variables raw-variables state-names
                                   raw-top-inputs bound-names)
      (let* ((name resolved-name)
             (params-mlist
               (mochi--mlist
                (mapcar (lambda (entry)
                          (mochi--mlist
                           (list (mochi--mxsym (car entry))
                                 (mochi--start-value (cdr entry)))))
                        params)))
             (state-syms (mapcar #'mochi--mxsym state-syms-str))
             (deriv-syms (mapcar (lambda (n)
                                   (mochi--mxsym
                                    (concatenate 'string "der_" n)))
                                 state-syms-str))
             (input-syms  (mapcar #'mochi--mxsym input-syms-str))
             (output-syms (mapcar #'mochi--mxsym output-syms-str))
             (alg-from-vars (mapcar #'mochi--mxsym alg-syms-str))
             (known-syms (append (mapcar (lambda (e) (mochi--mxsym (car e))) params)
                                 state-syms deriv-syms
                                 input-syms output-syms alg-from-vars))
             ;; Catch connector-flattened algebraics that aren't in :variables.
             (alg-extra (mochi--unclassified-syms residual-exprs known-syms))
             (algebraics (append alg-from-vars alg-extra))
             (initial-mlist
               (mochi--mlist
                (mapcar (lambda (sname)
                          (let ((info (mochi--state-info-by-name
                                       raw-variables sname)))
                            (mochi--mlist
                             (list (mochi--mxsym sname)
                                   (if info
                                       (mochi--start-value info)
                                       0)))))
                        state-syms-str))))
        (mochi--mlist
         (list (mochi--mequal '$name name)
               (mochi--mequal '$params params-mlist)
               (mochi--mequal '$states (mochi--mlist state-syms))
               (mochi--mequal '$derivs (mochi--mlist deriv-syms))
               (mochi--mequal '$algebraics (mochi--mlist algebraics))
               (mochi--mequal '$inputs (mochi--mlist input-syms))
               (mochi--mequal '$outputs (mochi--mlist output-syms))
               (mochi--mequal '$initial initial-mlist)
               (mochi--mequal '$residuals (mochi--mlist residual-exprs))
               (mochi--mequal '$events (mochi--extract-events
                                        raw-when-clauses
                                        extra-cond-asts))))))))
