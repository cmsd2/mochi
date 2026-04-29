# mochi — design and architecture

This document explains *how* mochi works and *why* it works that way.
It's aimed at someone who will modify the code, not just use it.
For user-facing reference see `doc/mochi.md`.

## What mochi is, in one paragraph

mochi is a Maxima frontend over a Modelica compiler. It reads a `.mo`
file, gets a flat differential-algebraic equation (DAE) system out of
it, and provides Maxima-level operations on top: symbolic
linearisation, time-domain simulation (linear and nonlinear), transfer
functions, plant composition, dataflow visualisation, and discrete-
event handling. The end goal is to make a working control engineer
able to write `m : mod_load("MyPlant.mo")` and immediately have access
to the same kit they'd reach for in Matlab/Simulink — but expressed
through Maxima's symbolic algebra rather than a closed
proprietary toolchain.

## Why split the work with rumoca

Modelica's surface language is large: a type system with units, a
two-pin connector model with `flow` and potential variables that
expand into Kirchhoff equations at `connect()` sites, multi-file
packages with `within` qualifiers, `extends` inheritance, `redeclare`
component substitution, conditional and stream connectors, hybrid
discrete events with `when`/`reinit`, function libraries, and a small
cottage industry of compatibility quirks across versions. Doing
that work inside Maxima would be its own multi-year project.

[rumoca](https://github.com/CogniPilot/rumoca) is a Rust-based Modelica
compiler that does the heavy lifting: parse, type-check, expand
connectors, flatten `extends`, and emit a "flat" JSON description of
the resulting DAE. mochi consumes that flat description and adds the
analysis layer that's specific to a Maxima workflow.

This split shapes everything else in mochi:

- mochi never *parses* Modelica syntax beyond what's needed to
  re-discover information rumoca didn't surface (input/output
  declarations, `extends` parents — see "the source-file scanner"
  below). Anything structural that requires understanding of the
  language goes through rumoca.
- mochi's input format is rumoca's JSON, not the `.mo` text. New
  Modelica features become available the moment rumoca supports them,
  whether mochi has been updated or not — provided the resulting AST
  shapes are ones mochi knows how to walk.
- When something doesn't work end-to-end, the first question is
  always "is this rumoca's bug, mochi's bug, or a mismatch between
  rumoca's AST shape and what mochi expects". Most often it's the
  third.

## The pipeline, end to end

The user writes:

```maxima
load("mochi")$
m : mod_load("examples/RCFilter.mo")$
[A, B, C, D] : mod_state_space(m, [iL = 0, vC = 0, Vin = 0])$
```

Behind the scenes, here's what happens:

1. **`mod_load`** (`mochi-loader.lisp`) shells out to rumoca with
   `--source-root <each>` for every directory in the resolved source-
   root list (see "source-root resolution" below). rumoca returns a
   JSON document — top-level keys `p`/`x`/`y`/`f_x`, plus
   `f_z`/`f_c`/`m`/`f_m`/`relation`/`initial`/`functions` if the model
   has discrete or hybrid components.
2. **cl-json** parses that JSON into a tagged-variant Lisp form. mochi
   walks the AST, building Maxima symbols for each parameter, state,
   derivative, input, output, and algebraic, plus Maxima expressions
   for residuals, output equations, event detectors, reset rules.
3. **The source-file scanner** (Lisp) re-reads the original `.mo` file
   to recover the input/output classification and the `extends`
   parent list, both of which rumoca's JSON doesn't expose with
   sufficient fidelity (see "the source-file scanner" below).
4. **`mod_load` returns a Maxima struct** — a list of `[key = value]`
   equations. `name`, `params`, `states`, `derivs`, `algebraics`,
   `inputs`, `outputs`, `initial`, `residuals`, `events`. Every
   downstream operation reads from this struct via `mod_get`.

Then on top of the struct:

- `mod__causalise(m)` — symbolically solve the residuals for the
  derivatives, algebraics, and outputs, returning explicit equations.
- `mod_state_space(m, op)` — substitute parameters, constant-fold,
  differentiate to get Jacobians, substitute the operating point,
  return numeric `[A, B, C, D]`.
- `mod_simulate / mod_step / mod_impulse` — ZOH discretise the
  linearised model and propagate.
- `mochi-nonlinear`'s `mod_simulate_nonlinear` — feed the causalised
  RHS to SUNDIALS CVODE, with optional event handling.
- `mod_diagram` — extract edges from `[A | B; C | D]`, render via
  GraphViz `dot` (or Mermaid CLI), embed the SVG as an image output
  the notebook UI displays inline.

Each of the next sections digs into one of these stages. The
recurring theme is "we know the mathematical operation we want; the
work is making the symbolic plumbing on either side line up".

## The Lisp loader

`mochi-loader.lisp` is a single-file Common Lisp module that hooks
into Maxima as `$mod_load`. It exists in Lisp rather than Maxima
because:

- The JSON parser (`cl-json`) is a Lisp library available through
  Quicklisp, with no Maxima equivalent.
- We need to construct Maxima symbols and expressions *directly*,
  using `intern` + the `$` prefix convention, without going through
  a Maxima parse-eval cycle. This matters because evaluating user
  code as Maxima can disturb session state — a model parameter named
  `R` would clobber a user variable also named `R`. Our loader
  produces fresh internal symbols and never invokes Maxima's
  evaluator.
- File I/O, subprocess invocation (`uiop:run-program` for rumoca),
  and AST walking are all easier in Lisp than in Maxima.

### Symbol construction

`mochi--mxsym` is the central helper that turns a Modelica name
string into a Maxima user symbol:

1. **Dot-flattening.** `tank1.h` becomes `tank1_h`. Modelica's dotted
   names are illegal in Maxima symbols.
2. **Case-inversion.** `mochi--maxima-case-invert`. Maxima's reader
   inverts case for all-uppercase or all-lowercase symbols (it's how
   Common Lisp's case-folding reader hides itself from users). If we
   intern `(make-symbol "$R")` it'd display as `r` in Maxima output,
   which is confusing. We pre-invert the case so the internal symbol
   `$r` displays as `R` — matching what the user wrote.
3. **`$` prefix and intern in the `:maxima` package.** This is
   Maxima's convention for user-defined symbols. `$x` in Lisp is the
   user-visible symbol `x` in Maxima.

Every parameter, state, derivative, input, and output goes through
`mochi--mxsym`. Forgetting any step produces a symbol that doesn't
match what `mod_get` returns later — the bug typically manifests as
"`solve` says it can't solve the residuals because X isn't a known
variable".

### AST walker

`mochi--ast-to-maxima` recursively walks rumoca's tagged-variant AST
(cl-json wraps each variant tag as a one-element alist with a `:*`-
prefixed key like `:*BINARY`, `:*VAR-REF`, `:*BUILTIN-CALL`). The
walker maps each tag to the Maxima Lisp form:

| rumoca tag | Maxima form |
|---|---|
| `:*var-ref` | `$x` |
| `:*literal` | the unwrapped numeric / Boolean / string |
| `:*binary` | `((mplus) ...)`, `((mtimes) ...)`, comparisons (`mlessp`, `mleqp`, etc.), Boolean (`mand`, `mor`) |
| `:*unary` | `((mtimes) -1 arg)` for minus, identity for plus, `((mnot) arg)` |
| `:*builtin-call` | named function (`(($sin) arg)`, `(($exp) arg)`, ...) |
| `:*paren` | inner expression |
| `:*function-call` | user-named function call |
| `:*if` | `((mcond) cond1 val1 ... t else)` |
| `:*array` | `((mlist) elements...)` |

The walker is incomplete by design — when it hits a tag it doesn't
recognise, it errors with the unknown tag name. Adding support for a
new Modelica feature usually means: try to load a model that uses it,
see what tag rumoca emits, add a clause. The current set covers
~100% of the core language plus the common pieces of the standard
library. Nothing in the loader pre-populates a list of "known
features" — we discover them as users hit them.

### The source-file scanner

rumoca's JSON has a `y` (algebraic) list that includes everything
that isn't a parameter or a state — including variables that the user
intends as `input` or `output`. The JSON doesn't currently tag inputs
versus other algebraics. So mochi falls back to scanning the original
`.mo` file (via `mochi--scan-keyword`) for `input Real X` and
`output Real X` declarations.

This scan is also recursive: the scanner walks `extends ParentName`
declarations in the top-level model, looks up each parent in the
file's class table (built once by `mochi--collect-class-io`), and
inherits the parent's input/output declarations transitively. Without
this, refactoring an electrical RC circuit to use a `partial model
OnePort` base would lose all of `OnePort`'s ports.

The scanner is deliberately stupid — it pattern-matches `input` /
`output` / `extends` as token-bounded keywords, no Modelica parser.
Multi-line declarations, nested comments, attribute lists, anything
exotic — would slip through. So far nothing in the wild has tripped
it; if something did, the right fix is probably to lift the relevant
metadata from rumoca's JSON instead of re-parsing.

### Source-root resolution

Real models — especially ones using the Modelica Standard Library —
reference components from other files: `Modelica.Electrical.Analog
.Basic.Resistor`. rumoca needs `--source-root <dir>` arguments for
each directory under which to search, where each `<dir>` is the
parent of a `Modelica/` package directory.

mochi resolves the source-root list with explicit-configuration-wins
rules:

1. `mod_set_source_root(...)` / `mod_add_source_root(...)`  
   from Maxima — the user's authoritative list.
2. The `MODELICAPATH` environment variable (the standard Modelica-
   tools convention from the language spec, multi-path: colon-
   separated on Unix, semicolon on Windows).
3. As a fallback only when neither is set, OpenModelica's documented
   install location: `$OPENMODELICAHOME/lib/omlibrary/Modelica*/`
   (defaulting to `/opt/openmodelica` if the env var isn't exported).

Anything not in one of these locations is invisible. No invented
paths, no auto-cloning. The user is in charge.

## Causalisation

The continuous DAE that arrives from rumoca is residual-form: each
equation `F_i(x, dx/dt, y, u, p, t) = 0`. To linearise or simulate
we need explicit `dx/dt = f(x, u, p)` and `y = g(x, u, p)`, which
means solving the residuals for the derivative, algebraic, and output
variables.

`mod__causalise(m)` does this with a single Maxima `solve` call,
treating all derivatives + outputs + algebraics as unknowns. It
returns three lists:

- `der_eqs`: `[der_state_i, rhs_expr]` per state.
- `alg_eqs`: `[alg_var_j, rhs_expr]` per algebraic.
- `output_eqs`: `[output_k, rhs_expr]` per output.

The pure-symbolic step is what makes everything else possible — both
the linear and nonlinear paths consume `mod__causalise`'s output. The
linear path differentiates the explicit `f` and `g`; the nonlinear
path substitutes parameters and hands `f` to a numeric integrator.

**Why one `solve` call rather than per-equation manipulation**: for
plants of any structural complexity (RCFilter has ~22 unknowns,
PIDClosedLoop has ~40), Maxima's `linsolve`/`solve` does all the
necessary block-triangulation internally. Calling it once is faster
and produces simpler expressions than walking the equations
ourselves.

**Why this fails for genuinely-implicit algebraic loops**: if some
unknowns can only be determined by simultaneously satisfying
nonlinear equations (think a solver loop in Modelica's `Modelica
.Mechanics.MultiBody`), `solve` returns either an unsolved equation
or `[]`. We currently error out. Phase 2 of nonlinear simulation —
not yet built — would shift those into a runtime nonlinear solver via
SUNDIALS IDA.

## Linear analysis

`mod_state_space(m, op_point[, override_params])` does the most
mathematically substantive piece of work in mochi. The order of
operations matters and is the result of trial and error:

1. **Causalise.** Call `mod__causalise` to get explicit `f_expr`,
   `y_expr` in terms of states, inputs, and parameters.
2. **Resolve parameters.** rumoca emits MSL parameter defaults as
   *expressions* (e.g. `D.T = max({Td/Nd, 100*1e-15})`) rather than
   pre-computed values. `mod__resolve_params` iterates substitution
   to fixed point so chained defaults reduce to numbers, with the
   user's override list applied first so overrides shadow defaults.
3. **Substitute parameters into `f_expr` and `y_expr`.** This step is
   *before* differentiation. The substituted expressions still
   contain state and input symbols — they're symbolic in those — but
   parameter symbols are now concrete numbers (or symbolic
   expressions of input/state if the user didn't fully resolve).
4. **`mod__simplify_expr` (constant fold).** `ev(expr, eval)` walks
   the substituted expression tree. The reason this step exists at
   all: MSL's PID emits residuals like `if D.zeroGain then 0 else
   (u - x) / D.T`. After step 3, `D.zeroGain` is the literal `false`,
   so the branch *should* collapse to `(u - x) / D.T`. But Maxima's
   simplifier is lazy and won't reduce the `mcond` until forced. If
   we differentiate first, `diff(mcond, x)` returns a piecewise
   expression that downstream substitution can't always evaluate
   cleanly (the symptom is "0 to a negative exponent" or "diff:
   variable must not be a number"). Forcing `ev(.., eval)` here
   collapses every parameter-gated branch.
5. **`mod__resolve_cond_ifs`** — handles `if`s whose conditions
   depend on *states*, not just parameters (so step 4 can't reduce
   them). The walker substitutes the operating point into the
   condition only, evaluates with `is(...)`, and picks the surviving
   branch. The branch body retains its state symbols so `diff` still
   has something to differentiate. This is what makes regime-aware
   linearisation work — `der(x) = if x > 0 then a(x,u) else b(x,u)`
   linearised at `x = 0.5` becomes `diff(a, x)` cleanly. See
   `examples/SwitchedRC.mo`. Without this step Maxima's `diff`
   refuses to push inside the `if` (it's piecewise, not classically
   differentiable at the boundary), leaving an unevaluated `d/dx
   (...)` that breaks downstream.
6. **Build Jacobians.** `genmatrix(lambda([i,j], diff(...)), n, n)`
   for each of `A = ∂f/∂x`, `B = ∂f/∂u`, `C = ∂g/∂x`, `D = ∂g/∂u`.
   Now operates on a clean rational-function expression.
7. **Substitute the operating point.** Op-point values for states
   and inputs go in here, last.
8. **`float(...)`** to convert from Maxima rationals to double-
   precision floats so downstream numeric code (`np_eig`, `np_expm`,
   etc.) accepts them.

The "substitute parameters before differentiating" choice is contrary
to what most symbolic-Jacobian implementations do (typically diff is
done first to keep one symbolic Jacobian that gets cheaply re-
evaluated as parameters change). We picked the order we did because
mod_state_space is called fresh per request anyway — there's no
caching to amortise — and constant-folding pre-differentiation is the
only way to avoid the `mcond` issue.

`mod_state_space_symbolic(m, op_point)` exists for users who want
parameters to remain symbolic. It uses an older code path
(`mod__symbolic_ss`) that does diff first, no parameter
substitution, no constant folding. Models with state-dependent `If`
branches in their residuals still produce piecewise Jacobians here;
that's accepted because symbolic mode is for inspection rather than
numeric evaluation.

## Linear simulation

`mod_simulate(m, op_point, u_fn, t_end[, opts])` propagates the
linearised state-space forward in time. The implementation is
straightforward:

- `mod__zoh_discretise(A, B, dt)` builds `A_d, B_d` via the
  augmented-matrix exponential trick: `M = [[A, B]; [0, 0]]; eM =
  expm(M * dt); A_d = eM[1:n, 1:n]; B_d = eM[1:n, n+1:n+m]`. This
  handles the singular-A case correctly (where the closed form
  `B_d = A^{-1}(eAdt - I)B` would divide by zero).
- A simple loop `x[k+1] = A_d x[k] + B_d u[k]; y[k] = C x[k] + D u[k]`.

`mod_step` and `mod_impulse` are thin wrappers that synthesise
specific `u_fn` lambdas. The closure-capture for `magnitude` and
`input_index` uses Maxima's `buildq` so the lambda body has the
captured values inlined as literals.

## Nonlinear simulation

`mochi-nonlinear` is an opt-in subsystem that depends on the
`numerics-sundials` package (which provides `np_cvode`, a CFFI
binding to SUNDIALS CVODE). It's split off rather than included in
the core for two reasons:

- SUNDIALS is a heavy C dependency. Pure-linear users (most control-
  design workflows) shouldn't have to install it.
- The semantics are different — direct integration of the *nonlinear*
  DAE without operating-point assumptions. Bundling it with the
  linear analysis would imply they're equivalent, which they aren't.

`mod_simulate_nonlinear(m, x0, u_fn, t_end[, opts])` uses CVODE in
*expression mode*: the RHS is a list of Maxima expressions in
`[t, x_1, ..., x_n]`, and CVODE compiles them to native Lisp closures
via `coerce-float-fun`. This is much faster than CVODE's
function-mode (which routes each call through Maxima's evaluator)
and avoids a class of bugs we'd otherwise hit.

The pipeline:

1. Causalise.
2. Resolve and substitute parameters.
3. Evaluate `u_fn('t)` to bake the user's input function into the
   RHS expressions in terms of `t`. This is what locks us into
   expression mode for inputs that can be expressed symbolically — a
   pure data-table input wouldn't be expressible this way (a future
   improvement could be falling back to function mode in that case).
4. Hand the RHS list, the variable list `[t, x_1, ...]`, the initial
   state, and the time vector to `np_cvode`.

### Discrete events

When `m` has events (extracted by the loader from rumoca's `f_z`
block), `mod_simulate_nonlinear` runs *segment by segment* between
requested sample times. Inside each segment:

1. Call `np_cvode` watching the event detector expressions. CVODE's
   rootfinder reports zero crossings.
2. Drop spurious detections too close in time to the previous
   accepted event (`event_dedup_eps = 1e-3` watchdog).
3. For each surviving event, evaluate the user-supplied guard
   expression at the event-time state. If `guard <= 0`, the event is
   spurious (the original Modelica boolean isn't actually true at
   this point — typically a re-detection of the same boundary the
   reset just left), and we advance past it without applying the
   reset.
4. If `guard > 0`, apply the reset rule, then take a tiny Euler
   half-step (`nudge_dt = 1e-4`) using the RHS evaluated at the
   post-reset state. This pushes the state cleanly off the event
   boundary so CVODE's rootfinder doesn't immediately re-detect the
   same zero crossing on restart.
5. A 100-iteration max bounds Zeno-like loops.

The nudge + dedup combination feels hacky and is — proper direction-
aware event detection would be the principled fix, requiring an
upstream addition to numerics-sundials of `CVodeSetRootDirection`.
Worth doing eventually; not blocking anything today.

### Event extraction

Rumoca's `f_z` block describes `when` clauses as `If(branches=
[[Edge(cond), reset_value]], else=current_value)` per state being
reset. `mochi--extract-events` walks each `f_z` entry:

- Strip the `Edge(...)` wrapper (we treat rising and falling crossings
  the same way — direction filtering happens via the guard).
- Decompose the boolean condition into a *list* of primitive
  detectors (one per `Le`/`Lt`/`Ge`/`Gt`/`Eq`), with `And` and `Or`
  appending the detector lists from each side. So `h <= 0 and v < 0`
  produces two detectors (`h`, `v`), both registered with CVODE so
  whichever crosses first triggers re-evaluation of the full guard.
- Build a real-valued guard expression that's positive iff the
  original boolean holds: `min(g_A, g_B)` for `And`, `max(g_A, g_B)`
  for `Or`, `-(g_c)` for `Not`. The simplifier eats away at this once
  parameters are subbed in.
- `pre(x)` in reset values simplifies to `x` — at event time the
  pre-event state is exactly what CVODE returns.

The struct's `events` field stores 4-element entries: `[detector,
reset_eqs, guard, cond_pretty]`. The first three are what
`mod_simulate_nonlinear` consumes; `cond_pretty` is the original
boolean preserved in Maxima form so `mod_print` can render `when h
<= 0 and v < 0 : [v = -e*v]` (rather than the derived `min(-h, -v)`
guard, which is correct but unhelpful for the reader).

`mochi--extract-events` also walks rumoca's `f_c` block — the
*condition equations* — to capture implicit events from bare `if`s
in continuous equations (Modelica Language Spec §8.5: relations on
continuous variables generate implicit events whether or not the
user wrapped them in a `when`). For each `f_c` entry whose condition
isn't already covered by an `f_z`, mochi synthesises a *detector-
only* event: empty `reset_eqs` (state stays unchanged), `guard =
true` (always accept). The role: tell CVODE to stop at the
discontinuity, not to step over it. Without this, a model like
`der(x) = if x > 0 then a(x,u) else b(x,u)` integrates correctly but
slowly — CVODE takes a step over the boundary, error estimates blow
up, step gets halved and retried until the integrator stumbles
through. With the f_c-derived event, CVODE stops cleanly at the
zero crossing and resumes on the other side. See
`examples/SwitchedRC.mo`.

f_c entries whose condition isn't a comparison-shaped expression
(e.g. PID's `local_reset` Boolean variable being checked directly)
are silently skipped — they're discrete-mode flags rather than
continuous-state boundary detectors, and CVODE has no way to detect
"a Boolean became true" anyway. mochi swallows the AST-walker error
via an `errcatch-mochi` wrapper so the rest of the model still
loads.

## Diagrams

`mod_diagram(m, op_point)` renders the dataflow as inline SVG. The
implementation pipes:

```
mod_dataflow(m, op_point)        -- extract edges from A, B, C, D
  → mod__emit_dot(...)           -- format as DOT or Mermaid source
    → uiop:run-program            -- shell out to dot/mmdc
      → write SVG to a temp file in $maxima_tempdir
        → print the path as `"path.svg"\n` to stdout
```

The notebook UI (Aximar / maxima-extension) parses that quoted-path
output, slurps the SVG content, and embeds it as `image/svg+xml` in
the cell's display data. The SVG goes into the saved notebook
directly — the temp file is read once and is no longer needed; if the
OS cleans `/tmp` later the notebook keeps its image.

## Subsystems

| File | Role | Hard deps |
|---|---|---|
| `mochi.mac`, `mochi-loader.lisp` | Core: load, linearise, linear simulate, transfer fn, compose, render diagrams | rumoca, GraphViz (optional, for diagrams) |
| `mochi-nonlinear.mac` | Nonlinear simulation, discrete events | core + `numerics-sundials` |

The split is enforced by a runtime guard in `mochi-nonlinear.mac`
that errors with a clear message if `mod_load` or `np_cvode` aren't
yet bound. Tests are in three separate `rtest_*.mac` files
correspondingly: core, nonlinear, MSL.

## Interaction with the broader stack

mochi sits inside a small ecosystem of related Maxima packages, each
maintained separately:

| Package | Role for mochi |
|---|---|
| [Maxima](https://maxima.sourceforge.io/) | Symbolic algebra, host language. |
| [rumoca](https://github.com/CogniPilot/rumoca) | Modelica → JSON DAE. |
| [numerics](https://github.com/cmsd2/maxima-numerics) | ndarray + linear algebra (`np_eig`, `np_expm`, `np_matmul`, `np_inv`). |
| [numerics-sundials](https://github.com/cmsd2/numerics-sundials) | CVODE/IDA wrappers. Hard dep of `mochi-nonlinear`. |
| [Aximar](https://github.com/cmsd2/aximar) | Notebook execution backend. Recognises `image/svg+xml` and `image/png` via path-emission protocol. |
| [maxima-nbconvert](https://github.com/cmsd2/maxima-nbconvert) | Notebook rendering for the rendered markdown copies in `docs/notebooks/`. |
| [mxpm](https://github.com/cmsd2/mxpm) | Package manager. `mxpm test` runs the rtest files. |

Two cross-cutting fixes during this work:

1. **`numerics-sundials` `CVodeSetInitStep`.** SUNDIALS' default
   initial-step heuristic divides by `||f(t0, y0)||`, which crashes
   when the RHS evaluates to zero at the start. Fixed upstream so
   any plant whose drift vanishes at the operating point can still
   be simulated.
2. **`maxima-nbconvert` `MaximaMarkdownExporter`.** The standard
   markdown exporter dumps Maxima's tex() output verbatim, leaving
   raw `\ifx\endpmatrix\undefined` macros that GitHub doesn't
   render. We added a markdown exporter that runs the existing
   LaTeX-cleanup + Plotly→SVG preprocessors and uses a fenced
   `math` block template — GitHub's markdown renderer handles those
   cleanly.
