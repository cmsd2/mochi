# mochi — gap analysis

Snapshot of where mochi falls short, written from inside the
codebase, prioritised by the impact on a working control engineer
rather than by the interestingness of the fix. Use this as a backlog
of "what would I want to fix next if this were my main project".

For context on how the system is *supposed* to work, see
`architecture.md` in this folder.

## Tier 1 — Could silently produce wrong answers

These are the gaps I'd lose sleep over. Each one is a place where
mochi appears to succeed but the result is wrong, with no clear
signal to the user.

### 1. State-dependent `If` in residuals breaks linearisation — **fixed**

~~A residual like `if x > 0 then a else b` — common in real models
with switched dynamics, dead zones, saturation, ideal diodes —
survives mochi's `mod__simplify_expr` constant-folding pass~~

**Status: fixed.** `mod__resolve_cond_ifs` walks each expression
after parameter substitution but before differentiation. For each
`if-then-else` it finds, it substitutes the op-point INTO THE
CONDITION ONLY and uses Maxima's `is(...)` to decide. If a condition
resolves to true / false, the surviving branch is recursed into; the
other is dropped. Branch bodies retain their state symbols so `diff`
operates on a smooth single-branch expression.

The mathematical interpretation is exactly what a textbook small-
signal linearisation does: pick the regime from the operating point,
linearise within that regime. A user calling `mod_state_space(m, [x
= 0.5, u = 0])` against a `der(x) = if x > 0 then (u - x)/T else (u
+ x)/T` model gets `A = -1/T` (the `x > 0` regime); calling with
`[x = -0.5, u = 0]` gets `A = +1/T` (the unstable other side). For
trajectories that cross the boundary at runtime, the right tool is
`mod_simulate_nonlinear`, which keeps the `if` in the live RHS
expression and re-evaluates at each integration step.

See `examples/SwitchedRC.mo` and the regime-aware tests at the end
of `rtest_mochi.mac` for the worked example.

Caveat: this fix doesn't auto-detect `if`s in equations as discrete
events the way Modelica's full semantics technically does. A bare
`if x > 0 then a else b` (no enclosing `when`) is a continuous
expression in mochi's nonlinear sim — CVODE's adaptive step size
will work through it, but the integrator slows down and can lose
accuracy near the boundary. The principled way to mark a switch as
an event is to use `when (x > 0)` in the `.mo` file, which mochi-
nonlinear's auto-extraction wires into CVODE's rootfinder.

### 2. `assert(...)` statements are silently dropped

MSL's `Modelica.Electrical.Analog.Basic.Resistor` has

```modelica
assert((1 + alpha*(T_heatPort - T_ref)) >= eps,
       "Temperature outside scope of model!");
```

rumoca doesn't surface this in the JSON. mochi never sees it. The
user could pass a temperature that violates the model's assumptions
and get a numerically valid but physically meaningless answer.

**What's likely to fix it:** if rumoca starts emitting asserts in a
known JSON shape, mochi can convert them to Maxima `assume` or
runtime checks. Until rumoca does, this gap is upstream — but it's
worth filing the request and tracking.

### 3. Top-level `output` *connectors* miss the IO scanner

The source-file scanner matches `input Real X` and `output Real X` —
literal string patterns. If a user declares a top-level
`Modelica.Blocks.Interfaces.RealOutput y` (a connector type rather
than a primitive `Real`), the scanner doesn't notice. The variable
ends up in `:y` from rumoca's perspective, gets classified as an
algebraic, and never appears in `mod_get(m, 'outputs)` — so
`mod_state_space` gives an empty `C` and `D`.

**Why it matters:** anyone wiring an MSL plant's output up to a
`RealOutput` block at the top level (a natural pattern for "I want
this signal accessible from outside") loses their output
classification.

**What's likely to fix it:** stop pattern-matching `Real` literally;
walk rumoca's variable metadata and look for things flagged as
`input`/`output` regardless of declared type. Requires rumoca to
expose this metadata cleanly, which I haven't fully checked yet.

## Tier 2 — Wrong-but-rare or workaround-able

### 4. Discrete machinery from MSL Blocks is silently ignored

rumoca emits `m`/`f_m`/`relation`/`initial`/`functions` keys for
models with discrete components — Boolean states, discrete
equations, explicit initialisation equations, and Modelica function
libraries. mochi doesn't read any of them. For continuous-only
simulation this is mostly fine because typical discrete defaults
(e.g. `local_reset = false`) leave the continuous behaviour
unaffected. But:

- A model with a sample-and-hold, a counter, or a Boolean controller
  mode that depends on `m`-state evolution will behave wrong.
- Initial-value equations (`initial { ... }`) that override the
  default `start =` annotations are dropped. Models that set states
  via `initial` rather than `start` will start from zero instead.
- Modelica function libraries — rumoca's `functions` key — are
  ignored. A model using a custom `function f(x, y) input ...; output
  ...; algorithm ...; end f;` that's referenced in residuals would
  get an unresolved function call from mochi's AST walker.

**Why it doesn't matter much yet:** none of our examples need it.
Most users of mochi today are doing continuous control design where
this isn't on the critical path.

**Likely fix:** consume `initial` to override start values during
struct construction (probably one afternoon's work). Discrete state
and Modelica functions are deeper — defer until we have a model
that actually needs them.

### 5. State-only path in `mod_state_space_symbolic`

`mod_state_space_symbolic` uses the older `mod__symbolic_ss` code
path which differentiates *before* parameter substitution and
doesn't do constant folding. For models with parameter-gated `If`
branches (anything in MSL Blocks, including PID), the resulting
symbolic matrices contain `mcond` nodes that look strange but
typically still evaluate correctly when the user later substitutes
parameters by hand. Still, the inconsistency between the symbolic
and numeric paths is a bug.

**Likely fix:** unify the two paths through a shared helper that
takes a "substitute parameters now or leave symbolic" flag.

## Tier 3 — Missing features (acknowledged roadmap)

### 6. DAE simulation via SUNDIALS IDA

For implicit algebraic loops that can't be symbolically causalised
(typical of constrained mechanics, hybrid power systems, and any
plant where multiple unknowns satisfy a coupled nonlinear equation
simultaneously), `mod__causalise` errors out — and even if it
didn't, expression-mode CVODE assumes pure ODE. SUNDIALS IDA solves
implicit DAEs directly. Numerics-sundials would need an `np_dae`
wrapper; mochi-nonlinear would need a code path that detects "
causalisation would error" and routes to IDA.

None of the current examples need this. Known requested by users
hitting the multi-body limits of the simpler approach. Probably a
two-week project.

### 7. Sensitivity analysis (CVODES)

CVODES is the SUNDIALS sensitivity-augmented variant. Computing
∂x(t)/∂p — trajectory sensitivity to parameters — is required for
parameter estimation, optimal-control gradient computation, and
shape-optimisation workflows. Once `numerics-sundials` exposes
CVODES, `mochi-nonlinear` could expose it as a flag on
`mod_simulate_nonlinear`.

Roughly the same scope as the IDA work. Higher payoff for the user
base that does parameter tuning against measured data.

### 8. `mod_export_state_space(m, file)`

Round-trip: dump the symbolic A/B/C/D back to a file (`.mat`,
`.json`, or `.mo`) so a controller designed in mochi can be handed
to another tool. Small project — probably half a day. The reason it
hasn't shipped is just that nobody's asked for it yet.

### 9. FMU export

Probably not. FMU is a C-shared-library format; producing one
requires emitting C code from the symbolic equations and compiling
it. Outside what mochi is designed to do. If a user wants FMU
export, the right path is OpenModelica or Dymola.

### 10. Modelica `redeclare` / `replaceable`

MSL uses these for advanced configurability (e.g. redeclaring a
generic component to a temperature-dependent variant). Models using
them probably fail at the rumoca level today; mochi never sees them.
Status depends on rumoca's roadmap.

### 11. `stream` connectors for thermo-fluid systems

Modelica's specialised connector type for fluid mixing, with semantics
beyond the `flow` / potential pair. Niche — needed only for thermo-
hydraulic models. rumoca-side work first.

## Tier 4 — Test / dev infra

### 12. No CI test runner

The repo has `Docs` and `Deploy docs to GitHub Pages` workflows but
no `mxpm test` workflow. A contributor (or me-in-six-months) could
land a regression in the loader and not see it for weeks. Single
biggest force-multiplier of any item on this list — every other fix
gets safer once CI is running tests.

The constraint is that the test suite uses three separate rtest
files, two of which need optional dependencies (`numerics-sundials`,
MSL). Setting those up in CI is doable but not zero work.

### 13. MSL tests can't run without MSL installed in CI

Tied to (12) — once we have a CI runner, the question becomes
whether to install MSL on it. A `git clone` of the MSL repo plus
setting `MODELICAPATH` in the workflow YAML would do it. The clone
is ~20MB; CI runs would take a small extra hit.

Alternative: have `rtest_mochi_msl.mac` skip gracefully when no MSL
is found, so a non-MSL CI run reports "skipped" rather than
"failed".

### 14. `OPENMODELICAHOME` discovery is untested

mochi's auto-discovery code for `$OPENMODELICAHOME/lib/omlibrary/
Modelica*/` was written based on documented OpenModelica conventions
without an actual OM install to verify against. Almost certainly
correct, but I'd want a real test (or at least someone else
confirming it works on their OM-equipped machine) before promising
it works.

### 15. No tests for `mod_set_source_root` actually changing
       behaviour

The current MSL test verifies that `mod_set_source_root` and
`mod_add_source_root` are accepted as calls and that the path
appears in `mod_source_roots()`. It doesn't verify that the path
actually gets passed to rumoca. If `mochi--source-roots` regressed
to always return `nil`, the existing tests would still pass.

### 16. Notebook outputs aren't deterministic across runs

Plotly→SVG conversion stamps a random `id="defs-XXXXX"` per render.
Every `make render` produces a noisy diff in `docs/notebooks/`,
even when no semantic content has changed. Drowns out actual
content changes during reviews.

Cleanest fix is upstream — the random ID could be deterministic
(based on cell index + content hash) in either Plotly's Kaleido
config or maxima-nbconvert's preprocessor. I haven't dug into where
the easiest insertion point is.

## Tier 5 — Documentation / polish

### 17. `mod_print` doesn't show the `algebraics` field

The connector-flattened intermediate variables (every
`comp.port.flow`, every internal `OnePort.v` and `i`) live in the
struct's `algebraics` field but `mod_print` doesn't show them. For
debugging connector-rich models this matters — when a residual
mentions `tank1_q_out` and the user is wondering "where did that
come from", the answer is in `algebraics` but they have to know to
ask `mod_get(m, 'algebraics)`. Trivial fix.

### 18. No example exercising parameter overrides

`mod_state_space(m, op_point, override_params)` works (it goes
through `mod__resolve_params`'s ordered substitution). No test or
example actually exercises the override path. The interaction
between user overrides and chained parameter expressions is subtle
— a user override of `Td` in MSL's PID needs to flow through to
`D.T = max({Td/Nd, ...})` correctly, which our fixed-point iteration
does, but isn't currently tested. A regression here would be
silent.

### 19. Notebook paths are fragile

Each example notebook starts with `load("../../mochi.mac")` —
relative to its location in `examples/notebooks/`. Move a notebook
to a different folder and it breaks. Workable solution: have a
`load("mochi")` that works once the package is installed via mxpm,
then notebooks don't need to know their location. The dev story
(running notebooks against an in-tree mochi) needs more thought.

---

## Priority recommendation

If you're doing one more round and want maximum impact:

1. **CI test runner** (#12). Force-multiplier on everything else.
2. **Top-level `RealOutput` connector recognition** (#3). Most likely
   tier-1 risk now that #1 is fixed.
3. **`mod_print` shows algebraics** (#17). Trivial; helps everyone
   debug.

If you're doing two rounds, also pick up:

4. **`assert(...)` carry-through** (#2) and the rest of the discrete
   machinery from MSL Blocks (#4). Gets us closer to "MSL just works,
   period".
5. **Notebook output determinism** (#16). Removes review noise.

Anything in tier 3 (DAE/IDA, sensitivity, FMU) is its own multi-week
project and probably warrants its own design doc before starting.
