# mochi

Read Modelica models into Maxima symbolic equations.

`mochi` is a thin Maxima frontend over [rumoca](https://github.com/CogniPilot/rumoca), a Rust-based Modelica compiler. (The name puns on the `.mo` file extension; the package itself is unaffiliated with the Modelica Association.) `rumoca` does the heavy lifting (parse, flatten, classify variables, emit a JSON DAE); a small Common Lisp loader (`mochi-loader.lisp`) walks the AST and constructs Maxima symbols directly; this package wraps the result in a struct and provides operations on top: pretty-printing, state-space linearisation around an operating point, linear time-domain simulation, transfer functions, plant composition, dataflow diagram rendering. The opt-in `mochi-nonlinear` subsystem adds direct integration of the original nonlinear DAE via SUNDIALS CVODE.

## Status

Early prototype. Handles a *useful subset* of Modelica:

- **Single-model files** (one `model … end Name;` block).
- **Equation-composed multi-component models** (e.g. `tank1.q_in = source;` wiring instances by direct assignment — see `examples/TwoTanks.mo`).
- **Connector-composed multi-component models** with `connect(a.port, b.port)` and `flow`-marked variables — see `examples/RCFilter.mo`. rumoca expands the connectors into Kirchhoff-style equations; mochi translates dotted names to Maxima-safe identifiers, auto-detects instance inputs by walking class declarations, and treats connector-internal variables as algebraics.
- **`extends` inheritance** for component reuse — see `examples/extends/RCFilterExtends.mo` (a small electrical-component library with a `partial model OnePort` base class). rumoca flattens the hierarchy; mochi walks the source's `extends` clauses transitively to inherit `input`/`output` declarations from base classes.
- **Discrete events / hybrid systems** (opt-in, via `mochi-nonlinear`) — `when` clauses with `reinit(state, expr)` resets, see `examples/BouncingBall.mo`. The `'events` opt to `mod_simulate_nonlinear` watches a list of `[event_expr, reset_eqs]` pairs; CVODE detects zero crossings and the loop applies the resets.
- Declarations of parameters, state variables (with `der(...)`), inputs, outputs.
- Continuous-time equations.
- The standard Modelica builtins (`sin`, `cos`, `exp`, `sqrt`, `log`, ...).
- **Linear analysis** (built in): state-space linearisation, transfer functions, step/impulse/arbitrary-input simulation of the linearised model, plant composition.
- **Nonlinear simulation** (opt-in `mochi-nonlinear` subsystem): direct integration of the original nonlinear DAE via SUNDIALS CVODE — no linearisation, no operating-point assumption. Adams (non-stiff) and BDF (stiff) methods.

Not yet supported: conditional declarations, FMU export. Modelica Standard Library is supported via rumoca's `--source-root` mechanism — see below.

## Prerequisites

- [Maxima](https://maxima.sourceforge.io/) running on SBCL (the standard distribution)
- [Aximar](https://github.com/cmsd2/aximar) for notebook usage
- [rumoca](https://github.com/CogniPilot/rumoca) — `cargo install rumoca`
- [Quicklisp](https://www.quicklisp.org/) (one-time: `mxpm setup quicklisp`); `cl-json` is auto-installed when the package loads.

Optional, only for the `mochi-nonlinear` subsystem:

- [`numerics-sundials`](https://github.com/cmsd2/numerics-sundials) — `mxpm install numerics-sundials`. Brings in SUNDIALS C bindings; needed only if you want to integrate the nonlinear DAE directly. Pure linear analysis (state-space, Bode, transfer fn, dataflow diagrams) doesn't need it.

Optional, for using Modelica Standard Library components in your `.mo` files:

- The [Modelica Standard Library](https://github.com/modelica/ModelicaStandardLibrary) (MSL 4.x) at any directory `D` such that `D/Modelica/package.mo` exists.

  Tell mochi where to find it:

  - **`MODELICAPATH` environment variable** (the standard Modelica-tools convention, defined in [MLS §13.2.4](https://specification.modelica.org/master/packages.html#mapping-package-class-structures-to-a-hierarchical-file-system)). Colon-separated on Unix, semicolon-separated on Windows. Example:

    ```sh
    export MODELICAPATH=/path/to/your/clone
    ```

  - **Or programmatically from Maxima:**

    ```maxima
    mod_set_source_root("/path/to/clone1", "/path/to/clone2")$
    /* or */
    mod_add_source_root("/path/to/clone")$
    ```

  - **OpenModelica users** get auto-discovery for free: if neither `MODELICAPATH` nor the API is set, mochi falls back to `$OPENMODELICAHOME/lib/omlibrary/Modelica*/` (the OM-managed install location, defaulting to `/opt/openmodelica` if `OPENMODELICAHOME` isn't exported). No fabricated paths beyond that.

  Run `mod_source_roots()` to inspect the resolved list. If you don't have MSL installed and aren't using OpenModelica, get a fresh clone:

  ```sh
  git clone --depth 1 --branch v4.0.0 https://github.com/modelica/ModelicaStandardLibrary
  export MODELICAPATH="$PWD/ModelicaStandardLibrary"
  ```

The `RUMOCA_BIN` environment variable can override the rumoca path; otherwise the loader looks in `$PATH` and `~/.cargo/bin`.

## Install

Once published:

```
mxpm install mochi
```

For now (development install):

```
cd ~/Development/mochi
# or:  mxpm install --path .
```

## Usage

```maxima
load("mochi")$

m : mod_load("examples/RLCircuit.mo")$
mod_print(m)$

/* Linearise around the equilibrium iL = vC = 0, Vin = 0,
   keeping default parameters from the .mo file. */
[A, B, C, D] : mod_state_space(m, [iL = 0, vC = 0, Vin = 0])$

/* Or override parameters explicitly: */
[A, B, C, D] : mod_state_space(m,
                               [iL = 0, vC = 0, Vin = 0],
                               [R = 2.0, L = 0.4, Ccap = 0.5])$

/* Step / impulse / arbitrary-input simulation */
[t, y_step] : mod_step(m, [iL = 0, vC = 0, Vin = 0], 5.0)$
[t, y_imp]  : mod_impulse(m, [iL = 0, vC = 0, Vin = 0], 5.0)$
[t, y]      : mod_simulate(m, [iL = 0, vC = 0, Vin = 0],
                           lambda([t], [sin(2*t)]),  /* input vector */
                           5.0)$

/* Symbolic transfer function H(s) = C(sI - A)^{-1} B + D */
H : mod_transfer_function(m, [iL = 0, vC = 0, Vin = 0])$

/* Compose two plants */
SS  : mod_state_space(m, [iL = 0, vC = 0, Vin = 0])$
SS2 : mod_cascade(SS, SS)$            /* output of 1 → input of 2 */
SS3 : mod_parallel(SS, SS)$           /* same input, sum outputs */
SS4 : mod_unity_feedback(SS, 2.0)$    /* close loop with K = 2 */

/* Render a dataflow diagram inline.  mod_diagram is the one-shot
   front-end: it builds the diagram source and pipes it through the
   chosen renderer to an SVG in the Maxima temp directory; the notebook
   UI picks up the printed path and embeds it as image/svg+xml. */
mod_diagram(m, [iL = 0, vC = 0, Vin = 0])$
mod_diagram(m, [iL = 0, vC = 0, Vin = 0], ['renderer = 'mermaid])$

/* Or get the diagram as text (Mermaid / GraphViz DOT / raw edges) */
mod_to_mermaid(m, [iL = 0, vC = 0, Vin = 0]);
mod_to_dot    (m, [iL = 0, vC = 0, Vin = 0]);
mod_dataflow  (m, [iL = 0, vC = 0, Vin = 0], ['format = 'edges]);
```

`mod_diagram` accepts:

- `'diagram` — only `'dataflow` is supported right now (default).
- `'renderer` — `'dot` (GraphViz, default) or `'mermaid` (Mermaid CLI).
- `'threshold`, `'params` — passed through to `mod_dataflow` /
  `mod_state_space`.

Renderer prerequisites:

- **`'dot`**  — GraphViz on `$PATH` (or set `DOT_BIN`). `brew install graphviz`.
- **`'mermaid`** — Mermaid CLI on `$PATH` (or set `MMDC_BIN`):
  `npm install -g @mermaid-js/mermaid-cli`. The CLI itself depends on a
  headless Chrome via puppeteer.

### Nonlinear simulation (opt-in)

Everything above linearises around an operating point. To integrate the
*original* nonlinear DAE — useful for closed-loop validation, or any
plant where small-angle approximations don't hold — load the
`mochi-nonlinear` subsystem:

```maxima
load("mochi")$
load("numerics-sundials")$    /* hard dep — provides np_cvode */
load("mochi-nonlinear")$

m : mod_load("examples/Pendulum.mo")$

/* Step torque, starting from upright (theta = 0).  No linearisation —
   the integrator sees the actual sin(theta) gravity term. */
[t, theta] : mod_step_nonlinear(m, [0.0, 0.0], 4.0,
                                 ['magnitude = 5])$

/* Compare to the linearised step: small angles agree, large angles diverge. */
[t_lin, theta_lin] : mod_step(m, [theta = 0, omega = 0, tau = 0], 4.0,
                               ['magnitude = 5])$

/* Custom input lambda, BDF for stiff systems, etc. */
[t, y] : mod_simulate_nonlinear(m, [theta = 0.1, omega = 0],
                                 lambda([t], [sin(2*t)]),
                                 10.0,
                                 ['method = 'bdf, 'rtol = 1e-10])$
```

Same `[t_list, y_list]` return shape as the linear `mod_simulate`. See
`doc/mochi.md` § *Subsystem: mochi-nonlinear* for the full option list.

`mod_state_space` returns numeric matrices `A, B, C, D` such that the linearisation around the operating point is

    dx/dt = A·(x − x₀) + B·(u − u₀)
    y     = C·(x − x₀) + D·(u − u₀)

For the RLC example with default parameters:

    A = [[ -2, -2 ], [ 1, 0 ]]
    B = [[  2 ], [ 0 ]]
    C = [[  0,  1 ]]
    D = [[ 0 ]]

## How it works

```
  .mo file ─► rumoca compile --json ─► JSON DAE
                                    │
                                    ▼
                          mochi-loader.lisp
                       (cl-json parses, walks AST,
                        constructs Maxima struct)
                                    │
                                    ▼
                              mochi.mac
                       (mod_state_space, mod_simulate,
                        mod_transfer_function, etc.)
```

`mod_load` is implemented in Common Lisp via `mochi-loader.lisp` — it calls `rumoca compile --json` via `uiop:run-program`, parses the JSON with `cl-json`, walks the AST, and constructs a Maxima list directly with fresh Lisp symbols. **No temp file, no Python helper, and no Maxima-side eval pass that could disturb session state** — user variables that happen to share a name with a model parameter are unaffected.

## Examples

### Models

See `examples/`:

- `RLCircuit.mo` — series RLC, two-state plant, single input/output.
- `DCMotor.mo` — armature current + angular speed, voltage in, speed out.
- `DoubleTank.mo` — cascaded tank levels, inflow in, lower-tank level out (single-model formulation).
- `TwoTanks.mo` — same as DoubleTank but built from two `Tank` *instances* connected by direct equations. Demonstrates dotted-name flattening and algebraic-variable handling.
- `RCFilter.mo` — RC low-pass filter using proper Modelica `connect(...)` syntax over an `ElectricalPin` connector class. Demonstrates flow/potential expansion and per-instance input detection.
- `Pendulum.mo` — damped pendulum with `sin(theta)` gravity term — used by the nonlinear-simulation notebook.
- `BouncingBall.mo` — classic hybrid-system test case. Continuous free-fall plus a `when h <= 0 and v < 0 then reinit(v, -e * pre(v))` clause for the ground impact. Demonstrates discrete events via the `mochi-nonlinear` subsystem's `'events` opt.
- `extends/RCFilterExtends.mo` — same RC filter as above, but composed from a `partial model OnePort` base class that Resistor / Capacitor / VoltageSource extend. Demonstrates `extends` inheritance: rumoca flattens the hierarchy and mochi follows the parent chain to inherit `input` / `output` declarations.
- `msl/RCFilterMSL.mo` — same RC filter again, but built from `Modelica.Electrical.Analog.Basic.{Resistor, Capacitor, Ground}` and `Modelica.Electrical.Analog.Sources.SignalVoltage` from the Modelica Standard Library. No hand-rolled connectors or base classes. Requires MSL installed (see Prerequisites). Linearises to identical `A = -2, B = 2, C = 1, D = 0`.

### Notebooks

The `examples/notebooks/` directory has executable `.macnb` notebooks
that walk through the workflow end-to-end. Rendered markdown copies
(committed alongside the sources for in-browser reading) live in
`docs/notebooks/`:

- [`01_linear_analysis`](docs/notebooks/01_linear_analysis.md) — load a `.mo` file, linearise around an operating point, run step/impulse/transfer-function analysis on RLCircuit, DCMotor, and DoubleTank, then compose two RLCs in cascade.
- [`02_multi_component`](docs/notebooks/02_multi_component.md) — multi-instance composition (TwoTanks via direct equations, RCFilter via `connect(...)`) and dataflow-diagram rendering with `mod_diagram`.
- [`03_nonlinear`](docs/notebooks/03_nonlinear.md) — direct nonlinear simulation via the `mochi-nonlinear` subsystem, with the Pendulum showing where the linearised model and the true `sin(theta)` dynamics visibly diverge.
- [`04_events`](docs/notebooks/04_events.md) — discrete events / hybrid systems via Modelica's `when` clauses. The bouncing ball: continuous free-fall + a discontinuous velocity reset when the ball hits the floor. Includes the linearised dataflow diagram (continuous part), trajectory plots with and without event handling, and a phase portrait showing the velocity jumps as vertical edges at $h = 0$.
- [`05_msl`](docs/notebooks/05_msl.md) — modelling with the Modelica Standard Library: source-root discovery, an RC plant built from `Modelica.Electrical.Analog.Basic.{Resistor, Capacitor, Ground}` (matching the hand-rolled version's dynamics), and a closed-loop control system from `Modelica.Blocks.Continuous.{FirstOrder, PID}` + `Modelica.Blocks.Math.Feedback` — linearised end-to-end, with eigenvalues showing the dominant complex pair and the fast derivative-filter pole.

To regenerate the markdown after editing a notebook:

```
make            # execute every notebook (writes outputs back into the .macnb)
                # then render to docs/notebooks/*.md
make render     # render only — assumes outputs are already in the .macnb
make clean      # delete the rendered docs
```

(Requires [`uv`](https://docs.astral.sh/uv/) for the Python toolchain
and `aximar-mcp` on `$PATH` — or override via `make AXIMAR_MCP=/path/to/aximar-mcp`.)

## Roadmap

- [x] `extends` inheritance for component reuse (single-file; `examples/extends/RCFilterExtends.mo`).
- [x] Multi-file `.mo` projects via rumoca's `--source-root` (`MODELICAPATH` env var + auto-discovery).
- [x] Modelica Standard Library integration for plants and analyses (`examples/msl/`), including `Modelica.Blocks` controllers. The linearisation pipeline does parameter substitution + constant folding before computing the Jacobian, so MSL's `If`-gated branches (e.g. `Modelica.Blocks.Continuous.PID`'s derivative-filter shortcut) collapse to clean state-space matrices.
- [x] Connector models (electrical / mechanical / thermal connectors) — covered by MSL.
- [x] Direct nonlinear simulation via `np_cvode` (`mochi-nonlinear` subsystem)
- [ ] DAE simulation via SUNDIALS IDA for index-1 algebraic loops that can't be symbolically causalised
- [x] Discrete events / hybrid systems (`when`, `reinit`) — `examples/BouncingBall.mo`. Ideal switches and full multi-event coordination are still TODO.
- [ ] `mod_export_state_space(m, file)` to dump the symbolic SS form

## License

MIT
