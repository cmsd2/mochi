# mochi

Read Modelica models into Maxima symbolic equations.

`mochi` is a thin Maxima frontend over [rumoca](https://github.com/CogniPilot/rumoca), a Rust-based Modelica compiler. (The name puns on the `.mo` file extension; the package itself is unaffiliated with the Modelica Association.) `rumoca` does the heavy lifting (parse, flatten, classify variables, emit a JSON DAE); a small Python helper translates the JSON to Maxima syntax; this package wraps the result in a struct and provides operations on top: pretty-printing, state-space linearisation around an operating point.

## Status

Early prototype. Handles a *useful subset* of Modelica:

- single-model files
- **multi-component models composed via direct equations** (e.g. `tank1.q_in = source;` to wire instances together — see `examples/TwoTanks.mo`). Hierarchical names like `tank1.h` are translated to Maxima-safe `tank1_h`; connector-internal variables are auto-classified as algebraic and solved away.
- declarations of parameters, state variables (with `der(...)`), inputs, outputs
- continuous-time equations
- the standard Modelica builtins (`sin`, `cos`, `exp`, `sqrt`, `log`, ...)

Not yet supported: full Modelica `connect()` syntax with custom connector types, inheritance / `extends`, conditional declarations, discrete events, FMU export.

## Prerequisites

- [Maxima](https://maxima.sourceforge.io/) running on SBCL (the standard distribution)
- [Aximar](https://github.com/cmsd2/aximar) for notebook usage
- [rumoca](https://github.com/CogniPilot/rumoca) — `cargo install rumoca`
- [Quicklisp](https://www.quicklisp.org/) (one-time: `mxpm setup quicklisp`); `cl-json` is auto-installed when the package loads.

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
```

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

See `examples/`:

- `RLCircuit.mo` — series RLC, two-state plant, single input/output.
- `DCMotor.mo` — armature current + angular speed, voltage in, speed out.
- `DoubleTank.mo` — cascaded tank levels, inflow in, lower-tank level out (single-model formulation).
- `TwoTanks.mo` — same as DoubleTank but built from two `Tank` *instances* connected by direct equations. Demonstrates dotted-name flattening and algebraic-variable handling.

## Roadmap

- [ ] Library / inheritance support (multi-file `.mo` projects via `--source-root`)
- [ ] Connector models (electrical / mechanical / thermal connectors)
- [ ] Direct simulation via `np_cvode` (skip the linearisation step)
- [ ] Optionally drop the Python helper in favour of a pure-Maxima JSON parser
- [ ] `mod_export_state_space(m, file)` to dump the symbolic SS form

## License

MIT
