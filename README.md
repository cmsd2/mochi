# mochi

Read Modelica models into Maxima symbolic equations.

`mochi` is a thin Maxima frontend over [rumoca](https://github.com/CogniPilot/rumoca), a Rust-based Modelica compiler. (The name puns on the `.mo` file extension; the package itself is unaffiliated with the Modelica Association.) `rumoca` does the heavy lifting (parse, flatten, classify variables, emit a JSON DAE); a small Python helper translates the JSON to Maxima syntax; this package wraps the result in a struct and provides operations on top: pretty-printing, state-space linearisation around an operating point.

## Status

Early prototype. Handles a *useful subset* of Modelica:

- single-model files (no inheritance / connectors / hybrid blocks)
- declarations of parameters, state variables (with `der(...)`), inputs, outputs
- continuous-time equations
- the standard Modelica builtins (`sin`, `cos`, `exp`, `sqrt`, `log`, ...)

Not yet supported: connector models, inheritance / `extends`, conditional declarations, discrete events, FMU export.

## Prerequisites

- [Maxima](https://maxima.sourceforge.io/)
- [Aximar](https://github.com/cmsd2/aximar) for notebook usage
- [rumoca](https://github.com/CogniPilot/rumoca) — `cargo install rumoca`
- Python 3 (system Python is fine; only used by the helper script)

The `RUMOCA_BIN` environment variable can override the rumoca path; otherwise the helper looks in `$PATH` and `~/.cargo/bin`.

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
                       tools/rumoca_to_mac.py
                       (walks JSON AST, emits Maxima syntax)
                                    │
                                    ▼
                              modelica.mac
                       (loads the emitted file,
                        computes Jacobians, etc.)
```

The Python helper is invoked from Maxima via a `system()` call to a temp file; the result is `batch`-loaded back into Maxima.

## Examples

See `examples/`:

- `RLCircuit.mo` — series RLC, two-state plant, single input/output.
- `DCMotor.mo` — armature current + angular speed, voltage in, speed out.
- `DoubleTank.mo` — cascaded tank levels, inflow in, lower-tank level out.

## Roadmap

- [ ] Library / inheritance support (multi-file `.mo` projects via `--source-root`)
- [ ] Connector models (electrical / mechanical / thermal connectors)
- [ ] Direct simulation via `np_cvode` (skip the linearisation step)
- [ ] Optionally drop the Python helper in favour of a pure-Maxima JSON parser
- [ ] `mod_export_state_space(m, file)` to dump the symbolic SS form

## License

MIT
