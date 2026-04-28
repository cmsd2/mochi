# Package mochi

## Introduction to mochi

`mochi` reads [Modelica](https://modelica.org/) plant models into Maxima as symbolic equations, using [rumoca](https://github.com/CogniPilot/rumoca) as the parser/compiler backend. The intended use is **pulling plant equations out of a model library and into a control-design workflow**: derive symbolic state-space matrices from a `.mo` file, feed them into LQR/Kalman/observer/MPC code, and never hand-transcribe equations again.

To use the package:

```
load("mochi");
```

## Definitions for mochi

### Function: mod_load (path)

Parse a `.mo` file. Returns a *model struct* — a list of `[tag = value]` equations
with these fields:

- `name` — the model's name (string)
- `params` — `[[symbol, default_value], ...]`
- `states` — list of state-variable symbols (those with `der(...)` in equations)
- `derivs` — list of derivative symbols, one per state, named `der_<state>`
- `inputs` — list of input-variable symbols
- `outputs` — list of output-variable symbols
- `initial` — `[[state, initial_value], ...]` from `start = ...` annotations
- `residuals` — list of equations in residual form (each ≡ 0)

Use `mod_get(m, 'states)` to fetch a field.

### Function: mod_print (m)

Pretty-print a model struct.

### Function: mod_state_space (m, op_point)
### Function: mod_state_space (m, op_point, override_params)

Linearise the model around an operating point. Returns `[A, B, C, D]` numeric matrices such that

    dx/dt = A · (x − x₀) + B · (u − u₀)
    y     = C · (x − x₀) + D · (u − u₀).

`op_point` is a list of equations specifying a value for each state and each input,
e.g. `[iL = 0, vC = 0, Vin = 0]`.

`override_params` is an optional list of equations like `[R = 2.0, L = 0.4]` that
overrides the defaults from the `.mo` file.

### Function: mod_get (m, key)

Fetch one field from a model struct: `mod_get(m, 'params)`.

## Example

```maxima
load("mochi");

m : mod_load("examples/RLCircuit.mo");
mod_print(m);

[A, B, C, D] : mod_state_space(m, [iL = 0, vC = 0, Vin = 0]);
```
