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

### Function: mod_state_space_symbolic (m, op_point)

Same shape as `mod_state_space`, but returns Maxima matrices with **parameters
left as symbols**. Useful for inspecting the closed-form A, B, C, D before
substituting numerical values.

For nonlinear plants, the Jacobian still depends on the operating point;
`op_point` is substituted in but parameters are not.

### Function: mod_simulate (m, op_point, u_fn, t_end, [opts])

Simulate the linearised model from zero state with input `u_fn(t)`. `u_fn`
must return a list of length `|inputs|`. Returns `[t_list, y_list]`. For SISO
plants `y_list` is a flat list; for multi-output plants it is a list of
length-`|outputs|` lists.

Optional `opts` is a list of equations, recognising:

- `dt` — sample period (default `t_end/500`)
- `params` — parameter override list (default: `.mo` file's defaults)

### Function: mod_step (m, op_point, t_end, [opts])
### Function: mod_impulse (m, op_point, t_end, [opts])

Convenience wrappers over `mod_simulate`:

- `mod_step` applies a step to the chosen input (`input_index`, default 1) of
  magnitude `magnitude` (default 1), with all other inputs held at zero.
- `mod_impulse` applies a discrete impulse approximation: `u[0] = magnitude/dt`
  on the chosen input, zero thereafter, so that ∫u dt ≈ magnitude.

Both accept the same opts as `mod_simulate` plus `input_index` and `magnitude`.

### Function: mod_cascade (SS1, SS2)
### Function: mod_parallel (SS1, SS2)
### Function: mod_unity_feedback (SS, K)

Block-stack two state-space tuples to form a connected system, returning the
result as a tuple `[A, B, C, D]` ready for further composition.

- `mod_cascade(SS1, SS2)` — output of system 1 fed into input of system 2.
- `mod_parallel(SS1, SS2)` — same input to both, outputs summed.
- `mod_unity_feedback(SS, K)` — close a negative feedback loop with scalar
  gain `K`. Assumes `D = 0` for the open loop.

`SS_i` is `[A, B, C, D]` (Maxima matrices), as returned by `mod_state_space`
or `mod_state_space_symbolic`.

### Function: mod_transfer_function (m, op_point)
### Function: mod_transfer_function (m, op_point, override_params)

Compute `H(s) = C·(sI − A)⁻¹·B + D` symbolically. Returns a Maxima expression
(scalar for SISO, matrix for MIMO). The Laplace variable is `s`.

Without `override_params`, the parameter defaults from the `.mo` file are
substituted; with them, the supplied values are used.

### Function: mod_get (m, key)

Fetch one field from a model struct: `mod_get(m, 'params)`.

### Function: mod__causalise (m)

Symbolic causalisation: solve the model's residual equations for the
derivative, algebraic, and output variables, returning explicit equation
lists.

    [der_eqs, alg_eqs, output_eqs] = mod__causalise(m)

Each returned list has shape `[[lhs_sym, rhs_expr], ...]` where the RHS is a
Maxima expression in the states, inputs, and parameters. No parameter
substitution and no operating-point substitution is applied — the result is
purely symbolic.

Used directly by `mod_state_space` for the linearisation Jacobians, and by
[`mochi-nonlinear`](https://github.com/cmsd2/mochi-nonlinear) for direct
integration of the nonlinear DAE without linearising first.

Errors if the residuals can't be solved explicitly for the unknowns — that
would mean a true implicit algebraic loop, which needs a numeric solver
inside the integration loop (lives in `mochi-nonlinear`).

### Function: mod_dataflow (m, op_point, [opts])

Render the linearised state-space connectivity as a dataflow diagram. Nonzero
entries of `A`, `B`, `C`, `D` become labelled edges:

    B[i,j] ≠ 0   →   input[j]  ──>  state[i]   (label: B[i,j])
    A[i,j] ≠ 0   →   state[j]  ──>  state[i]   (label: A[i,j])
    C[k,i] ≠ 0   →   state[i]  ──>  output[k]  (label: C[k,i])
    D[k,j] ≠ 0   →   input[j]  ──>  output[k]  (label: D[k,j])

`opts` is an optional list of equations:

- `format` — `'dot` (GraphViz, default), `'mermaid`, or `'edges` (raw list).
- `threshold` — hide edges with `|weight| < threshold` (default `1e-12`).
- `params` — parameter override list, passed to `mod_state_space`.

Returns a Maxima string for `'dot` / `'mermaid`, or the raw edge list
`[[from, to, weight], ...]` for `'edges`.

### Function: mod_to_dot (m, op_point)
### Function: mod_to_dot (m, op_point, override_params)
### Function: mod_to_mermaid (m, op_point)
### Function: mod_to_mermaid (m, op_point, override_params)

Thin wrappers over `mod_dataflow` that pin the format. Return the diagram
source as a Maxima string, suitable for embedding in markdown / piping into
an external renderer.

### Function: mod_diagram (m, op_point, [opts])

One-shot front-end: build a diagram of the model and render it inline. Shells
out to the chosen renderer to write an SVG into Maxima's temp directory and
prints the path so the maxima-extension / Aximar notebook UI picks it up and
embeds the SVG content as `image/svg+xml` in the cell output (the notebook is
self-contained — the temp file is read once at evaluation time and is no
longer needed).

Returns the SVG file path as a string.

`opts` is an optional list of equations:

- `diagram` — only `'dataflow` is supported right now (default).
- `renderer` — `'dot` (GraphViz, default) or `'mermaid` (Mermaid CLI).
  `'graphviz` is accepted as an alias for `'dot`.
- `threshold`, `params` — passed through to `mod_dataflow` /
  `mod_state_space`.

Renderer prerequisites:

- `'dot` — GraphViz `dot` on `$PATH` (or set the `DOT_BIN` environment
  variable). `brew install graphviz` on macOS.
- `'mermaid` — Mermaid CLI `mmdc` on `$PATH` (or set `MMDC_BIN`):
  `npm install -g @mermaid-js/mermaid-cli`. The CLI itself depends on a
  headless Chrome via puppeteer.

## Subsystem: mochi-nonlinear

Direct nonlinear simulation of the original DAE — no linearisation, no
operating-point assumption. Lives in `mochi-nonlinear.mac` and is loaded
on demand:

```maxima
load("mochi");
load("numerics-sundials");      /* hard dep — provides np_cvode */
load("mochi-nonlinear");
```

The hard dep on [`numerics-sundials`](https://github.com/cmsd2/numerics-sundials)
is why this is a separate file rather than auto-loaded with mochi:
users doing pure linear analysis don't need SUNDIALS installed.

### Function: mod_simulate_nonlinear (m, x0, u_fn, t_end, [opts])

Integrate the model's nonlinear DAE via SUNDIALS CVODE.

- `x0` — initial state vector. Either a flat list of floats in the order
  given by `mod_get(m, 'states)`, or a list of equations
  `[state_sym = value, ...]`.
- `u_fn` — `lambda([t], [u1, u2, ...])` returning the input vector.
- `t_end` — end time.
- `opts` — list of equations:
  - `dt` — sample period (default `t_end/500`).
  - `params` — parameter override list (default: `.mo` defaults).
  - `method` — `'adams` (default, non-stiff) or `'bdf` (stiff systems).
  - `rtol` — relative tolerance (default `1e-8`).
  - `atol` — absolute tolerance (default `1e-8`).
  - `return` — `'outputs` (default) or `'states` to get the full state
    trajectory instead of the output trajectory.

Returns `[t_list, y_list]`. For SISO `y_list` is a flat list; for
multi-output it's a list of length-`|outputs|` lists.

### Function: mod_step_nonlinear (m, x0, t_end, [opts])
### Function: mod_impulse_nonlinear (m, x0, t_end, [opts])

Convenience wrappers over `mod_simulate_nonlinear` mirroring the linear
`mod_step` / `mod_impulse`. Same opts plus `'input_index` and `'magnitude`.

## Example

```maxima
load("mochi");

m : mod_load("examples/RLCircuit.mo");
mod_print(m);

[A, B, C, D] : mod_state_space(m, [iL = 0, vC = 0, Vin = 0]);

/* Step response and impulse response */
[t, y_step] : mod_step(m, [iL = 0, vC = 0, Vin = 0], 5.0);
[t, y_imp]  : mod_impulse(m, [iL = 0, vC = 0, Vin = 0], 5.0);

/* Symbolic transfer function */
H : mod_transfer_function(m, [iL = 0, vC = 0, Vin = 0]);

/* Compose a cascade of two RLCs */
SS : mod_state_space(m, [iL = 0, vC = 0, Vin = 0]);
[A2, B2, C2, D2] : mod_cascade(SS, SS);

/* Render a dataflow diagram inline */
mod_diagram(m, [iL = 0, vC = 0, Vin = 0]);

/* Direct nonlinear simulation (opt-in subsystem; needs numerics-sundials) */
load("numerics-sundials");
load("mochi-nonlinear");
m_pend : mod_load("examples/Pendulum.mo");
[t, theta] : mod_step_nonlinear(m_pend, [0.0, 0.0], 4.0,
                                 ['magnitude = 5]);
```

## Note: `mod_load` does not touch session state

`mod_load` runs in Common Lisp (`mochi-loader.lisp`) and constructs the model
struct directly — it interns fresh Lisp symbol objects for every parameter,
state, derivative, input, and output name. **No Maxima-side eval pass means
no global bindings are created or destroyed**, so a user variable in your
session that shares a name with a model parameter (e.g. `A2` for
`DoubleTank`'s second-tank area) is safe across `mod_load` calls.
