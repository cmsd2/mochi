// State-dependent dynamics in an RC circuit: the inflow current sign
// is gated by the capacitor voltage's sign — positive `x' charges
// toward `+u', negative `x' charges toward `-u'.  Not a physical
// circuit anyone would build — it's a minimal test case for mochi's
// linearisation of plants with state-gated dynamics (saturation, dead
// zones, ideal diodes, comparator-driven branches, etc.).
//
// Linearisation is regime-dependent: at `x = 0.5' the model behaves
// like the standard RC charging law (A = -2, stable); at `x = -0.5'
// the sign flip makes it unstable (A = +2).  mod_state_space's
// mod__resolve_cond_ifs pass picks the surviving branch using the
// operating point so the state-dependent `if' doesn't appear in the
// Jacobian — it's been resolved before `diff' sees it.
//
// For dynamics that genuinely cross the boundary during simulation,
// linearisation alone isn't the right tool: linearise multiple times
// (one model per regime, switch at runtime — gain scheduling), or
// use mod_simulate_nonlinear which evaluates the `if' afresh at every
// CVODE step.

model SwitchedRC
  parameter Real R = 1.0;
  parameter Real C = 0.5;
  Real x(start = 0);
  input  Real u;
  output Real y;
equation
  der(x) = if x > 0 then (u - x) / (R * C)
           else          (u + x) / (R * C);
  y = x;
end SwitchedRC;
