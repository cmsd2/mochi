// Closed-loop control system built entirely from Modelica.Blocks:
//   setpoint --[+]--> PID --> plant ---> output
//                ^                       |
//                +-----------------------+
//
// The plant is a first-order system (dy/dt = (-y + u) / T), the
// controller is a PID block, and the loop is closed via a Feedback
// (subtraction) block.  Demonstrates that mochi linearises models
// containing MSL Blocks correctly — the parameter-gated If branches
// in `Modelica.Blocks.Continuous.PID' (e.g. the derivative-filter's
// zero-gain shortcut) collapse to clean Jacobians via the constant-
// folding pass in mod_state_space.
//
// States (3): plant.y (first-order plant output), pid.I.y
// (integrator), pid.D.x (derivative filter).  Linearises around the
// origin to a 3x3 A matrix with one fast real pole (~ -110, the
// derivative filter) and a complex pair (~ -0.91 +- 0.99j, the
// dominant closed-loop dynamics).

model PIDClosedLoop
  Modelica.Blocks.Continuous.FirstOrder plant(T = 1.0, k = 1.0);
  Modelica.Blocks.Continuous.PID        pid(k = 1.0, Ti = 0.5, Td = 0.1);
  Modelica.Blocks.Sources.Step          setpoint(height = 1.0, startTime = 0);
  Modelica.Blocks.Math.Feedback         fb;
equation
  connect(setpoint.y, fb.u1);
  connect(fb.y,       pid.u);
  connect(pid.y,      plant.u);
  connect(plant.y,    fb.u2);
end PIDClosedLoop;
