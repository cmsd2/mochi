model DoubleTankPumps "Two cascaded tanks with two on/off pumps"
  // Tank 1 is filled by pump 1, drains into tank 2.
  // Tank 2 is also fed by pump 2 directly, drains to atmosphere.
  // Linear (orifice-flow) outflow keeps mochi's expression mode happy
  // and avoids sqrt-of-negative drama if a level dips below zero.

  parameter Real A1     = 0.05    "Tank 1 cross-section (m^2)";
  parameter Real A2     = 0.05    "Tank 2 cross-section (m^2)";
  parameter Real C1     = 0.02    "Tank 1 outflow coeff (m^2/s)";
  parameter Real C2     = 0.02    "Tank 2 outflow coeff (m^2/s)";
  parameter Real q_max  = 0.005   "Per-pump max flow (m^3/s)";

  Real h1(start = 0.05)  "Tank 1 level (m)";
  Real h2(start = 0.05)  "Tank 2 level (m)";

  input  Real p1 "Pump 1 command (0 = off, 1 = on, full flow)";
  input  Real p2 "Pump 2 command (0 = off, 1 = on, full flow)";
  output Real y1 "Tank 1 level";
  output Real y2 "Tank 2 level";

equation
  der(h1) = (p1 * q_max - C1 * h1) / A1;
  der(h2) = (C1 * h1 + p2 * q_max - C2 * h2) / A2;
  y1 = h1;
  y2 = h2;
end DoubleTankPumps;
