model DoubleTank
  // Two-tank cascade: inflow into tank 1, gravity-fed flow to tank 2,
  // outflow from tank 2. Linearised flows (proportional to level).
  parameter Real A1 = 1.0    "tank 1 area";
  parameter Real A2 = 1.0    "tank 2 area";
  parameter Real k1 = 0.5    "outlet coefficient tank 1";
  parameter Real k2 = 0.3    "outlet coefficient tank 2";
  Real h1(start = 0);
  Real h2(start = 0);
  input Real q_in;
  output Real y;
equation
  A1 * der(h1) = q_in - k1*h1;
  A2 * der(h2) = k1*h1 - k2*h2;
  y = h2;
end DoubleTank;
