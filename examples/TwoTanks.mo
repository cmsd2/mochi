model Tank
  // Single tank with linear outlet flow.
  parameter Real A = 1.0     "tank cross-sectional area";
  parameter Real k = 0.5     "outlet coefficient";
  Real h(start = 0);
  input Real q_in;
  output Real q_out;
equation
  A * der(h) = q_in - k*h;
  q_out = k*h;
end Tank;

model TwoTanks
  // Two tanks in cascade: source feeds tank1, tank1 feeds tank2.
  // Output is the level of the second tank.
  Tank tank1;
  Tank tank2(A = 2.0);
  input Real source;
  output Real level2;
equation
  tank1.q_in = source;
  tank2.q_in = tank1.q_out;
  level2 = tank2.h;
end TwoTanks;
