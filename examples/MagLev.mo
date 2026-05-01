model MagLev "Ball position above an electromagnet"
  // Classic unstable plant: a steel ball is suspended below an
  // electromagnet by an attractive force ~ i^2 / x^2.  Open-loop the
  // ball either falls or slams into the magnet — feedback is required.

  parameter Real ball_mass = 0.02     "Ball mass (kg)";
  parameter Real grav      = 9.81     "Gravity (m/s^2)";
  parameter Real km        = 1.6e-4   "Magnetic constant (N*m^2/A^2)";
  parameter Real x_ref     = 0.008    "Setpoint position (m)";

  // Bias current at the operating point (force balance:
  //   km * i^2 / x^2 = ball_mass * g
  // mochi's fixed-point parameter resolver folds this to a number.
  parameter Real i_bias = sqrt(ball_mass * grav * x_ref^2 / km);

  Real x(start = 0.011)  "Position above magnet (m), positive = below magnet";
  Real v(start = 0.0)    "Vertical velocity (m/s, positive = falling)";

  input  Real coil_i "Coil current (A)";
  output Real y      "Position (m), for sensing";

equation
  der(x) = v;
  der(v) = grav - km * coil_i^2 / (ball_mass * x^2);
  y = x;
end MagLev;
