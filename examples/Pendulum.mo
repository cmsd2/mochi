model Pendulum
  // Damped pendulum driven by an applied torque.
  // States: angular position theta, angular velocity omega.
  // Output: angle theta.
  // Genuinely nonlinear: the gravity term -g/L * sin(theta) means the
  // linearised model (sin(theta) ~ theta) only matches near theta = 0.
  parameter Real g = 9.81;
  parameter Real L = 1.0;
  parameter Real b = 0.1;        // viscous damping
  Real theta(start = 0);
  Real omega(start = 0);
  input Real tau;                // applied torque
  output Real y;
equation
  der(theta) = omega;
  der(omega) = -(g / L) * sin(theta) - b * omega + tau;
  y = theta;
end Pendulum;
