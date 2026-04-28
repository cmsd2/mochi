model DCMotor
  // DC motor with armature dynamics + mechanical inertia.
  // States: armature current ia, angular speed omega.
  // Input: armature voltage va. Output: speed.
  parameter Real Ra = 1.0    "armature resistance (Ω)";
  parameter Real La = 0.01   "armature inductance (H)";
  parameter Real Kt = 0.05   "torque constant (Nm/A)";
  parameter Real Ke = 0.05   "back-EMF constant (V·s/rad)";
  parameter Real Jm = 0.001  "rotor inertia (kg·m²)";
  parameter Real Bv = 0.0001 "viscous damping (Nm·s/rad)";
  Real ia(start = 0);
  Real omega(start = 0);
  input Real va;
  output Real y;
equation
  La * der(ia)    = va - Ra*ia - Ke*omega;
  Jm * der(omega) = Kt*ia - Bv*omega;
  y = omega;
end DCMotor;
