// RC low-pass filter built entirely from Modelica Standard Library
// components — no hand-rolled connector class, no hand-rolled OnePort
// base.  Demonstrates that mochi can consume MSL via rumoca's
// --source-root mechanism (set MODELICAPATH or use mod_set_source_root).
//
// Same dynamics as examples/RCFilter.mo and examples/extends/RCFilterExtends.mo:
//   A = -1/(R*C) = -2,  B = 1/(R*C) = 2,  C = 1,  D = 0.
//
// Requires MSL 4.x at one of the source-root locations mochi searches
// (default: ~/.modelica/library/Modelica/).  See the README for install
// instructions.

model RCFilterMSL
  Modelica.Electrical.Analog.Basic.Resistor   r1(R = 1.0);
  Modelica.Electrical.Analog.Basic.Capacitor  c1(C = 0.5);
  Modelica.Electrical.Analog.Basic.Ground     gnd;
  Modelica.Electrical.Analog.Sources.SignalVoltage src;
  input  Real Vin   "external driving voltage";
  output Real Vcap  "capacitor voltage";
equation
  src.v = Vin;
  Vcap  = c1.v;
  connect(src.p, r1.p);
  connect(r1.n,  c1.p);
  connect(c1.n,  gnd.p);
  connect(src.n, gnd.p);
end RCFilterMSL;
