// RC low-pass filter built from a small electrical-component library
// that uses `extends` to share the two-pin interface.  Compare with
// RCFilter.mo which repeats the connector + Kirchhoff equations in
// every component.
//
// `partial model OnePort' factors out the boilerplate that's common
// to every 2-pin electrical element: declare positive / negative
// pins, define voltage drop, and assert current sums to zero.  The
// concrete components (Resistor, Capacitor, VoltageSource) then only
// need to declare their own constitutive equation.

connector ElectricalPin
  Real v;
  flow Real i;
end ElectricalPin;

partial model OnePort
  // Common interface for any 2-pin electrical element.
  ElectricalPin p, n;
  Real v;       // voltage drop p.v - n.v
  Real i;       // current into p (= -current into n)
equation
  v = p.v - n.v;
  0 = p.i + n.i;
  i = p.i;
end OnePort;

model Resistor
  extends OnePort;
  parameter Real R = 1.0;
equation
  v = R * i;
end Resistor;

model Capacitor
  extends OnePort;
  parameter Real Ccap = 0.5;
  Real vC(start = 0) "capacitor voltage = state variable";
equation
  v = vC;
  Ccap * der(vC) = i;
end Capacitor;

model Ground
  ElectricalPin p;
equation
  p.v = 0;
end Ground;

model VoltageSource
  extends OnePort;
  input Real Vin "driving voltage";
equation
  v = Vin;
end VoltageSource;

model RCFilterExtends
  // Same plant as RCFilter, but composed from the OnePort-extending
  // components above.  The dynamics are identical — A = -1/(R*Ccap),
  // B = 1/(R*Ccap), C = 1, D = 0 — just expressed via inheritance.
  Resistor r1(R = 1.0);
  Capacitor c1(Ccap = 0.5);
  VoltageSource vs;
  Ground gnd;
  output Real v_cap;
equation
  connect(vs.p, r1.p);
  connect(r1.n, c1.p);
  connect(c1.n, gnd.p);
  connect(vs.n, gnd.p);
  v_cap = c1.vC;
end RCFilterExtends;
