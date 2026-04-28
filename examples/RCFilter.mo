// Component-based RC low-pass filter, wired via Modelica connect().
// Demonstrates real connector-based composition: a connector class
// (ElectricalPin) defines the shared interface, components expose ports,
// and connect() statements wire them together.  rumoca expands each
// connect() into:
//   - flow vars sum to zero at the connection point  (Kirchhoff's current law)
//   - potential vars equate across the connection    (Kirchhoff's voltage law)

connector ElectricalPin
  Real v;          // potential variable: voltage at the pin
  flow Real i;     // flow variable: current INTO the pin
end ElectricalPin;

model Resistor
  parameter Real R = 1.0;
  ElectricalPin p, n;
equation
  p.i + n.i = 0;
  p.v - n.v = R * p.i;
end Resistor;

model Capacitor
  parameter Real C = 1.0;
  Real v(start = 0)        "capacitor voltage = state variable";
  ElectricalPin p, n;
equation
  v = p.v - n.v;
  p.i + n.i = 0;
  C * der(v) = p.i;
end Capacitor;

model Ground
  ElectricalPin p;
equation
  p.v = 0;
end Ground;

model VoltageSource
  ElectricalPin p, n;
  input Real Vin           "driving voltage";
equation
  p.v - n.v = Vin;
  p.i + n.i = 0;
end VoltageSource;

model RCFilter
  // First-order RC low-pass: input Vin drives a series R into a parallel C
  // to ground.  v_cap is the capacitor voltage; the transfer function
  // from Vin → v_cap is 1/(RCs + 1).
  Resistor r1(R = 1.0);
  Capacitor c1(C = 0.5);
  VoltageSource vs;
  Ground gnd;
  output Real v_cap;
equation
  connect(vs.p, r1.p);
  connect(r1.n, c1.p);
  connect(c1.n, gnd.p);
  connect(vs.n, gnd.p);
  v_cap = c1.v;
end RCFilter;
