model RLCircuit
  // Series RLC driven by an input voltage.
  // States: inductor current iL, capacitor voltage vC.
  // Output: capacitor voltage.
  parameter Real R = 1.0;
  parameter Real L = 0.5;
  parameter Real Ccap = 1.0;
  Real iL(start = 0);
  Real vC(start = 0);
  input Real Vin;
  output Real y;
equation
  L * der(iL) = Vin - R*iL - vC;
  Ccap * der(vC) = iL;
  y = vC;
end RLCircuit;
