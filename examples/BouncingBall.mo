model BouncingBall
  // Classic hybrid-system test case.  Continuous dynamics are
  // free-fall under gravity with viscous-free flight (no air drag);
  // the discrete event is the ground impact, which discontinuously
  // flips the velocity (multiplied by the coefficient of restitution).
  parameter Real e = 0.8;        // coefficient of restitution
  parameter Real g = 9.81;       // gravitational acceleration
  Real h(start = 10);            // height above ground
  Real v(start = 0);             // upward velocity
equation
  der(h) = v;
  der(v) = -g;
  when h <= 0 and v < 0 then
    reinit(v, -e * pre(v));
  end when;
end BouncingBall;
