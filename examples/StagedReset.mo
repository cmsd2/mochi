model StagedReset
  // A `when' chain with an `elsewhen' branch.  The clock x rises at unit
  // rate; y is set to 1 when x passes 1 and to 2 when it passes 2.  The
  // two crossings never coincide, so branch priority doesn't matter here.
  // Both crossings land on sample times when simulated with dt = 0.1.
  Real x(start = 0);   // clock
  Real y(start = 0);   // stage
equation
  der(x) = 1;
  der(y) = 0;
  when x > 1 then
    reinit(y, 1);
  elsewhen x > 2 then
    reinit(y, 2);
  end when;
end StagedReset;
