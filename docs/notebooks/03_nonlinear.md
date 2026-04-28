# 03 — Nonlinear simulation

The notebooks `01_linear_analysis.macnb` and `02_multi_component.macnb` worked entirely with the **linearised** state-space form: pick an operating point, get $A, B, C, D$, and propagate $\dot x = Ax + Bu$. That's fast and analytic, but the answer is only valid for small perturbations around the operating point.

For trajectories that visit large angles, big swings, or any regime where the original nonlinear ODE/DAE doesn't behave like its tangent — we need to integrate the actual nonlinear system. That's what the **`mochi-nonlinear`** subsystem is for: it builds a numeric RHS straight from the (causalised) Modelica residuals and feeds it to SUNDIALS CVODE.

This notebook walks through:

1. Loading the Pendulum model (genuinely nonlinear — gravity term is $-\frac{g}{L}\sin\theta$).
2. **Linear vs nonlinear step response** — they agree near $\theta = 0$ and visibly diverge once we leave the small-angle regime.
3. **Free oscillation from a large initial angle** — period elongation is a classic nonlinear effect.
4. **Arbitrary input** — sinusoidal forcing through the same API.
5. Pointers on stiff systems and tolerances.


```
/* Setup: core mochi + the optional nonlinear subsystem.
   numerics-sundials is a hard dep here — it provides np_cvode. */
load("../../mochi.mac")$
load("numerics")$
load("numerics-sundials")$
load("../../mochi-nonlinear.mac")$
load("ax-plots")$
```

## 1. The pendulum

The model lives at `examples/Pendulum.mo`:

```modelica
model Pendulum
  parameter Real g = 9.81;
  parameter Real L = 1.0;
  parameter Real b = 0.1;
  Real theta(start = 0);
  Real omega(start = 0);
  input Real tau;
  output Real y;
equation
  der(theta) = omega;
  der(omega) = -(g / L) * sin(theta) - b * omega + tau;
  y = theta;
end Pendulum;
```

Two states (angle $\theta$, angular velocity $\omega$), one input (applied torque $\tau$), one output ($\theta$). The nonlinearity is in $\sin\theta$ — the small-angle linearisation replaces it with $\theta$.


```
m : mod_load("../Pendulum.mo")$
mod_print(m)$
```

    Model:  Pendulum
      parameters:  [[g,9.81],[L,1.0],[b,0.1]]
      states:      [theta,omega]
      derivs:      [der_theta,der_omega]
      inputs:      [tau]
      outputs:     [y]
      initial:     [[theta,0],[omega,0]]
      residuals:
         der_theta-omega  = 0
         (g*sin(theta))/L-tau+b*omega+der_omega  = 0
         y-theta  = 0

Notice the residual `der_omega + g·sin(theta)/L + b·omega - tau = 0` — `mod__causalise` (used internally by `mod_simulate_nonlinear`) solves it explicitly for `der_omega`, leaving the `sin(theta)` intact. The linearisation step in `mod_state_space` would then take a Jacobian and replace `sin(theta)` with its derivative at the operating point ($\cos 0 = 1$), but `mod_simulate_nonlinear` keeps the original.

## 2. Linear vs nonlinear step response

Apply a torque step of $\tau = 5$ N·m for 6 seconds, starting from rest. The **linear** simulation thinks gravity is $-\frac{g}{L}\theta$ (a Hookean spring); the **nonlinear** one keeps the actual $-\frac{g}{L}\sin\theta$.


```
[t_lin, y_lin] : mod_step(m, [theta=0, omega=0, tau=0], 6.0,
                           ['magnitude = 5])$
[t_nl,  y_nl]  : mod_step_nonlinear(m, [0.0, 0.0], 6.0,
                                    ['magnitude = 5])$

ax_draw2d(
  color="blue",  line_width=2, name="linear (small-angle)",
  lines(t_lin, y_lin),
  color="red",   line_width=2, name="nonlinear (true)",
  lines(t_nl, y_nl),
  title="Pendulum step response: tau = 5 N·m for 6 s",
  xlabel="t (s)", ylabel="theta (rad)",
  grid=true, showlegend=true
)$
```



For the first ~0.5 s the angle stays small ($\sin\theta \approx \theta$ holds) and the two curves overlap. After that the nonlinear curve climbs further (the linear "spring" pulls harder than the real gravity does once $\theta$ grows past $\sim\pi/4$) and the periods diverge. Past $t \approx 2$ s the trajectories are qualitatively different.

## 3. Free oscillation from a large initial angle

Release the pendulum from $\theta_0 = 2.5$ rad ($\approx 143°$) with no torque. Linearised theory predicts a sinusoid with period $T = 2\pi\sqrt{L/g} \approx 2.006$ s; the true large-amplitude pendulum has a longer period (you can derive it as a complete elliptic integral, but here we just integrate it).


```
[t_free, y_free] : mod_simulate_nonlinear(m, [theta=2.5, omega=0.0],
                                           lambda([t], [0.0]),
                                           10.0)$

ax_draw2d(
  color="red", line_width=2, name="nonlinear (theta_0 = 2.5 rad)",
  lines(t_free, y_free),
  color="black", dash="dot", explicit(0, t, 0, 10),
  title="Pendulum free oscillation, no torque, no damping shift",
  xlabel="t (s)", ylabel="theta (rad)",
  grid=true, showlegend=true
)$
```



The trajectory is asymmetric and the period is noticeably longer than the small-angle value — both classic large-amplitude pendulum effects. The amplitude decays because of the $-b\omega$ damping term in the model.

## 4. Arbitrary input via lambda

`mod_simulate_nonlinear` takes any input function `lambda([t], [u1, u2, ...])`. Here's a sinusoidal torque (with the pendulum starting from rest):


```
[t_sin, y_sin] : mod_simulate_nonlinear(m, [0.0, 0.0],
                                         lambda([t], [3*sin(2*t)]),
                                         8.0)$

ax_draw2d(
  color="red", line_width=2, name="theta(t)",
  lines(t_sin, y_sin),
  color="gray", dash="dash", name="3 sin(2t) (input)",
  explicit(3*sin(2*t), t, 0, 8),
  title="Pendulum forced by tau(t) = 3 sin(2t)",
  xlabel="t (s)", ylabel="theta (rad)  /  tau (N·m)",
  grid=true, showlegend=true
)$
```



Under a periodic forcing the response is also periodic but distorted — you can see the asymmetry from the `sin(theta)` term modulating the swing.

## 5. Stiff systems and tolerances

`mod_simulate_nonlinear` accepts the same options as the underlying `np_cvode` for stiffness and tolerances:

- `'method = 'adams` (default) — variable-order non-stiff solver, good for most plants.
- `'method = 'bdf` — BDF solver for stiff systems (e.g. very disparate eigenvalues, like the DCMotor's electrical/mechanical pole split — though we're still in the linear regime there, so the linear `mod_step` happens to handle it fine via ZOH discretisation).
- `'rtol`, `'atol` — relative and absolute tolerances (both default to $10^{-8}$). Tighten them for tighter accuracy at the cost of more steps.

There's also `'return = 'states` if you want the full state trajectory instead of just the outputs:


```
[t, x_traj] : mod_simulate_nonlinear(m, [theta=2.5, omega=0.0],
                                       lambda([t], [0.0]),
                                       4.0,
                                       ['return = 'states])$

/* x_traj is a list of [theta, omega] pairs, one per sample. */
theta_t : map(first,  x_traj)$
omega_t : map(second, x_traj)$

ax_draw2d(
  color="red", line_width=2, name="phase trajectory",
  lines(theta_t, omega_t),
  marker_size=10, color="green", name="start",
  points([first(theta_t)], [first(omega_t)]),
  marker_size=10, color="black", name="end",
  points([last(theta_t)],  [last(omega_t)]),
  title="Pendulum phase portrait (theta vs omega), 4 s damped",
  xlabel="theta (rad)", ylabel="omega (rad/s)",
  grid=true, showlegend=true, aspect_ratio=true
)$
```



The phase portrait shows the trajectory spiralling toward the origin (the stable equilibrium at $\theta = 0$) under viscous damping — a clear two-state picture you can't get from the output trajectory alone.

## What we got

`mochi-nonlinear` lets us go from a hand-written `.mo` file straight to a true nonlinear time-domain trajectory:

- `mod_load` parses the Modelica source.
- `mod__causalise` (in mochi core) symbolically solves the residual equations for the derivatives.
- `mod_simulate_nonlinear` substitutes parameters, builds an expression-mode RHS in `[t, x_1, x_2, ...]`, and hands it to SUNDIALS CVODE.
- The result has the same `[t_list, y_list]` shape as the linear `mod_simulate`, so it drops into the same plotting / analysis flow.

The linear and nonlinear paths complement each other: linearise for control design (LQR, Bode, root locus), nonlinearly simulate for closed-loop validation.
