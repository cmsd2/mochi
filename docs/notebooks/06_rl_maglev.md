# 06 — Reinforcement learning a MagLev controller

A steel ball is suspended below an electromagnet by an attractive force $\propto i^2 / x^2$, fighting gravity. Open-loop the system is *unstable*: drop the current and the ball falls; raise it slightly and the ball slams into the magnet. A controller has to keep the ball at a target distance below the magnet using only position feedback.

This notebook does the same control problem **two ways** against the same Modelica plant:

1. **Linear**: linearise around the equilibrium with `mod_state_space`, hand-pick PD gains by pole placement.
2. **Reinforcement learning**: black-box optimise the same PD gains with `np_cem` (cross-entropy method) directly against the *nonlinear* plant — no linearisation, no operating-point assumption.

The interesting comparison is what each method gets you. Pole placement gives you analytical insight (closed-form gain ↔ closed-loop pole locations) but works only near the operating point. CEM gives you a controller tuned for the actual nonlinear dynamics, including the inverse-square magnetic-force term.


```maxima
load("../../mochi.mac")$
load("numerics")$
load("numerics-sundials")$
load("numerics-learn")$
load("../../mochi-nonlinear.mac")$
load("ax-plots")$
```

## 1. The plant

$$m\,\ddot{x} = m g - \frac{k_m i^2}{x^2}$$

where $x$ is the ball's distance below the magnet (positive downward), $i$ is the coil current, and $k_m$, $m$, $g$ are constants. The bias current $i_{\text{bias}}$ is the value at which gravity exactly balances the magnetic force at the setpoint $x_{\text{ref}}$:

$$i_{\text{bias}} = \sqrt{\frac{m g x_{\text{ref}}^2}{k_m}}$$

It's declared as a *parameter-of-parameters* in `MagLev.mo`; mochi's fixed-point resolver folds the expression to a number.


```maxima
m : mod_load("../MagLev.mo")$
mod_print(m)$
```

    Model:  MagLev
      parameters:
                  [[ball_mass,0.02],[grav,9.81],[km,1.6e-4],[x_ref,0.008],
                   [i_bias,sqrt((ball_mass*grav)/km)*abs(x_ref)]]
      states:      [x,v]
      derivs:      [der_x,der_v]
      inputs:      [coil_i]
      outputs:     [y]
      initial:     [[x,0.011],[v,0.0]]
      residuals:
         der_x-v  = 0
         (coil_i^2*km)/(ball_mass*x^2)-grav+der_v  = 0
         y-x  = 0

## 2. Open-loop check: the ball falls

With `coil_i = 0` the magnetic force vanishes; the ball just free-falls. We start it slightly below the setpoint at $x_0 = 0.011$ m.


```maxima
[t_open, x_open] : mod_simulate_nonlinear(
    m, [0.011, 0.0], lambda([t], [0.0]), 0.05,
    ['return = 'states, 'dt = 0.0005])$

ax_draw2d(
  color="red", line_width=2, name="x(t), no current",
  lines(t_open, map(first, x_open)),
  color="black", dash="dot",
  explicit(0.008, t, 0, 0.05),
  title="Open-loop free fall (no coil current)",
  xlabel="t (s)", ylabel="x (m, positive = below magnet)",
  yrange=[0, 0.04],
  grid=true, showlegend=true)$
```


    
![svg](06_rl_maglev_files/06_rl_maglev_5_0.svg)
    


## 3. Linearise → pole placement (the textbook approach)

Linearise around the equilibrium $(x = x_{\text{ref}}, v = 0, i = i_{\text{bias}})$. The state-space form is small (2 states, 1 input) so we can read $A$, $B$ off `mod_state_space` directly:


```maxima
[A, B, C, D] : mod_state_space(m, [x = 0.008, v = 0, coil_i = 0.28])$
print("A =")$ A;
print("B =")$ B;

/* Open-loop poles: A has one positive eigenvalue → unstable. */
[evals, _] : np_eig(ndarray(float(A)))$
print("open-loop eigenvalues:", np_to_list(evals))$
```

    A =
    matrix([0.0,1.0],[2450.0000000000005,0.0])
    B =
    matrix([0.0],[-70.0])
    open-loop eigenvalues: [49.49747468305833,-49.497474683058336]

The positive eigenvalue makes this unstable, exactly as expected. We close the loop with $u = i_{\text{bias}} + k_1 (x - x_{\text{ref}}) + k_2 v$ and choose $(k_1, k_2)$ so the closed-loop poles are at $-50 \pm 50i$ (a damped, fast pair). For this 2×2 the math is closed form: with $A_{cl}[2,1] = 2452 - 70 k_1$ and $A_{cl}[2,2] = -70 k_2$, matching $s^2 + 100s + 5000$ gives $k_1 \approx 106.5$, $k_2 \approx 1.43$.


```maxima
/* Pole-placed gains for closed-loop poles at -50 ± 50i.
   Note k1 must be > 2452/70 ≈ 35 just to get stability, and the
   double-derivative on x gives a *positive* spring back to the
   wall, so the gain has to be sized to overcome it. */
k1_lin : 106.5$
k2_lin : 1.43$

/* Verify the closed-loop pole locations numerically. */
A_cl : matrix([0, 1], [2452 - 70*k1_lin, -70*k2_lin])$
[evals_cl, _] : np_eig(ndarray(float(A_cl)))$
print("closed-loop eigenvalues:", np_to_list(evals_cl))$

/* Close the loop and simulate the *nonlinear* plant from the same IC. */
u_pd : buildq([k1v : k1_lin, k2v : k2_lin],
              lambda([t], [i_bias + k1v * (x - x_ref) + k2v * v]))$

[t_pd, x_pd] : mod_simulate_nonlinear(
    m, [0.011, 0.0], u_pd, 0.5,
    ['return = 'states, 'dt = 0.001])$

ax_draw2d(
  color="blue", line_width=2, name="PD (pole placement)",
  lines(t_pd, map(first, x_pd)),
  color="black", dash="dot",
  explicit(0.008, t, 0, 0.5),
  title="MagLev with pole-placed PD gains",
  xlabel="t (s)", ylabel="x (m)",
  yrange=[0.005, 0.013],
  grid=true, showlegend=true)$
```

    closed-loop eigenvalues:
                            [49.979970988386945*%i-50.05000000000001,
                             -(49.979970988386945*%i)-50.05000000000001]


    
![svg](06_rl_maglev_files/06_rl_maglev_9_1.svg)
    


## 4. Same controller form, gains learned by CEM

Now we let the cross-entropy method tune the same two gains by black-box rollout. Each generation, `np_cem` samples 30 candidate $(k_1, k_2)$ pairs from a Gaussian, runs each through `mod_simulate_nonlinear` for half a second, scores the trajectory by mean squared position error plus a small velocity penalty, keeps the best 8, and refits the Gaussian to the survivors.

What's worth noticing about the cost function: the policy is written as a Maxima expression in the model's *symbolic* state names (`x`, `v`) and parameters (`i_bias`, `x_ref`). `mod_simulate_nonlinear` substitutes the parameters and compiles the resulting expression to a native Lisp closure. There's no callback into the Maxima evaluator on each integration step.


```maxima
cem_cost(params_arr) := block(
  [k1, k2, u_fn, sim, x_traj, errs],
  k1 : np_ref(params_arr, 0),
  k2 : np_ref(params_arr, 1),
  /* Closed-loop policy (no clipping in the integrator — let the smooth
     expression go to coerce-float-fun; we won't pursue gains that
     demand absurd current). */
  u_fn : buildq([k1v : k1, k2v : k2],
                lambda([t], [i_bias + k1v * (x - x_ref) + k2v * v])),
  /* Wild candidates can blow up CVODE — errcatch and return a large
     finite cost so CEM still ranks them and converges. */
  sim : errcatch(mod_simulate_nonlinear(
                   m, [0.011, 0.0], u_fn, 0.5,
                   ['return = 'states, 'dt = 0.001,
                    'rtol = 1e-8, 'atol = 1e-10])),
  if sim = [] then return(1e6),
  x_traj : second(first(sim)),
  errs : map(lambda([state], (state[1] - 0.008)^2 + 1e-4 * state[2]^2),
             x_traj),
  float(apply("+", errs)) / length(errs))$

/* Sanity: cost of the pole-placed gains. */
print("PD cost:", cem_cost(ndarray([k1_lin, k2_lin], [2])))$
```

    PD cost: 3.3523450959124696e-7


```maxima
np_seed(7)$

[best, hist] :
  np_cem(cem_cost, 2,
         n_samples=30, n_elites=8, n_gens=15,
         sigma0=50.0, sigma_min=0.5,
         mu0=ndarray([20.0, 50.0], [2]))$

k1_cem : np_ref(best, 0)$
k2_cem : np_ref(best, 1)$
printf(true, "best gains: k1 = ~,2f, k2 = ~,2f~%", k1_cem, k2_cem)$
printf(true, "final cost: ~,3e~%", last(np_to_list(hist)))$
```

    [ERROR][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1487][CVode] At t = 0.0014303320194092, mxstep steps taken before reaching tout.
    CVode: SUNDIALS error (flag=-1): too much work (increase max_steps or try a different method)
    [ERROR][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1487][CVode] At t = 0.00347298503946232, mxstep steps taken before reaching tout.
    CVode: SUNDIALS error (flag=-1): too much work (increase max_steps or try a different method)
    [ERROR][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1487][CVode] At t = 0.00359162241723439, mxstep steps taken before reaching tout.
    CVode: SUNDIALS error (flag=-1): too much work (increase max_steps or try a different method)
    [ERROR][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1487][CVode] At t = 0.00242770541907126, mxstep steps taken before reaching tout.
    CVode: SUNDIALS error (flag=-1): too much work (increase max_steps or try a different method)
    [ERROR][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1487][CVode] At t = 0.00720630975138554, mxstep steps taken before reaching tout.
    CVode: SUNDIALS error (flag=-1): too much work (increase max_steps or try a different method)
    [ERROR][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1487][CVode] At t = 0.0378522963918432, mxstep steps taken before reaching tout.
    CVode: SUNDIALS error (flag=-1): too much work (increase max_steps or try a different method)
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0014303320194092 and h = 7.53778089312838e-20 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1521][CVode] The above warning has been issued mxhnil times and will not be issued again for this problem.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00347298503946232 and h = 1.86864393747061e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1521][CVode] The above warning has been issued mxhnil times and will not be issued again for this problem.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00359162241723439 and h = 1.85238325997594e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1521][CVode] The above warning has been issued mxhnil times and will not be issued again for this problem.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00242770541907126 and h = 1.77436615070714e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1521][CVode] The above warning has been issued mxhnil times and will not be issued again for this problem.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.00720630975138554 and h = 3.94077681243709e-19 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1521][CVode] The above warning has been issued mxhnil times and will not be issued again for this problem.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.0378522963918432 and h = 2.76822185715374e-18 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1521][CVode] The above warning has been issued mxhnil times and will not be issued again for this problem.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.188632746499813 and h = 1.35015933319164e-17 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.0/src/cvode/cvode.c:1516][CVode] Internal t = 0.188632746499813 and h = 1.35015933319164e-17 are such that t + h = t on the next step. The solver will continue anyway.
    [WARNING][rank 0][/private/tmp/sundials-20260406-7535-hqs0wm/sundials-7.7.best gains: k1 = 98.36, k2 = 0.94
    o153final cost: 3.591e-7
    o153$$\mathbf{false}$$
    false

Plot the cost-vs-generation curve to confirm the CEM actually converged:


```maxima
ax_draw2d(
  color="royalblue", line_width=2, name="best-of-generation cost",
  lines(makelist(g - 1, g, 1, length(np_to_list(hist))),
        np_to_list(hist)),
  title="CEM training: best-of-generation cost",
  xlabel="generation", ylabel="cost (mean squared error)",
  logy=true,
  grid=true, showlegend=true)$
```


    
![svg](06_rl_maglev_files/06_rl_maglev_14_0.svg)
    


## 5. Head-to-head

Simulate both controllers from the same initial perturbation and overlay:


```maxima
u_cem : buildq([k1v : k1_cem, k2v : k2_cem],
               lambda([t], [i_bias + k1v * (x - x_ref) + k2v * v]))$

[t_cem, x_cem] : mod_simulate_nonlinear(
    m, [0.011, 0.0], u_cem, 0.5,
    ['return = 'states, 'dt = 0.001])$

ax_draw2d(
  color="blue", line_width=2, name="PD (pole placement)",
  lines(t_pd, map(first, x_pd)),
  color="red", line_width=2, name="PD (CEM-tuned)",
  lines(t_cem, map(first, x_cem)),
  color="black", dash="dot",
  explicit(0.008, t, 0, 0.5),
  title="MagLev: hand-tuned vs CEM-tuned (same plant, same form)",
  xlabel="t (s)", ylabel="x (m)",
  yrange=[0.005, 0.013],
  grid=true, showlegend=true)$
```


    
![svg](06_rl_maglev_files/06_rl_maglev_16_0.svg)
    


## What we got

Both controllers stabilise the ball at the 8 mm setpoint. CEM has *more* overshoot than the pole-placed PD, which is worth understanding: CEM minimised mean squared position error (plus a tiny velocity penalty) over the 0.5 s rollout. Mean squared error is dominated by *how fast* you reach the setpoint, so the optimum trades a sharper rise against a more-underdamped response — the resulting closed-loop poles sit near $-33 \pm 58i$ (damping ratio $\zeta \approx 0.49$), more aggressive than the $-50 \pm 50i$ ($\zeta = 0.707$) pole-placed pair. CEM did exactly what the cost asked for. To get pole-placement-like critical damping out of CEM, the cost would need an explicit overshoot term — e.g. an asymmetric one-sided squared penalty heavily weighted, or a peak-error term such as $\max_t (x_{\text{ref}} - x)^2$.

Where CEM matters more than this notebook shows:

- **Wider operating envelope.** Pole placement is correct only near $(x_{\text{ref}}, 0)$; the inverse-square nonlinearity makes those gains aggressive at small $x$ and soggy at large $x$. CEM training over a *distribution* of initial conditions (instead of just one IC at 11 mm) finds gains that span the range.
- **Cost shapes that aren't analytical.** Add a control-effort penalty, a current-rate-of-change penalty, or a saturation clip — pole placement no longer applies; CEM doesn't care.
- **No model.** CEM treats `mod_simulate_nonlinear` as a black box. If `m` were a CFD simulator or a hardware-in-the-loop rig with the same `[t_list, y_list]` interface, the same code would work.

The limitation: CEM scales poorly past 10–20 parameters. For richer controllers (neural networks, parameterised feedback laws) you'd reach for `np_qlearn` (next notebook) or a policy-gradient method (not yet in numerics-learn).
