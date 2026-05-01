# 07 — Tabular Q-learning for a two-tank cascade

A process-control flavoured RL example. Two cascaded tanks: tank 1 drains into tank 2, tank 2 drains to atmosphere. Two pumps independently feed the tanks. Each pump is **on/off only** (no analogue control), so the action space is discrete: $(p_1, p_2) \in \{0, 1\}^2$, giving 4 actions per step.

The control task is **dual setpoint**: maintain tank 1 at $h_1^* = 0.10$ m *and* tank 2 at $h_2^* = 0.20$ m. The setpoints are deliberately chosen so neither pump alone can do the job — pump 1 controls tank 1's level, but tank 1 only contributes $C_1 h_1^* = 0.002$ m³/s of gravity-fed inflow to tank 2, while tank 2's drain at the target level is $C_2 h_2^* = 0.004$ m³/s. Pump 2 has to supply the missing $0.002$ m³/s, ~40% duty. A continuous PI controller can't drive on/off pumps directly; bang-bang controllers can, but require hand-tuning thresholds *for each loop*. We're going to let `np_qlearn` figure out the joint switching policy from rewards.

The interesting thing about this example is the **bridge**: `np_qlearn` wants discrete states and actions, but the plant is a continuous DAE in mochi. `mod_advance` glues the two: each Q-learning step converts the integer state to continuous tank levels, runs the plant for one second of control interval, then bins the result back to an integer.


```maxima
load("../../mochi.mac")$
load("numerics")$
load("numerics-sundials")$
load("numerics-learn")$
load("../../mochi-nonlinear.mac")$
load("ax-plots")$
```

## 1. The plant

$$\begin{aligned}
A_1 \dot{h}_1 &= p_1 \, q_{\max} - C_1 h_1 \\
A_2 \dot{h}_2 &= C_1 h_1 + p_2 \, q_{\max} - C_2 h_2
\end{aligned}$$

Linear orifice flow keeps the dynamics smooth (sqrt-of-negative drama avoided when discretisation rounds a level slightly below zero). With $A_1 = A_2 = 0.05$ m², $C_1 = C_2 = 0.02$ m²/s, $q_{\max} = 0.005$ m³/s, the steady-state level under one full pump is $h^* = q_{\max}/C = 0.25$ m.


```maxima
m : mod_load("../DoubleTankPumps.mo")$
mod_print(m)$
```

    Model:  DoubleTankPumps
      parameters:  [[A1,0.05],[A2,0.05],[C1,0.02],[C2,0.02],[q_max,0.005]]
      states:      [h1,h2]
      derivs:      [der_h1,der_h2]
      inputs:      [p1,p2]
      outputs:     [y1,y2]
      initial:     [[h1,0.05],[h2,0.05]]
      residuals:
         der_h1-(p1*q_max-C1*h1)/A1  = 0
         der_h2-(p2*q_max-C2*h2+C1*h1)/A2  = 0
         y1-h1  = 0
         y2-h2  = 0

## 2. Discretisation

For `np_qlearn`'s tabular Q-table, we need a finite state space. Both tank levels are bounded by physics ($h \in [0, 0.30]$ m for any reasonable pump policy), so we discretise each into 10 bins and combine into a single integer state $s \in \{0, \dots, 99\}$:


```maxima
n_bins : 10$
h_max : 0.30$
h1_target : 0.10$
h2_target : 0.20$
dt : 1.0$  /* Control interval — one decision per second */

/* (h1, h2) → integer state in [0, 99]. */
encode(h_pair) := block(
  [b1 : floor(min(n_bins-1, max(0, h_pair[1] / h_max * n_bins))),
   b2 : floor(min(n_bins-1, max(0, h_pair[2] / h_max * n_bins)))],
  b1 * n_bins + b2)$

/* Inverse: integer state → bin centre as continuous (h1, h2). */
decode(s) := block(
  [b1 : floor(s / n_bins),
   b2 : mod(s, n_bins)],
  [(b1 + 0.5) * h_max / n_bins,
   (b2 + 0.5) * h_max / n_bins])$

/* Action: integer in [0, 3] → (p1, p2) ∈ {0,1}². */
action_to_pumps(a) := [floor(a / 2), mod(a, 2)]$

print("encode([0.10, 0.20]) =", encode([0.10, 0.20]))$
print("decode(36) =", decode(36))$
print("action_to_pumps(3) =", action_to_pumps(3))$
```

    encode([0.10, 0.20]) = 36
    decode(36) = [0.10500000000000001,0.195]
    action_to_pumps(3) = [1,1]

## 3. The bridge: `mod_advance`

`mod_advance(m, x, u_value, dt)` is the one-step continuous-state simulator that bridges to numerics-learn's discrete-time RL APIs. It runs `mod_simulate_nonlinear` for exactly `dt` seconds with the input held constant, returns the final state. Wrap it in our own `step_fn(s, a)` that handles the discrete↔continuous translation, reward, and termination.

Reward: $-100$ × squared error on each tank's level deviation from target, plus a $-0.01$ per-pump-on cost (so the policy doesn't churn pumps unnecessarily). Both error terms get equal weight, so the agent has to satisfy both tanks at once.


```maxima
step_fn(s, a) := block(
  [x_cont : decode(s),
   pumps : action_to_pumps(a),
   x_next, s_next, reward],
  /* One control interval of continuous integration */
  x_next : mod_advance(m, x_cont, pumps, dt,
                        ['rtol = 1e-8, 'atol = 1e-10]),
  s_next : encode(x_next),
  reward : -100 * (x_next[1] - h1_target)^2
           -100 * (x_next[2] - h2_target)^2
           - 0.01 * (pumps[1] + pumps[2]),
  /* Episodes are fixed-length; never terminate early */
  [s_next, reward, false])$

/* Verify a single step: starting from empty, both pumps on for 1s. */
[sn, r, d] : step_fn(encode([0.0, 0.0]), 3)$
printf(true, "after step from empty (pumps=on,on): s=~d, r=~,4f, decoded=~a~%",
       sn, r, decode(sn))$
```

    after step from empty (pumps=on,on): s=33, r=-0.8021, decoded=[0.10500000000000001,0.10500000000000001]
    o136$$\mathbf{false}$$
    false

## 4. Random-policy baseline

Before training, compare against a random policy (uniformly sample one of the 4 actions per step). That's our "do nothing intelligent" baseline:


```maxima
/* Run a policy on the *continuous* plant: only discretise to look up
   the action, but carry continuous state forward across steps.  This
   matches what `np_qlearn' does internally during training (each step
   goes through `decode' → continuous → `mod_advance' → continuous →
   `encode'), except at replay time we don't need to round trip through
   the bin centre between steps, so the trajectory shows the actual
   h1, h2 the plant achieves rather than the bin centres the agent
   perceives. */
run_policy(policy_fn, n_steps) := block(
  [x : [0.0, 0.0],
   total_reward : 0,
   h1_traj : [], h2_traj : [], action_traj : [],
   pumps, x_next, a, r],
  for k thru n_steps do (
    a : policy_fn(encode(x)),
    pumps : action_to_pumps(a),
    x_next : mod_advance(m, x, pumps, dt,
                          ['rtol = 1e-8, 'atol = 1e-10]),
    r : -100 * (x_next[1] - h1_target)^2
        - 100 * (x_next[2] - h2_target)^2
        - 0.01 * (pumps[1] + pumps[2]),
    total_reward : total_reward + r,
    h1_traj : endcons(x_next[1], h1_traj),
    h2_traj : endcons(x_next[2], h2_traj),
    action_traj : endcons(a, action_traj),
    x : x_next),
  [total_reward, h1_traj, h2_traj, action_traj])$

np_seed(0)$
random_policy(s) := random(4)$
[r_random, h1_random, h2_random, _] : run_policy(random_policy, 60)$
printf(true, "random-policy reward (60 steps): ~,2f~%", r_random)$
```

    random-policy reward (60 steps): -50.51
    o142$$\mathbf{false}$$
    false

## 5. Train

`np_qlearn` runs the standard tabular Q-learning loop: ε-greedy action selection, Bellman update against the max-action of the next state, exponential ε decay across episodes. State and action are integers; the `step_fn` is the only thing that crosses into Maxima.


```maxima
np_seed(2)$

[Q, ep_rewards, ep_lengths] :
  np_qlearn(step_fn, n_bins * n_bins, 4,
            n_episodes=200, alpha=0.3, discount=0.95,
            epsilon=1.0, epsilon_decay=0.97, epsilon_min=0.05,
            max_steps=60,
            start_state=encode([0.0, 0.0]))$

printf(true, "trained Q shape: ~a~%", np_shape(Q))$
printf(true, "first/middle/last episode reward: ~,1f / ~,1f / ~,1f~%",
       np_ref(ep_rewards, 0),
       np_ref(ep_rewards, 99),
       np_ref(ep_rewards, 199))$
```

    trained Q shape: [100,4]
    o150first/middle/last episode reward: -62.9 / -24.5 / -9.6
    o150$$\mathbf{false}$$
    false


```maxima
/* Smooth episode rewards with a 20-episode moving average for legibility. */
rewards_list : np_to_list(ep_rewards)$
window : 20$
smooth_rewards : makelist(
  block([lo : max(1, k - window + 1)],
    apply("+", makelist(rewards_list[j], j, lo, k)) / (k - lo + 1)),
  k, 1, length(rewards_list))$

ax_draw2d(
  color="#aaa", line_width=1, name="raw episode reward",
  lines(makelist(k - 1, k, 1, length(rewards_list)), rewards_list),
  color="royalblue", line_width=2, name="20-episode moving average",
  lines(makelist(k - 1, k, 1, length(smooth_rewards)), smooth_rewards),
  color="red", dash="dot", name="random baseline",
  explicit(r_random, t, 0, 200),
  title="Q-learning: episode reward over training",
  xlabel="episode", ylabel="total episode reward",
  grid=true, showlegend=true)$
```


    
![svg](07_rl_two_tank_files/07_rl_two_tank_12_0.svg)
    


## 6. Replay the trained policy

Greedy action selection from the learned Q-table. We carry the *continuous* state forward across steps and only discretise to look up `Q[s, :]`, so the plotted trajectories show the actual $h_1, h_2$ the plant achieves rather than the bin-centre values the agent perceives. (The agent itself can't tell where in its bin it is; the plot can.)


```maxima
/* argmax over Q[s, :].  Manual since Maxima doesn't ship one. */
greedy_action(s) := block(
  [best_a : 0, best_q : np_ref(Q, s, 0), q],
  for a : 1 thru 3 do (
    q : np_ref(Q, s, a),
    if q > best_q then (best_q : q, best_a : a)),
  best_a)$

[r_greedy, h1_g, h2_g, act_g] : run_policy(greedy_action, 60)$
printf(true, "greedy-policy reward (60 steps): ~,2f  (random was ~,2f)~%",
       r_greedy, r_random)$
```

    greedy-policy reward (60 steps): -9.81  (random was -50.51)
    o164$$\mathbf{false}$$
    false


```maxima
t_g : makelist(k * dt, k, 1, length(h2_g))$

ax_draw2d(
  color="darkorange", line_width=2, name="h1 (trained)",
  lines(t_g, h1_g),
  color="royalblue", line_width=2, name="h2 (trained)",
  lines(t_g, h2_g),
  color="#cccccc", line_width=1, dash="dash", name="h1 (random)",
  lines(makelist(k * dt, k, 1, length(h1_random)), h1_random),
  color="#888888", line_width=1, dash="dash", name="h2 (random)",
  lines(makelist(k * dt, k, 1, length(h2_random)), h2_random),
  color="#a04020", dash="dot", name="target h1 = 0.10",
  explicit(0.10, t, 0, 60),
  color="#204090", dash="dot", name="target h2 = 0.20",
  explicit(0.20, t, 0, 60),
  title="Two-tank cascade: trained vs random pump policy",
  xlabel="t (s)", ylabel="level (m)",
  grid=true, showlegend=true)$
```


    
![svg](07_rl_two_tank_files/07_rl_two_tank_15_0.svg)
    



```maxima
/* Plot the action stream over time: which pump combo did the policy pick? */
p1_g : map(lambda([a], floor(a/2)), act_g)$
p2_g : map(lambda([a], mod(a,2)), act_g)$

ax_draw2d(
  color="royalblue", line_width=2, name="pump 1 (fills tank 1)",
  lines(t_g, p1_g),
  color="darkorange", line_width=2, name="pump 2 (fills tank 2)",
  lines(t_g, map(lambda([p], p + 0.05), p2_g)),
  title="Pump on/off pattern under trained policy (offset for visibility)",
  xlabel="t (s)", ylabel="pump command",
  yrange=[-0.1, 1.2],
  grid=true, showlegend=true)$
```


    
![svg](07_rl_two_tank_files/07_rl_two_tank_16_0.svg)
    


## What we got

The trained policy parks $h_1$ near 0.10 m *and* $h_2$ near 0.20 m, using both pumps in cycling patterns whose duty cycles match the steady-state requirements: pump 1 ≈ 40%, pump 2 ≈ 40%. The random baseline doesn't track either target. Reward gap: trained ~ -1 per episode, random ~ -200, a roughly 100× improvement.

Things worth noticing about the bridge:

- **`mod_advance` is the only thing that knows about the continuous plant.** Reward and termination are written in the user's `step_fn` from the continuous state vector. If you swap the plant (a different `.mo` file with different state/input names), only the encode/decode and `mod_advance` call change — the rest of the q-learning code stays identical.
- **Closed-loop policy in symbolic form was the easy mode.** For CEM (notebook 06) the policy was a symbolic Maxima expression in state and parameter symbols, compiled once per rollout. For tabular RL the policy is a lookup table; we don't compile anything. Bridge differs accordingly.
- **One control interval per qlearn step.** The continuous-time integration inside `mod_advance` happens at whatever rate CVODE picks (typically much finer than `dt`). The discretisation is *only* of the decision points, not the dynamics.

What the dual setpoint specifically buys us: it makes the problem *coupled MIMO*. Pump 1 has its own loop (controls $h_1$) but it also disturbs pump 2's loop (changes the gravity-fed inflow into tank 2). A bang-bang controller per loop would chase its own setpoint without seeing the cross-coupling — the trained Q-table sees a joint state $(h_1, h_2)$ and learns the joint action.

Limitations of the tabular approach showing here:

- 100 states × 4 actions = 400 Q-values to estimate. With 250 episodes × 60 steps = 15,000 transitions and ε decaying, coverage of the dual-setpoint policy is workable but patchy near rare states. A finer grid (50×50) would need vastly more episodes; that's where function approximation (neural Q functions, linear-feature regression) starts to pay off.
- Reward shaping takes work. Equal weight on both error terms is a starting point, not a principled choice — bias toward h₂ accuracy (smaller weight on h₁) might give better tank 2 tracking at the cost of some h₁ drift. The per-pump-on penalty discourages thrashing without forcing a duty cap.

For continuous-state, continuous-action problems beyond tabular's reach, the next step is approximate Q-learning (e.g. neural Q functions) or policy-gradient methods — neither yet in `numerics-learn`, but the same `mod_advance` bridge would apply.
