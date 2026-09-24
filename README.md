# Beamforming formation tracking with nonlinear quadrotors

Target: **MATLAB R2025a + Simulink**. Three physical generic quadrotors by default,
with a virtual centroid following a smooth figure-eight trajectory. Their
positions rotate a rigid triangular antenna formation toward a fixed elevated
receiver. Each quadrotor has an independent simulated autopilot and full nonlinear
6DOF dynamics, including four motor-speed states.

This is an adaptation of Bharat Manvi, Bharath Bhikkaji and Arun D. Mahindrakar,
*Simultaneous beamforming and trajectory tracking in a multi-agent formation*,
MED 2021, pp. 1114-1119, DOI: `10.1109/MED51440.2021.9480277`.

## 1. Start here

Extract the ZIP, open the extracted `beamforming_quadrotor_R2025a` directory as
MATLAB's Current Folder, and run:

```matlab
startup_project
run_tests(true)       % tests the equations and compiles/runs short Simulink cases
result = run_demo;    % builds the complete .slx, opens it, runs 60 simulated seconds
```

`build_model.m` creates `models/bf_swarm_generated.slx` directly in your R2025a
installation. **A precompiled .slx is not included:** MATLAB/Simulink was not
available in the authoring environment. All model construction code is included;
there is no external model download, manual wiring, or paid add-on beyond your
MATLAB/Simulink installation. The builder explicitly compiles the generated model
before reporting success. Local runtime validation is still required.

The source has been checked with a MATLAB grammar parser. An **independent Python
implementation** of the equations has also been simulated; its evidence is in
`validation/`. Those results are not claimed as MATLAB or Simulink execution.
See `docs/VALIDATION.md` for the exact boundary of verification.

To inspect the model before running:

```matlab
startup_project
cfg = config_default();
[model,cfg] = build_model(cfg);
open_system(model)
```

The generated model contains a formation coordinator, one named subsystem per
drone, continuous nonlinear plants, sampled autopilots, logging, and a live view.
Double-click a drone to inspect its autopilot and plant. The Level-2 MATLAB
S-functions are adapters around readable functions in `core/`; they do not call
an external simulator or emulate a waypoint follower with a first-order lag.

## 2. What you will see

- A ground plane, the fixed receiver at `[4;-3;15]` m, and the prescribed path.
- The actual drone trajectories and bodies, with body attitude visibly changing.
- A translucent actual formation polygon and a dashed desired polygon.
- The actual formation normal in red and the centroid-to-receiver line in green.
- Live normal-pointing, centroid-tracking, and edge-length errors.

The default run starts airborne, with the centroid displaced by 0.15, -0.10,
0.10 m and the formation rotated by 12 degrees. Thus the pointing error should
start nonzero, converge, and remain small during the trajectory. A perfectly
aligned initial condition is available by setting both initial errors to zero.

Closing the live figure stops graphics updates but does not stop the simulation.
Use Simulink Stop to stop the simulation. Display updates run at 20 Hz. Enable
`cfg.pacing=true` to request one simulated second per wall-clock second; a slow
computer can still run slower. The dynamics use a 2 ms fixed step regardless of
graphics speed. Run in **Normal mode**, not Accelerator or Rapid Accelerator.

## 3. Scenario and controller options

```matlab
cfg = config_default();
cfg.mode = 'geometric';          % baseline trajectory tracking
baseline = run_demo(cfg);

cfg.mode = 'paper';              % corrected, extended paper-based feedback
formation = run_demo(cfg);

results = run_comparison();      % both modes, same initial conditions; graphics off

cfg = config_default();
cfg.N = 6;                      % hexagon, no central physical vehicle
cfg.visualize = true;
sixDrones = run_demo(cfg);

cfg = config_default();
cfg.wind.enabled = true;
withWind = run_demo(cfg);

cfg = config_default();
cfg.path.type = 'circle';        % also 'figure8' and 'hover'
cfg.receiver = [5;4;18];
cfg.path.period = 50;
customRun = run_demo(cfg);
```

`config_default.m` is the central configuration. Use column vectors for 3-D
coordinates and vehicle gains. Change `cfg` and rebuild through `run_demo(cfg)`;
editing a base-workspace variable will not change the model's stored configuration.
The builder embeds `cfg` in the **model workspace**, so you can reopen a generated
model after running `startup_project` and press Run. The generated file is rebuilt
on each `run_demo`; save hand-edited models under a different name. Unsaved edits
to an open generated model cause the builder to stop instead of discarding them.

Scaling changes the number of physical vehicles, polygon vertices, state-vector
dimensions, pairwise constraints, and graphics. Gains on the complete distance
graph are normalized by `3/N`. Scaling is not unlimited at a fixed radius:

\[
d_{\min,\mathrm{nominal}}=2r_f\sin(\pi/N).
\]

The configuration rejects a nominal adjacent separation below twice the drone
envelope radius, 0.54 m by default. Increasing N may require a larger formation,
smaller drones, and a different RF wavelength. This is a configuration check and
metric, **not an online collision-avoidance controller**.

## 4. Architecture and signal flow

| Layer | Inputs | Outputs | Execution |
|---|---|---|---|
| Reference generator | Time, receiver, centroid path, polygon offsets | Centroid and per-drone p/v/a; formation rotation and derivatives | Called by coordinator at 100 Hz |
| Formation coordinator | Reference and all measured plant states | One world acceleration request per drone | 100 Hz |
| Drone autopilot | Requested acceleration; measured body quaternion/rates | Four rotor-speed commands; limit diagnostics | 100 Hz |
| Rotor + 6DOF plant | Held rotor commands; wind | 17 physical states per drone | Continuous equations, ode4 at 500 Hz |
| Logger | Actual states and controller diagnostics | Timeseries, CSV, MAT and PNG outputs | 50 Hz |
| Live display | Actual state and analytic reference | 3-D animation and error plots | 20 Hz |

Every drone is independently underactuated. The outer coordinator does not
directly set physical position or acceleration. Motor commands drive rotor
states, rotor thrust and moments drive body dynamics, and body attitude directs
the thrust vector. Flight-controller bandwidth, motor lag, saturation, drag, and
sample-and-hold behavior therefore affect the formation.

The communication system is evaluated from the actual antenna positions. There
is no physical antenna at the virtual centroid. The receiver never moves.

## 5. Why position commands rotate the formation

Let `c_d(t)` be the desired centroid, `p_R` the receiver, and `rho_i` the fixed
formation-frame vertex offsets. They sum to zero. Define

\[
n_d=\frac{p_R-c_d}{\|p_R-c_d\|},\qquad
R_de_3=n_d,\qquad p_{i,d}=c_d+R_d\rho_i.
\]

Changing the three entries of `p_i,d` rotates the triangle in world space.
It does not prescribe any drone's roll/pitch to equal the formation orientation.
Body roll/pitch arise only from the forces required to follow the trajectory.

The references include exact time derivatives:

\[
v_{i,d}=\dot c_d+\dot R_d\rho_i,\qquad
a_{i,d}=\ddot c_d+\ddot R_d\rho_i.
\]

The trajectory starts from rest using a smooth phase-speed ramp. No stop-and-go
waypoint acceptance logic is used. A finite sampled command stream is a numerical
implementation of continuous trajectory tracking, not a sequence of stops.

The orientation gauge uses a fixed world seed projected into the desired plane.
This is smooth for the supplied elevated-receiver scenarios, with analytic first
and second derivatives. It is **not** a minimum-spin/parallel-transport frame.
The reference generator rejects seed/LOS near-parallel configurations instead of
silently switching axes and introducing a discontinuity. See the math document.

## 6. What the two modes do

### Geometric mode

Per-drone reference position, velocity and acceleration are generated directly.
The acceleration request is

\[
a_i^*=a_{i,d}+K_p(p_{i,d}-p_i)+K_d(v_{i,d}-v_i).
\]

This provides an understandable baseline for the full drone plant.

### Paper-based mode

The actual labelled vertex positions determine a least-squares formation rotation
using a proper Kabsch fit. Its angular velocity is obtained by differentiating
the polar factor. Feedback acts on the actual formation orientation, actual
centroid, actual relative velocities and actual pairwise distance errors.

For each edge, `q_ij = ||p_i-p_j||^2 - d_ij^2`. With the consistent rigidity
matrix, `qdot = 2 J v`. The implemented lower law has the paper's backstepping form:

\[
v_f=v_d-k_vJ^Tq,
\]
\[
u=-k_a(v-v_f)-k_qJ^Tq-k_v\dot J^Tq-2k_vJ^TJv+\dot v_d.
\]

For N>3 the effective `kv` and `kq` are the configured gains multiplied by `3/N`.
The desired rotational velocity includes SO(3) logarithm feedback. The centroid
velocity includes absolute-position feedback. An explicit planarity damping term
is added for N>3. `docs/MATHEMATICS.md` derives every term and marks these changes.

In this mode `u` feeds the thrust/attitude autopilot directly as an acceleration
request. An additional independent position PID is **not** placed in series with
the paper controller; that would change the intended outer feedback law.

This is a corrected, extended implementation of the paper's method, not a claim
of literal reproduction of its seven-agent simulation or an unchanged stability
proof for quadrotors. A virtual centroid and a different physical-agent graph
are substantive changes.

## 7. Drone model and autopilot

Each generic 1 kg vehicle uses a 0.225 m arm radius and inertia
`diag([0.015 0.015 0.025])` kg m^2. These are illustrative, physically plausible
simulation parameters; they are not identified from a particular commercial UAV.

The state is

`[position(3); world_velocity(3); quaternion_wxyz(4); body_rates(3); rotor_speeds(4)]`.

The body quaternion maps body FLU coordinates to a world frame with +z up.
The 6DOF Newton-Euler equations are implemented openly in `bf_plantRHS.m` and
integrated by Simulink. This project does not use the reduced-order UAV Guidance
Model. It also does not wrap Aerospace Blockset's 6DOF block: using explicit
equivalent equations keeps one transparent numerical core shared by the Simulink
and MATLAB-only engines. Extra toolboxes are not needed for this implementation.

The autopilot limits requested acceleration and desired tilt, constructs a thrust
direction and body heading, uses geometric attitude PD feedback, allocates total
thrust and body moments to four rotors, and clips infeasible rotor thrusts.
First-order motor dynamics then evolve the physical rotor speeds. Clipping can
alter the achieved wrench; no optimal constrained allocator or integral position
controller is included. Steady wind can therefore leave a small tracking offset.

## 8. Beamforming model and interpretation

- Equal-frequency, coherent narrowband transmitters; phase offsets default to zero.
- Ideal isotropic elements at the physical drone centroids; orientation/polarization
  effects and mutual coupling are omitted.
- Receiver is static, with a line-of-sight channel. No multipath or receiver motion.
- Per-drone transmit power: 0.1 W. Frequency: 138 MHz. Noise power: 1e-13 W.
- Wavelength is computed from `c/f`; at 138 MHz, half-wavelength is about 1.086 m.
  The default circumradius is 0.8 m. The paper's quoted 1.376 m value at that
  frequency is not reused.
- Exact ranges and Friis field amplitudes are used in the received-power sum.
  This is a quasi-static narrowband communication calculation, not a sampled
  138 MHz waveform simulation or a full electromagnetic solver.

For actual ranges `d_i`,

\[
A_i=\sqrt{P_t}\frac{\lambda}{4\pi d_i},\qquad
P_r=\left|\sum_i A_i e^{j\phi_i-j2\pi d_i/\lambda}\right|^2.
\]

The coherent efficiency is `P_r/(sum A_i)^2`, and the reported coherent loss is
`-10 log10(efficiency)`. This separates loss of coherence from path loss. SNR uses
the explicitly configured noise power; it is a modeled value, not measured radio
performance. Set `cfg.rf.phaseOffsets` to a nonzero N-vector to study synchronization
errors. Geometry cannot remove arbitrary independent carrier-phase offsets.

An ideal planar array has opposite broadside maxima. Consequently the metrics
include directed-normal error and an unsigned axis error using the absolute dot
product. For a deformed nonplanar array, the best-fit plane normal is a geometric
pointing proxy, not a claimed numerically optimized true radiation maximum.
Received power still uses every actual 3-D antenna position.

The fixed-plane comparator uses the same actual centroid and a rigid polygon
held at its **initial** formation orientation. It is a communication counterfactual,
not an additional drone flight simulation. The exported array-factor image is a
far-field angular slice at the final instant, normalized to N^2 for equal amplitudes.

## 9. Outputs and metric definitions

Each run writes to `results/<mode>_N<N>/` (repeated runs of that case replace its
previous files):

| File | Contents |
|---|---|
| `simulation.mat` | Full result struct, raw states/commands, metrics, configuration |
| `configuration.json` | All scenario, gain and vehicle parameters used |
| `summary.csv` | Scalar performance metrics |
| `metrics.csv` | Time-indexed errors, separation and RF metrics |
| `performance.png` | Tracking, shape, separation, SNR and body-tilt plots |
| `trajectories.png` | Actual 3-D paths and receiver |
| `array_factor_slice.png` | Final far-field pattern cut |

Centroid RMSE is `sqrt(mean(||mean(P)-c_d||^2))`. Individual position RMSE is
averaged across drones and logged times. Edge errors are physical distance errors
in meters, even though the controller uses squared-distance errors. Settled
pointing RMSE excludes the first 10 s, or the first half of a run shorter than
20 s. Settling time is the first logged time after the last violation of all three
conditions: pointing <2 degrees, centroid error <0.1 m and maximum edge error
<0.05 m. If they never remain satisfied, settling time is NaN.

Limit percentages count drone-time samples. These are sampled diagnostics, not
continuous-time worst-case proofs. Minimum separation is a center-distance metric;
the 0.54 m threshold is a conservative generic body-envelope check. There is no
contact, ground-impact, downwash-interaction or aerodynamic collision model.

Replay a result:

```matlab
replay_result(fullfile(pwd,'results','paper_N3','simulation.mat'),1)
```

The MATLAB-only integrator is an additional diagnostic path:

```matlab
cfg = config_default();
cfg.visualize = false;
r = run_matlab_reference(cfg);
check_engine_agreement       % compares a short Simulink run with the same .m core
```

## 10. Assumptions and limits

Perfect centralized state feedback is the starting point. Communication latency,
state estimation, mocap/GPS noise, packet loss, physical radio synchronization,
takeoff/landing, obstacle avoidance, downwash, ground contact, actuator faults,
battery dynamics and deployment firmware are not modeled. These are explicit
future extensions rather than hidden idealizations.

The desired reference normal points exactly toward the receiver by construction.
The actual formation only tracks that objective, with transient and disturbance
errors. The paper's point-mass local convergence result is not a stability theorem
for this saturated sampled-data quadrotor cascade. No global collision-free or
global SO(3) convergence claim is made. Start with the supplied moderate trajectory
before increasing speed or initial errors.

## 11. Troubleshooting

| Symptom | Action |
|---|---|
| `sf_...` or `bf_...` function not found | Run `startup_project` from the extracted project folder. |
| Builder stops on unsaved changes | Save your edited model under another name, close it, and rebuild. |
| State dimensions changed after changing N | Rebuild using `run_demo(cfg)`; do not edit N during a run. |
| Formation radius rejected | Increase radius or use a smaller vehicle; inspect wavelength implications too. |
| Frame seed nearly parallel to LOS | Choose a different fixed `cfg.frameSeed` and rerun the reference checks. |
| Slow graphics | Set `cfg.visualize=false`; use the exported plots or replay afterward. |
| Large tilt/shape oscillations | Slow the path first; inspect acceleration limits, motor limits and sample time. |
| Ground clearance assertion | Raise the path or reduce the formation radius/tilt; no contact model is supplied. |
| Accelerator/code generation error | Use Normal simulation mode. These MATLAB S-functions have no TLC implementation. |

## 12. Source map

- `config_default.m`: scenario and all tunable parameters.
- `build_model.m`: reproducible Simulink construction and compile gate.
- `run_demo.m`, `run_comparison.m`, `run_matlab_reference.m`: entry points.
- `core/bf_reference.m`: smooth trajectory and formation reference derivatives.
- `core/bf_formationControl.m`: both outer-loop modes.
- `core/bf_formationPose.m`: actual orientation and its exact polar-factor derivative.
- `core/bf_rigidity.m`: squared-distance errors, Jacobian and time derivative.
- `core/bf_autopilot.m`: force/attitude tracking and rotor allocation.
- `core/bf_plantRHS.m`: physical continuous-time drone dynamics.
- `simulink/`: Level-2 block adapters, sampling and state integration.
- `visualization/`: live view, saved plots and replay.
- `tests/`: executable MATLAB equation tests and Simulink smoke/agreement tests.
- `docs/MATHEMATICS.md`: derivation and paper-to-code mapping.
- `docs/VALIDATION.md`: verification status, evidence and local acceptance checks.
- `validation/`: reproducible independent numerical checks and clearly labeled previews.

MathWorks references used for the integration design:

- [Level-2 MATLAB S-functions](https://www.mathworks.com/help/simulink/sfg/writing-level-2-matlab-s-functions.html)
- [S-function sample times](https://www.mathworks.com/help/simulink/sfg/sample-times-matlab.html)
- [Simulation pacing](https://www.mathworks.com/help/simulink/slref/simulationpacingoptions.html)
- [Quaternion 6DOF equations](https://www.mathworks.com/help/aeroblks/6dofquaternion.html)
- [Reduced-order Guidance Model distinction](https://www.mathworks.com/help/uav/ref/guidancemodel.html)

Created 2026-09-24. The generic vehicle parameters and radio model must be replaced
or identified before interpreting results as predictions for a particular aircraft.
