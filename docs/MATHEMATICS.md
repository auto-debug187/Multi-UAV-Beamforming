# Mathematical model and paper-to-code mapping

This document uses an internally consistent convention throughout. It explains
the implementation rather than treating every printed expression in the source
paper as executable code. All quantities use SI units. `N` always counts physical
quadrotors, never a virtual centroid.

## 1. Frames, states and notation

`W` is a right-handed world frame with +z up. `B_i` is drone i's right-handed body
frame, x forward, y left, z up. `F` is the geometric formation frame.

- `R_F` maps formation-frame vectors into W.
- `R_i` maps drone-body vectors into W. It is unrelated to `R_F` in general.
- `p_i, v_i, a_i` are world-frame translational quantities.
- `omega_i` is the drone's angular velocity resolved in its own body frame.
- `Omega_F` is formation angular velocity resolved in W.
- `hat(x)y = x cross y`; `vee` is its inverse on skew-symmetric matrices.
- Lowercase `q_ij` denotes a squared-distance error. The plant's quaternion is
  separately denoted `Q_i`, scalar first.

Consequently `R_F_dot = hat(Omega_F) R_F`, whereas drone attitude satisfies
`R_i_dot = R_i hat(omega_i)`. Mixing these angular-velocity frames changes the
cross-product signs and is a frequent implementation error.

## 2. Array geometry and receiver direction

For circumradius r and physical drone count N,

\[
\rho_i=r[\cos\alpha_i,\sin\alpha_i,0]^T,\quad
\alpha_i=2\pi(i-1)/N,\quad \sum_i\rho_i=0.
\]

For three drones, the triangle side length is `sqrt(3)*r`. For general N,
adjacent spacing is `2*r*sin(pi/N)`. Desired pairwise distances are computed
from the offsets rather than assumed equal for all pairs of a larger polygon.

Given desired centroid c and fixed receiver p_R,

\[
\ell=p_R-c,\quad L=\|\ell\|,\quad n=\ell/L.
\]

At broadside, the far-field phase term for every planar offset is identical
because `n^T R_F rho_i = 0`. With common transmitter phases and positive field
amplitudes, this gives a maximum of the magnitude of the coherent sum. Both
`+n` and `-n` are broadside maxima. A virtual central agent is not necessary
for this property. Removing a real central transmitter changes total gain and
the angular pattern and must be reflected in the signal sum.

For an ideal regular polygon pointed at an on-axis finite-distance receiver,
all peripheral transmitters also have exactly equal ranges
`sqrt(L^2+r^2)`. This finite-distance fact is useful for the RF unit test, though
the paper's general beam-pattern argument uses a far-field approximation.

## 3. Smooth path and analytic reference derivatives

The default centroid is

\[
c(t)=[2.5\sin\phi,\;1.5\sin(2\phi),\;3+0.35\sin\phi]^T.
\]

To start from rest, with ramp duration T_r=5 s, frequency w=2pi/40, and u=t/T_r,

\[
\phi=wT_r(2.5u^4-3u^5+u^6),\quad 0\leq t<T_r,
\]
\[
\dot\phi=w(10u^3-15u^4+6u^5),\quad
\ddot\phi=\frac{w}{T_r}(30u^2-60u^3+30u^4).
\]

After the ramp, `phi=w*(t-T_r/2)`, `phidot=w`, `phiddot=0`. Position, velocity
and acceleration match across the join. The path does not stop at the final
simulation time; T is an observation horizon, not a landing command.

For any differentiable nonzero vector x with length L, its normalized direction
has derivatives

\[
n=x/L,\quad \dot L=n^T\dot x,\quad
\dot n=(\dot x-n\dot L)/L,
\]
\[
\ddot n=\frac{(I-nn^T)\ddot x}{L}
-2\frac{\dot L}{L}\dot n-n\|\dot n\|^2.
\]

`bf_unitDerivatives.m` implements this identity. It is used first with
`x=p_R-c`, then again to normalize the projected frame seed.

Choose a fixed unit world vector h and define

\[
s=h-n(n^Th),\quad b_1=s/\|s\|,\quad b_2=n\times b_1,
\quad R_d=[b_1\;b_2\;n].
\]

Its analytic derivatives follow from

\[
\dot s=-\dot n(n^Th)-n(\dot n^Th),
\]
\[
\ddot s=-\ddot n(n^Th)-2\dot n(\dot n^Th)-n(\ddot n^Th),
\]
\[
\dot b_2=\dot n\times b_1+n\times\dot b_1,
\quad
\ddot b_2=\ddot n\times b_1+2\dot n\times\dot b_1+n\times\ddot b_1.
\]

No numerical differentiation is used in the flight reference generator.
The seed method is smooth only away from h parallel to n. `norm(s)>0.05` is
enforced; select another seed or implement a transported frame for paths that
violate this domain. The actual reference spin about n is included in all
derivatives, so this gauge choice does not silently omit rotational velocity.

The angular quantities are

\[
\widehat\Omega_d=\dot R_dR_d^T,\qquad
\widehat{\dot\Omega}_d=\ddot R_dR_d^T+\dot R_d\dot R_d^T.
\]

Numerical round-off is removed by taking the skew-symmetric parts. The resulting
per-drone references are

\[
p_{i,d}=c+R_d\rho_i,\quad v_{i,d}=\dot c+\dot R_d\rho_i,
\quad a_{i,d}=\ddot c+\ddot R_d\rho_i.
\]

Equivalently, with s_i=R_d rho_i,

\[
a_{i,d}=\ddot c+\dot\Omega_d\times s_i+
\Omega_d\times(\Omega_d\times s_i).
\]

The last two terms are angular-acceleration and centripetal contributions.
Ignoring them would reduce tracking accuracy during formation rotation.

## 4. Estimating actual formation pose and angular velocity

Let `pbar=mean(P,2)`, `vbar=mean(V,2)`, `s_i=p_i-pbar`, and
`vr_i=v_i-vbar`. The proper rotation that fits labelled offsets to positions is
obtained from

\[
A=\sum_i s_i\rho_i^T=U\Sigma V^T,\quad
R=U\operatorname{diag}(1,1,\det(UV^T))V^T.
\]

Rank two is expected for a planar formation and is sufficient here; a collapsed
or collinear formation is rejected. Vertex labels resolve the orientation of
the normal and the in-plane orientation.

For deformation, fitting angular velocity to `v=Omega cross s` is not necessarily
the derivative of the best-fit rotation. Instead this implementation
differentiates the polar factor itself. Let

\[
S=R^TA,\quad K=R^T\dot A,\quad \dot A=\sum_i v_{r,i}\rho_i^T.
\]

Since S is symmetric and `K=hat(omega_b)S+Sdot`,

\[
K-K^T=\widehat\omega_b S+S\widehat\omega_b,
\]
\[
\omega_b=(\operatorname{tr}(S)I-S)^{-1}\operatorname{vee}(K-K^T),
\qquad \Omega_a=R\omega_b.
\]

`bf_formationPose.m` implements this expression. A finite-difference test includes
nonrigid position and velocity perturbations, so the test does not merely verify
an ideal rigid special case.

## 5. SO(3) pointing feedback

Use the actual rotation R and desired rotation R_d. Define the world rotation
error

\[
E=RR_d^T,\qquad e_R=\log(E)^\vee.
\]

For a perfectly controllable rigid formation, a consistent spatial-angular-
velocity command is

\[
\Omega_c=\Omega_d-\beta e_R.
\]

The principal logarithm is locally smooth away from rotations of pi. The code
includes numerically stable small-angle and near-pi evaluations, but a global
smooth attitude-feedback guarantee is not asserted.

The actual error derivative is computed from

\[
\dot e_R=J_l^{-1}(e_R)(\Omega_a-E\Omega_d),
\]
\[
J_l^{-1}(e)=I-\tfrac12\widehat e+
\left[\frac{1-(\theta/2)\cot(\theta/2)}{\theta^2}\right]\widehat e^2,
\quad \theta=\|e\|.
\]

The square-bracket factor approaches 1/12 as theta approaches zero. Therefore

\[
\dot\Omega_c=\dot\Omega_d-\beta\dot e_R.
\]

The controller uses actual measured state derivatives to avoid a noisy numerical
difference of the rotation command. This SO(3) law is a consistent implementation
of the paper's orientation-tracking objective; the paper's mixed frame notation
is not copied without reconciling conventions.

## 6. Centroid feedback and distance backstepping

Distance errors cannot observe a common translation. Introduce absolute centroid
feedback explicitly:

\[
v_c=\dot c-k_c(\bar p-c),\qquad
\dot v_c=\ddot c-k_c(\bar v-\dot c).
\]

The desired velocity field for actual vertex offsets is

\[
v_{d,i}=v_c+\Omega_c\times s_i,
\]
\[
\dot v_{d,i}=\dot v_c+\dot\Omega_c\times s_i+
\Omega_c\times(v_i-\bar v).
\]

This field is instantaneous rigid motion, so `J*v_d=0` for any edge set. Indeed,
relative desired velocity is `Omega_c cross (p_i-p_j)` and its dot product with
the edge vector is zero.

For edge (i,j),

\[
q_{ij}=\|p_i-p_j\|^2-d_{ij}^2.
\]

The row of J has `(p_i-p_j)^T` in columns of i, its negative in columns of j,
and zeros elsewhere. Thus J is `M x 3N`, not square in general. `Jdot` uses
relative velocities in those same columns. Direct differentiation gives

\[
\dot q=2Jv,\quad v_f=v_d-k_vJ^Tq,
\]
\[
\dot v_f=\dot v_d-k_v\dot J^Tq-2k_vJ^TJv.
\]

Choose

\[
u=\dot v_f-k_a(v-v_f)-k_qJ^Tq.
\]

Expanding yields the implemented form quoted in the README. Each term has an
explicit role: velocity-error damping, shape restoration, Jacobian time-variation
compensation, distance-error derivative compensation and reference feedforward.

For ideal double integrators, e=v-v_f then satisfies
`edot=-ka*e-kq*J'*q`. The translational/shape candidate

\[
V=\tfrac12e^Te+\tfrac{k_q}{4}q^Tq
\]

has

\[
\dot V=-k_a\|e\|^2-k_qk_v\|J^Tq\|^2,
\]

using `J*v_d=0`. This demonstrates the damping structure; it is **not** a proof
of global shape convergence, pose tracking or quadrotor stability. Redundant
edges make `JJ^T` singular, planar configurations have additional first-order
rigidity issues for N>3, and the actual drone acceleration differs from u.

The complete graph uses all `N(N-1)/2` physical-drone pairs. Effective gains are
`kv=cfg.control.kv*3/N` and `kq=cfg.control.kq*3/N`. This preserves the default
three-drone tuning while moderating the force growth as neighbors are added. It
does not replace gain design for arbitrary N, radii or drone dynamics.

## 7. Explicit N>3 planarity extension

At a planar configuration, squared distances have zero first derivative with
respect to some out-of-plane displacements. Distance feedback alone is therefore
a poor planarity-restoring mechanism near such configurations. For the actual
best-fit normal n=R*e3, define

\[
z_i=n^Ts_i,\qquad
\dot z_i=n^T(v_i-\bar v)+(\Omega_a\times n)^Ts_i.
\]

For N>3 add

\[
u_{\mathrm{plane},i}=-n(k_{\mathrm{plane}}z_i+d_{\mathrm{plane}}\dot z_i).
\]

This term vanishes for an exactly rigid planar formation, and its sum is zero,
so it does not deliberately translate the centroid. It is a practical extension
whose effects are checked numerically; the paper's original proof is not claimed
to cover it. For N=3, every noncollinear triangle is planar and this term is omitted.

## 8. Nonlinear autopilot

The outer loop produces world acceleration a*. Limit its vertical component,
horizontal magnitude and implied tilt before constructing

\[
F_d=m(a^*+g e_3),\quad b_{3,d}=F_d/\|F_d\|.
\]

Given body-heading seed h=[cos(yaw),sin(yaw),0], set
`b2_d=normalize(b3_d cross h)`, `b1_d=b2_d cross b3_d`, and
`R_body,d=[b1_d b2_d b3_d]`. The commanded collective thrust is the nonnegative
projection onto the current thrust axis:

\[
T_c=\max(0,F_d^TR_ie_3).
\]

Geometric attitude PD control uses

\[
e_{R,i}=\tfrac12(R_{d,i}^TR_i-R_i^TR_{d,i})^\vee,
\]
\[
\tau_c=-K_Re_{R,i}-K_\omega\omega_i+\omega_i\times(I\omega_i).
\]

There is no desired angular-rate feedforward in this simple inner loop. It is
adequate for the supplied slow formation motion, and its finite tracking error
is intentionally retained. Fast trajectories may require rate feedforward,
retuning, or a different inner controller.

With four rotor locations `(x_j,y_j,0)` and yaw-torque signs sigma_j,

\[
\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}
=Bf,\quad
B=\begin{bmatrix}1&1&1&1\\y_1&y_2&y_3&y_4\\-x_1&-x_2&-x_3&-x_4\\
\gamma\sigma_1&\gamma\sigma_2&\gamma\sigma_3&\gamma\sigma_4\end{bmatrix},
\quad \gamma=k_Q/k_T.
\]

The allocator computes `f_raw=B\[T_c;tau_c]`, clips each thrust to
`[0,kT*omegaMax^2]`, and commands `sqrt(f/kT)`. Clipping sacrifices exact wrench
tracking and is logged. It is not advertised as an optimal saturation allocator.

## 9. Physical plant

Motor speeds Omega_j obey

\[
\dot\Omega_j=(\Omega_{j,c}-\Omega_j)/\tau_m,\quad f_j=k_T\Omega_j^2.
\]

Translation and rotation are

\[
\dot p=v,\qquad
m\dot v=R_i e_3\sum_jf_j-mg e_3-D(v-v_{\mathrm{air}}),
\]
\[
I\dot\omega_i=\tau-\omega_i\times(I\omega_i)-D_\omega\omega_i.
\]

For scalar-first Q=[q0;qv],

\[
\dot Q=\tfrac12[-q_v^T\omega_i;\;q_0\omega_i+q_v\times\omega_i].
\]

A small norm-stabilization term `(1-Q'*Q)*Q` is added, and the quaternion is
normalized before constructing a rotation matrix. Quaternion norm error is
reported. All gravity and thrust signs are checked by the exact hover test.

The model has 13 rigid-body state coordinates (the unit quaternion contributes
four coordinates for three rotational degrees of freedom) plus four rotor states.
It is a 6DOF rigid-body model, not a 17-DOF mechanical system.

## 10. What is corrected or changed relative to the paper

| Paper element | Implementation treatment |
|---|---|
| Double-integrator agents, Eq. 12 | Full quadrotor plant; double integrator is only the outer-loop design abstraction. |
| Real central leader/transmitter | Virtual trajectory/centroid reference; only physical vertices enter RF sums. |
| Printed edge map/J notation near Eqs. 14-16 | Define squared-distance map explicitly so qdot=2Jv has correct dimensions. |
| Printed `vf=-kv J q+vd` and `J^T=J` in proof | Use `J^T q`; a rectangular rigidity matrix is not symmetric. |
| Frame conventions and desired rotation | Use formation-to-world and body-to-world rotations consistently; derive spatial log feedback. |
| Rotation state treated separately | Estimate actual formation rotation from actual labelled drone positions. |
| Centroid trajectory velocity | Add centroid position feedback so an initial common translation is corrected. |
| Polygon graph | Complete physical-vertex graph with size-normalized gains and explicit N>3 planarity damping. |
| Exponential stability proposition | Do not transfer the theorem to the modified graph and saturated drone cascade. |
| Received-power notation | Use modulus squared of a complex coherent sum with explicit power/amplitude units. |
| Radius 1.376 m quoted at 138 MHz | Recompute lambda/2 from c/f, approximately 1.086 m. |
| Attenuation description | Use field amplitude proportional to 1/d, giving single-source free-space power proportional to 1/d^2. |

The paper motivates the architecture and backstepping structure. This package is
an inspectable engineering adaptation, with its own explicit assumptions and
validation boundaries.
