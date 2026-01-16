Great—let’s start with **consistent state‑space formulations** for the models you asked about, focusing on **low‑speed (slowly driving) regimes** where non‑holonomic, no‑slip assumptions are appropriate. I’ll use a unified notation across models and point out where extensions (e.g., tire forces) enter. Citations are provided at the end of each subsection to anchor assumptions and standard forms.

### Global notation (used consistently)

*   Global pose of a reference point on the vehicle (usually rear axle mid‑point of the tractor or single vehicle): $$(x, y, \psi)$$ with yaw $$\psi$$.
*   Speed $$v$$ (along the body $$x$$-axis), wheelbase $$L$$, front/rear axle CG offsets $$l_f, l_r$$ (with $$L = l_f + l_r$$).
*   Steering angle(s) $$\delta$$. For dual/4WS: $$\delta_f$$ (front), $$\delta_r$$ (rear). Steering‑rate limits apply but are not shown unless needed.
*   States are stacked as $$ \mathbf{x} = [\ldots]^\top$$. Inputs are $$ \mathbf{u} = [\ldots]^\top$$. Parameters are collected in $$\theta$$.
*   **Low‑speed assumption**: small slip angles, no lateral tire slip in kinematic models; dynamic models include yaw inertia and linear tire forces where noted. This separation and its validity range are standard in the literature. [\[researchgate.net\]](https://www.researchgate.net/profile/Philip-Polack/publication/318810853_The_kinematic_bicycle_model_A_consistent_model_for_planning_feasible_trajectories_for_autonomous_vehicles/links/5addcbc2a6fdcc29358b9c01/The-kinematic-bicycle-model-A-consistent-model-for-planning-feasible-trajectories-for-autonomous-vehicles.pdf), [\[thomasferm....github.io\]](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)

***

## 1) **Kinematic Bicycle (Single‑Track) Model**

**State**

$$
\mathbf{x} = 
\begin{bmatrix}
x \\ y \\ \psi
\end{bmatrix}
\qquad
\text{Input }
\mathbf{u} =
\begin{bmatrix}
v \\ \delta
\end{bmatrix}
\qquad
\text{Params } \theta = \{L\}
$$

**Dynamics** (rear‑axle reference):

$$
\dot{x} = v \cos\psi,\quad
\dot{y} = v \sin\psi,\quad
\dot{\psi} = \frac{v}{L}\tan\delta.
$$

*   Valid when **lateral slip angles are negligible**; curvature $$\kappa \approx \tan\delta/L$$.
*   Often used for **trajectory planning/low‑speed control**; see consistency bounds vs. a 9‑DoF reference in Polack et al. and geometric derivation via ICR in Fermi’s notes. [\[researchgate.net\]](https://www.researchgate.net/profile/Philip-Polack/publication/318810853_The_kinematic_bicycle_model_A_consistent_model_for_planning_feasible_trajectories_for_autonomous_vehicles/links/5addcbc2a6fdcc29358b9c01/The-kinematic-bicycle-model-A-consistent-model-for-planning-feasible-trajectories-for-autonomous-vehicles.pdf), [\[thomasferm....github.io\]](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)

***

## 2) **Dynamic Bicycle (Yaw Dynamics with Tire Forces)**

To capture **transient yaw** and light slip at modest speeds, augment with lateral velocity $$v_y$$ and yaw rate $$r$$.

**State**

$$
\mathbf{x} =
\begin{bmatrix}
x \\ y \\ \psi \\ v_x \\ v_y \\ r
\end{bmatrix}
\qquad
\text{Input } \mathbf{u} =
\begin{bmatrix}
F_{x,f} \\ F_{x,r} \\ \delta
\end{bmatrix}
\qquad
\text{Params } \theta = \{m, I_z, l_f, l_r, C_f, C_r\}
$$

*   $$v_x$$ is longitudinal speed (you can hold $$v_x$$ constant for a simpler lateral model).
*   Linear tire model at low slip: $$F_{y,f} = -C_f \alpha_f,\; F_{y,r} = -C_r \alpha_r$$ with slip angles

$$
\alpha_f = \arctan\!\left(\frac{v_y + l_f r}{v_x}\right) - \delta,\quad
\alpha_r = \arctan\!\left(\frac{v_y - l_r r}{v_x}\right).
$$

**Body‑frame dynamics**

$$
\begin{aligned}
\dot{v}_x &= \frac{1}{m}\!\left(F_{x,f}\cos\delta - F_{y,f}\sin\delta + F_{x,r}\right) + v_y r,\\
\dot{v}_y &= \frac{1}{m}\!\left(F_{x,f}\sin\delta + F_{y,f}\cos\delta + F_{y,r}\right) - v_x r,\\
\dot{r}   &= \frac{1}{I_z}\!\left(l_f(F_{y,f}\cos\delta + F_{x,f}\sin\delta) - l_r F_{y,r}\right).
\end{aligned}
$$

**Kinematics to world**

$$
\dot{x} = v_x\cos\psi - v_y\sin\psi,\quad
\dot{y} = v_x\sin\psi + v_y\cos\psi,\quad
\dot{\psi} = r.
$$

*   For **very low speeds**, many works set $$v_x \approx \text{const}$$ and use the **2‑DoF lateral + yaw** linearized bicycle; this is standard in vehicle dynamics texts and university notes. [\[vtechworks...lib.vt.edu\]](https://vtechworks.lib.vt.edu/bitstream/handle/10919/36615/Chapter2a.pdf), [\[code.eng.buffalo.edu\]](https://code.eng.buffalo.edu/dat/sites/model/bicycle.html)

***

## 3) **Dual‑Steering (Four‑Wheel Steering) – Kinematic**

At low speed, a 4WS vehicle is still modeled kinematically but with **front and rear steering**.

**State**

$$
\mathbf{x} =
\begin{bmatrix}
x \\ y \\ \psi
\end{bmatrix}
\qquad
\text{Input } \mathbf{u} =
\begin{bmatrix}
v \\ \delta_f \\ \delta_r
\end{bmatrix}
\qquad
\text{Params } \theta = \{l_f, l_r\}
$$

A common single‑track 4WS relation uses the **equivalent curvature**

$$
\kappa = \frac{\tan\delta_f - \tan\delta_r}{L}
\quad\Rightarrow\quad
\dot{\psi} = v\,\kappa = \frac{v}{L}\,(\tan\delta_f - \tan\delta_r).
$$

Kinematics:

$$
\dot{x} = v\cos\psi,\quad \dot{y} = v\sin\psi.
$$

*   **Counter‑phase** ($$\delta_r \approx -\gamma\,\delta_f$$, $$0<\gamma\le 1$$) reduces turning radius; **in‑phase** enables crab motion.
*   For linkage‑level geometry (Ackermann vs parallel, two steered axles), see MathWorks’ block equations and research on multi‑axle steering linkages. [\[mathworks.com\]](https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html), [\[researchgate.net\]](https://www.researchgate.net/profile/Volodymyr-Kukhar-2/publication/327714832_Simulation_Technique_of_Kinematic_Processes_in_the_Vehicle_Steering_Linkage/links/5ba04d5945851574f7d26214/Simulation-Technique-of-Kinematic-Processes-in-the-Vehicle-Steering-Linkage.pdf)

> *Remark:* In industrial intralogistics, **double‑Ackermann** arrangements appear both on tractors and trailers; the above single‑track representation remains valid for **low‑speed path planning**. [\[KINEMATIC...ING SYSTEM\]](http://www.ijsimm.com/Full_Papers/Fulltext2021/text20-2_550.pdf)

***

## 4) **Truck–Trailer (Single Trailer), Kinematic with Off‑Axle Hitch**

Let the tractor pose be $$(x, y, \psi_0)$$, trailer yaw $$\psi_1$$, and articulation angle $$\beta_1 = \psi_1 - \psi_0$$. Hitch offset $$a$$ is the longitudinal distance from the tractor rear axle to the hitch (off‑axle hitching). Tractor wheelbase $$L_0$$; trailer axle to hitch length $$L_1$$.

**State**

$$
\mathbf{x} =
\begin{bmatrix}
x \\ y \\ \psi_0 \\ \beta_1
\end{bmatrix}
\qquad
\text{Input } \mathbf{u} =
\begin{bmatrix}
v_0 \\ \delta_0
\end{bmatrix}
\qquad
\text{Params } \theta = \{L_0, L_1, a\}
$$

**Tractor kinematics**

$$
\dot{x} = v_0\cos\psi_0,\quad
\dot{y} = v_0\sin\psi_0,\quad
\dot{\psi}_0 = \frac{v_0}{L_0}\tan\delta_0.
$$

**Articulation (off‑axle hitch)**

$$
\dot{\beta}_1
= -\frac{v_0}{L_0}\tan\delta_0
+ \frac{v_0}{L_1}\,\sin\beta_1
+ \frac{a\,v_0}{L_0 L_1}\,\tan\delta_0 \cos\beta_1.
$$

*   If **on‑axle** hitch ($$a=0$$), the last term vanishes and the familiar single‑trailer kinematic relation remains.
*   This model is standard in low‑speed **backing and docking** control and in active trailer steering works; the structure appears (with equivalent forms) in van de Wouw et al. [\[vandewouw.dc.tue.nl\]](https://vandewouw.dc.tue.nl/CDC2015_vandeWouw_Ritzen.pdf)

***

## 5) **Tractor with Multiple Trailers (Chain), Kinematic**

Extend to $$n$$ trailers with yaw $$\psi_i$$ and articulation $$\beta_i = \psi_i - \psi_{i-1}$$. Each segment $$i$$ has length $$L_i$$; allow off‑axle hitching $$a_i$$ where needed.

**State**

$$
\mathbf{x} =
\begin{bmatrix}
x \\ y \\ \psi_0 \\ \beta_1 \\ \beta_2 \\ \vdots \\ \beta_n
\end{bmatrix},
\qquad
\mathbf{u} =
\begin{bmatrix}
v_0 \\ \delta_0
\end{bmatrix}
\qquad
\theta = \{L_0,\ldots,L_n;\; a_1,\ldots,a_n\}
$$

**Tractor**

$$
\dot{x} = v_0\cos\psi_0,\quad
\dot{y} = v_0\sin\psi_0,\quad
\dot{\psi}_0 = \frac{v_0}{L_0}\tan\delta_0.
$$

**Articulation recursion (on‑axle baseline)**

$$
\dot{\beta}_i
= -\dot{\psi}_{i-1} + \frac{v_{i-1}}{L_i}\sin\beta_i,
\quad i=1,\ldots,n,
$$

with $$v_{i-1}$$ the speed at joint $$i-1$$. For **off‑axle** hitches, add the standard $$\propto a_{i}\dot{\psi}_{i-1}\cos\beta_i$$ term (pattern as in the single‑trailer case). This class of models is the workhorse in non‑holonomic planning and control for articulated systems. [\[vandewouw.dc.tue.nl\]](https://vandewouw.dc.tue.nl/CDC2015_vandeWouw_Ritzen.pdf), [\[academia.edu\]](https://www.academia.edu/83346185/Dynamic_model_of_a_two_trailer_articulated_vehicle_subject_to_nonholonomic_constraints)

***

## 6) **Dynamic Articulated Model (Tractor + Two Trailers)**

For slow but **dynamic** maneuvers where **inertia and yaw coupling** matter (e.g., oscillations), use Lagrangian dynamics with non‑holonomic constraints (no lateral slip at the wheels). State includes yaw rates and articulation rates.

**State** (one example)

$$
\mathbf{x} =
\begin{bmatrix}
x \\ y \\ \psi_0 \\ \psi_1 \\ \psi_2 \\ \dot{\psi}_0 \\ \dot{\psi}_1 \\ \dot{\psi}_2
\end{bmatrix},
\quad
\mathbf{u} =
\begin{bmatrix}
F_{x,0} \\ \delta_0
\end{bmatrix},
\quad
\theta = \{m_i, I_{z,i}, L_i, \text{hitch geometry}\}.
$$

**Structure**

$$
\mathbf{M}(\mathbf{q})\,\dot{\mathbf{v}} + \mathbf{C}(\mathbf{q},\mathbf{v})\,\mathbf{v} = \mathbf{B}(\mathbf{q})\,\mathbf{u},
\quad
\mathbf{A}(\mathbf{q})\,\mathbf{v} = \mathbf{0},
$$

where $$\mathbf{q} = [x,y,\psi_0,\psi_1,\psi_2]^\top$$, $$\mathbf{v} = \dot{\mathbf{q}}$$. The **Pfaffian constraints** $$\mathbf{A}(\mathbf{q})\mathbf{v}=0$$ encode rolling‑without‑slipping for each axle, and reduction yields explicit ODEs for $$\dot{\psi}_i,\ddot{\psi}_i$$.  
Full closed‑form expressions are lengthy; the derivation template and special cases are provided in De Santis et al. (Robotica) and related dynamic truck‑trailer studies. [\[academia.edu\]](https://www.academia.edu/83346185/Dynamic_model_of_a_two_trailer_articulated_vehicle_subject_to_nonholonomic_constraints), [\[link.springer.com\]](https://link.springer.com/article/10.1007/s11071-019-05452-1)

***

## 7) **Dual‑Steering in Tractor–Trailer Trains (Double‑Ackermann Trailers)**

When trailers themselves have steerable axles (common in **logistic trains** at low speed), a **kinematic chain with additional steering inputs** results.

**State** (tractor + one steerable trailer)

$$
\mathbf{x} =
\begin{bmatrix}
x \\ y \\ \psi_0 \\ \beta_1
\end{bmatrix},
\quad
\mathbf{u} =
\begin{bmatrix}
v_0 \\ \delta_0 \\ \delta_{t,1}
\end{bmatrix},
\quad
\theta = \{L_0, L_1, a, \ldots\}.
$$

**Key modification**  
The trailer yaw rate gains a steering‑curvature term analogous to the 4WS car but expressed in the trailer’s frame:

$$
\dot{\psi}_1 = \frac{v_1}{L_{1,\text{eq}}}\tan\delta_{t,1}
\quad\Rightarrow\quad
\dot{\beta}_1 = \dot{\psi}_1 - \dot{\psi}_0
$$

with $$v_1$$ induced by the hitch kinematics. This structure appears in studies of **double‑Ackermann** trains used for tight‑space manufacturing logistics. [\[KINEMATIC...ING SYSTEM\]](http://www.ijsimm.com/Full_Papers/Fulltext2021/text20-2_550.pdf)

***

# Assumptions & Validity (shared across models)

*   **Kinematic vs Dynamic**  
    Use **kinematic** models for **slow driving** (small slip, low lateral acceleration) and for motion planning; use **dynamic** models when yaw inertia, transient coupling, or mild tire slip must be captured—even at modest speeds. Benchmarks and guidance on validity are provided by Polack et al. and standard vehicle dynamics sources. [\[researchgate.net\]](https://www.researchgate.net/profile/Philip-Polack/publication/318810853_The_kinematic_bicycle_model_A_consistent_model_for_planning_feasible_trajectories_for_autonomous_vehicles/links/5addcbc2a6fdcc29358b9c01/The-kinematic-bicycle-model-A-consistent-model-for-planning-feasible-trajectories-for-autonomous-vehicles.pdf), [\[vtechworks...lib.vt.edu\]](https://vtechworks.lib.vt.edu/bitstream/handle/10919/36615/Chapter2a.pdf)

*   **Non‑holonomic constraints**  
    Encoded as **Pfaffian velocity constraints** ($$\mathbf{A}(\mathbf{q})\dot{\mathbf{q}} = 0$$)—formally treated in geometric mechanics and classical robotics references; these underpin the bicycle and articulated kinematics used above. [\[hal.science\]](https://hal.science/hal-01717298/file/Reduced%20dynamics%20of%20the%20non-holonomic%20Whipple.pdf)

***

## What’s next (step 2 of your request)

Up next I’ll provide a **comparison table** (constraints, states, inputs, parameters) for all models. Do you prefer:

*   a concise **one‑page table**, or
*   a more detailed **matrix of features with brief pros/cons per model**?

Either way, I’ll keep the notation from above for consistency.

***

### References

*   **Kinematic bicycle validity & comparison to high‑DOF models**: Polack, Altché, d’Andréa‑Novel, de La Fortelle, *“The Kinematic Bicycle Model: A Consistent Model for Planning Feasible Trajectories for Autonomous Vehicles”* (2017). [\[researchgate.net\]](https://www.researchgate.net/profile/Philip-Polack/publication/318810853_The_kinematic_bicycle_model_A_consistent_model_for_planning_feasible_trajectories_for_autonomous_vehicles/links/5addcbc2a6fdcc29358b9c01/The-kinematic-bicycle-model-A-consistent-model-for-planning-feasible-trajectories-for-autonomous-vehicles.pdf)
*   **Geometric derivation (ICR) of bicycle model**: *Kinematic Bicycle Model — Algorithms for Automated Driving* (Thomas Fermi). [\[thomasferm....github.io\]](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)
*   **Dynamic bicycle derivations**: Virginia Tech notes (Chapter 2 Vehicle Dynamics Modeling); Univ. at Buffalo Bicycle Model notes. [\[vtechworks...lib.vt.edu\]](https://vtechworks.lib.vt.edu/bitstream/handle/10919/36615/Chapter2a.pdf), [\[code.eng.buffalo.edu\]](https://code.eng.buffalo.edu/dat/sites/model/bicycle.html)
*   **Non‑holonomic mechanics foundations**: Boyer et al., *Reduced dynamics of the non‑holonomic Whipple bicycle* (geometric mechanics framing). [\[hal.science\]](https://hal.science/hal-01717298/file/Reduced%20dynamics%20of%20the%20non-holonomic%20Whipple.pdf)
*   **Steering geometry (Ackermann / parallel / rack‑and‑pinion)**: MathWorks Vehicle Dynamics Blockset documentation. [\[mathworks.com\]](https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html)
*   **Multi‑axle steering linkage kinematics**: Chernenko et al., *Simulation Technique of Kinematic Processes in the Vehicle Steering Linkage*. [\[researchgate.net\]](https://www.researchgate.net/profile/Volodymyr-Kukhar-2/publication/327714832_Simulation_Technique_of_Kinematic_Processes_in_the_Vehicle_Steering_Linkage/links/5ba04d5945851574f7d26214/Simulation-Technique-of-Kinematic-Processes-in-the-Vehicle-Steering-Linkage.pdf)
*   **Logistic trains with double‑Ackermann**: Paszkowiak et al., *Kinematic Model of a Logistic Train with a Double Ackermann Steering System*. [\[KINEMATIC...ING SYSTEM\]](http://www.ijsimm.com/Full_Papers/Fulltext2021/text20-2_550.pdf)
*   **Tractor–trailer low‑speed kinematics & active trailer steering**: van de Wouw et al., *Active Trailer Steering for Robotic Tractor‑Trailer Combinations* (CDC 2015). [\[vandewouw.dc.tue.nl\]](https://vandewouw.dc.tue.nl/CDC2015_vandeWouw_Ritzen.pdf)
*   **Dynamic articulated models**: De Santis et al., *Dynamic model of a two‑trailer articulated vehicle subject to nonholonomic constraints*; Latif et al., *Control of the nonlinear dynamics of a truck and trailer combination*. [\[academia.edu\]](https://www.academia.edu/83346185/Dynamic_model_of_a_two_trailer_articulated_vehicle_subject_to_nonholonomic_constraints), [\[link.springer.com\]](https://link.springer.com/article/10.1007/s11071-019-05452-1)

***
