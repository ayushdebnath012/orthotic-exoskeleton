# Euler-Lagrange Joint and Motor Torque Analysis

## ExoEskeleto lower-limb exoskeleton - knee-dominant actuator sizing

**Reference user:** 80 kg  
**Frame:** 6.49 kg, Aluminium 6061-T6  
**Actuation used for sizing:** passive hip, powered knee and ankle  
**Analysis date:** 28 July 2026  
**Status:** engineering sizing study; not a human-use safety certification

---

## 1. Result

The updated manufacturing drawing makes the hip a passive pivot and locates
actuation at the knee and ankle. The motor-torque hierarchy is therefore:

| Joint, each side | Motor status | Continuous output target | Peak output target | Provisional reducer |
|---|---|---:|---:|---:|
| Hip | Passive | 0 N m | 0 N m | None |
| **Knee** | **Powered - highest** | **60 N m** | **120 N m** | **80:1** |
| Ankle | Powered | 30 N m | 60 N m | 60:1 |

The governing knee-dominant case is a bent-knee support/sit-to-stand posture.
Euler-Lagrange inverse dynamics gives:

- Passive hip reaction: -11.03 N m
- **Knee actuator torque: +70.74 N m**
- Ankle actuator torque: -28.30 N m

After a 1.5 engineering factor, the absolute requirements are 16.55, 106.10,
and 42.45 N m. Rounding upward in 20 N m increments gives a 120 N m knee
target and 60 N m ankle target.

The knee is not made highest by changing the equations. It is highest because
the selected design posture places the ground-reaction-force line about
150 mm behind the knee but only 60 mm ahead of the ankle. The passive hip has
no motor even though it still carries a reaction moment.

---

## 2. Workspace evidence and precedence

The following source hierarchy was applied.

### Authoritative hardware input

`ExoEskeleto_Manufacturing_Drawing (1).pdf`, 18 pages, dated 26-07-2026:

- Aluminium 6061-T6 construction.
- Total frame mass: 6.49 kg, excluding motors and hardware.
- Thigh rod `Ac40`: 400 mm.
- Shank rod `Ac30`: 420 mm.
- "Knee & ankle joints house the actuators; hips are passive pivots."

This drawing supersedes the earlier `ExoEskeleto Drawing v1.pdf` where the BOM
listed steel and seven electromagnets but supplied no frame mass or actuator
architecture.

### Geometry and motion sources

- `ExoEskeleto_final.step`: assembly geometry and material-density entities,
  but no directly usable exported per-part mass, centre of mass, or inertia
  table.
- `ExoEskeleto_final.SLDASM`: zero-byte file; it contains no usable assembly
  information.
- `Exoskeleton.stp.SLDPRT`: large SolidWorks part container; no neutral
  per-part mass-properties report is supplied.
- `ExoWalk_frames/blended.json`: 60-frame bilateral joint-angle history.
- `ExoWalk_frames/ExoWalk_fromVideo_v2.gif`: 60 frames at 40 ms/frame, giving
  a 2.4 s cycle.
- Frame PNGs, GIFs, MP4s, and JPG renders confirm a bilateral, sagittal
  hip-knee-ankle linkage but do not define joint zeros or force histories.

### Context only

`aegis.pdf`, `ExoVLA_WM_paper.pdf`, and the ExoVLA-WM proposal describe
assistance control, safety supervision, and fall-risk modelling. They do not
provide this mechanism's motor torque, gearbox, mass-property, or
ground-reaction-force data. The M.Tech allotment form establishes the project
area as robotics but contains no dynamics input.

---

## 3. Euler-Lagrange model

One sagittal-plane leg is represented by three rigid links:

- `q1`: thigh absolute angle from the global horizontal.
- `q2`: knee angle relative to the thigh.
- `q3`: ankle/foot angle relative to the shank.

The absolute link angles are:

`phi1 = q1`

`phi2 = q1 + q2`

`phi3 = q1 + q2 + q3`

For component `a` on link `i`, its kinetic and potential energies are:

`T_a = 0.5 m_a v_a^T v_a + 0.5 I_a omega_i^2`

`V_a = m_a g y_a`

The system Lagrangian is:

`L = T - V`

For each generalized coordinate:

`d/dt(partial L / partial qdot_i) - partial L / partial q_i
 = tau_i + [J_COP(q)^T F]_i`

Rearranging into inverse-dynamics form gives the actuator/reaction demand:

`tau_req = M(q) qdd + C(q,qdot) qdot + G(q) - J_COP(q)^T F`

where:

- `M(q)` is the symmetric positive-definite mass matrix.
- `C(q,qdot) qdot` contains Coriolis and centrifugal terms.
- `G(q)` is the gravity vector.
- `J_COP^T F` maps the foot ground-reaction force into joint coordinates.

The hip entry in `tau_req` is reported as a passive reaction. The commanded
motor vector is:

`tau_motor_output = [0, tau_knee, tau_ankle]^T`

### Numerical validation

The implemented model passed:

| Check | Result |
|---|---:|
| Maximum mass-matrix asymmetry | 0.000e+00 |
| Minimum tested eigenvalue of `M` | 2.132e-02 kg m^2 |
| Coriolis term at zero velocity | 0.000e+00 N m |
| Euler-Lagrange power-identity error | 3.553e-15 W |

---

## 4. Parameters

### Human segments, one leg

The nominal user mass is 80 kg.

| Segment | Mass fraction | Mass | COM from proximal joint | Radius-of-gyration fraction |
|---|---:|---:|---:|---:|
| Thigh | 10.00% | 8.00 kg | 0.433 x 0.400 = 0.1732 m | 0.323 |
| Shank | 4.65% | 3.72 kg | 0.433 x 0.420 = 0.1819 m | 0.302 |
| Foot | 1.45% | 1.16 kg | 0.500 x 0.260 = 0.1300 m | 0.475 |

### Exoskeleton moving-link allocation

The 6.49 kg total is known, but the drawing does not give per-part mass
properties. The documented nominal allocation is:

| Moving exoskeleton group, one leg | Assigned mass | COM model |
|---|---:|---|
| Thigh link and local plates | 0.75 kg | 50% of link |
| Shank link and local plates | 0.90 kg | 50% of link |
| Foot link and local plates | 0.60 kg | 50% of link |
| **Moving subtotal, two legs** | **4.50 kg** | |
| Fixed pelvis/waist remainder | **1.99 kg** | Not in moving-link kinetic energy |
| **Total** | **6.49 kg** | |

Motor and gearbox mass is excluded because no model or mass is specified. A
final CAD mass-property export should replace this allocation.

---

## 5. CAD gait swing result

The 60-frame `blended.json` sequence was evaluated over the measured 2.4 s GIF
cycle. The first frame was used as the only available joint-zero calibration:
vertical thigh, extended knee, horizontal foot. The stance force was set to
zero, so this is a swing/inertial result only.

| Side | Joint | Peak absolute torque | RMS torque | Peak speed | Peak absolute power |
|---|---|---:|---:|---:|---:|
| Left | Passive hip reaction | 34.67 N m | 15.21 N m | 1.679 rad/s | 39.39 W |
| Left | **Knee motor** | **10.18 N m** | **6.24 N m** | **2.007 rad/s** | **17.84 W** |
| Left | Ankle motor | 3.67 N m | 2.36 N m | 1.157 rad/s | 3.54 W |
| Right | Passive hip reaction | 43.05 N m | 20.97 N m | 2.186 rad/s | 55.26 W |
| Right | **Knee motor** | **14.35 N m** | **7.22 N m** | **1.512 rad/s** | **16.70 W** |
| Right | Ankle motor | 3.20 N m | 2.19 N m | 0.965 rad/s | 2.41 W |

The knee is the highest powered joint in the supplied swing motion. The larger
hip result is a passive structural/pelvis reaction, not motor torque.

The animation is not a measured human gait dataset. Its joint naming, zero
offsets, and axis signs are not exported, and it contains no force-plate data.
It is useful for a first dynamic check, not final actuator certification.

---

## 6. Governing knee-dominant support case

The design posture uses:

- `q = [-60, -60, 120] deg`, giving a horizontal foot.
- Vertical support per leg:
  `F_y = 0.60 (80 + 6.49) 9.81 = 509.08 N`.
- This equals 1.2 combined body/frame weight split between two feet.
- Centre of pressure: 60 mm forward of the ankle.
- Quasi-static peak: `qdot = 0`, `qdd = 0`.

At this posture:

`M(q) =`

`[[2.668720, 0.867689, 0.048529],`

` [0.867689, 0.500985, 0.002769],`

` [0.048529, 0.002769, 0.050817]] kg m^2`

The torque decomposition is:

| Joint | `M qdd` | `C qdot` | `G` | `J_COP^T F` | `tau_req` | `1.5 abs(tau_req)` |
|---|---:|---:|---:|---:|---:|---:|
| Passive hip reaction | 0 | 0 | +14.423 | +25.454 | -11.031 | 16.546 N m |
| **Knee motor** | **0** | **0** | **-5.627** | **-76.362** | **+70.735** | **106.103 N m** |
| Ankle motor | 0 | 0 | +2.245 | +30.545 | -28.300 | 42.450 N m |

The knee ground-force moment is the governing term. The result directly
supports a knee-highest actuator rating for support and sit-to-stand.

---

## 7. Motor and gearbox mapping

For reduction ratio `N` and gearbox efficiency `eta`:

`T_motor = T_output / (N eta)`

`rpm_motor = qdot_joint N 60 / (2 pi)`

| Joint, each side | Output continuous | Output peak | Ratio | Efficiency | Motor continuous | Motor peak | Motor rpm at measured gait peak |
|---|---:|---:|---:|---:|---:|---:|---:|
| **Knee** | **60 N m** | **120 N m** | **80:1** | **0.75** | **1.000 N m** | **2.000 N m** | **1533 rpm** |
| Ankle | 30 N m | 60 N m | 60:1 | 0.75 | 0.667 N m | 1.333 N m | 663 rpm |

Provisional motor classes:

- Knee: 48 V BLDC/PMSM, approximately 300-400 W, with a reducer rated for at
  least 120 N m peak output and the required shock/fatigue life.
- Ankle: 48 V BLDC/PMSM, approximately 150-250 W, with a reducer rated for at
  least 60 N m peak output.

These power classes are provisional because the supplied files contain no
measured loaded joint-speed history or duty cycle. Select against the motor's
thermal continuous-torque curve, not stall torque.

The passive hip still needs bearings, stops, and structure sized for reaction
torque. The maximum swing reaction here is 43.05 N m; applying 1.5 gives
64.58 N m before shock/fatigue allowance.

---

## 8. Sensitivity and limits of the knee-highest condition

### User mass sensitivity

The same bent-knee case gives:

| User mass | Raw knee torque | Factored knee torque | Raw ankle torque | Factored ankle torque |
|---:|---:|---:|---:|---:|
| 60 kg | 54.14 N m | 81.20 N m | 21.61 N m | 32.41 N m |
| 70 kg | 62.44 N m | 93.65 N m | 24.95 N m | 37.43 N m |
| **80 kg** | **70.74 N m** | **106.10 N m** | **28.30 N m** | **42.45 N m** |
| 90 kg | 79.04 N m | 118.55 N m | 31.65 N m | 47.47 N m |
| 100 kg | 87.34 N m | 131.00 N m | 34.99 N m | 52.49 N m |

The 120 N m knee target covers this stated case through approximately 90 kg.
For a 100 kg user, use at least a 140 N m knee output target under the same
assumptions.

### Centre-of-pressure sensitivity

| COP forward of ankle | Absolute hip reaction | Absolute knee torque | Absolute ankle torque |
|---:|---:|---:|---:|
| 20 mm | 9.33 N m | 91.10 N m | 7.94 N m |
| 40 mm | 0.85 N m | 80.92 N m | 18.12 N m |
| **60 mm** | **11.03 N m** | **70.74 N m** | **28.30 N m** |
| 80 mm | 21.21 N m | 60.55 N m | 38.48 N m |
| 100 mm | 31.39 N m | 50.37 N m | 48.66 N m |
| 120 mm | 41.58 N m | 40.19 N m | 58.85 N m |

The knee is highest for the selected support/sit-to-stand loading. During
far-forward toe-off, ankle torque can physically exceed knee torque. If the
device must provide full powered push-off, the ankle must be re-sized from
measured force-plate and COP histories; it is not correct to suppress that
load merely to preserve a desired ranking.

---

## 9. Required data before hardware release

1. Maximum user mass, carried payload, and required assistance percentage.
2. Native CAD mass, COM, and inertia for every moving part, motor, and gearbox.
3. Exact motor and reducer models, efficiency maps, backlash, reflected
   inertia, output-bearing moment rating, and thermal curves.
4. Calibrated joint zeros and axis signs for every CAD `Revolute` channel.
5. Loaded joint angle, velocity, and acceleration versus time.
6. Synchronized ground-reaction force and COP trajectories.
7. Friction, cable/gear preload, cuff forces, hard-stop loads, and emergency
   braking loads.
8. Bench verification with an instrumented dummy load before any human test.

## Conclusion

For the current 80 kg reference case and the actuator architecture stated in
the updated manufacturing drawing, use **120 N m peak / 60 N m continuous at
each knee** and **60 N m peak / 30 N m continuous at each ankle**. The hip is
passive and has zero motor torque, although its structure must carry reaction
loads. This gives the requested and physically justified ordering:

**knee motor torque > ankle motor torque > hip motor torque (zero).**

