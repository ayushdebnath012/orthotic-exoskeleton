# Preliminary Motor and Joint-Torque Analysis

## Basis of analysis

Source drawing: `ExoEskeleto Drawing v1.pdf`, 17 sheets, dated 13-07-2026.

This is a preliminary actuator-sizing analysis, not a final safety certification. The drawing provides overall geometry and individual steel-part dimensions, but it does not provide user mass, payload, part masses, centre-of-mass locations, gait cycle, required assistance percentage, motor data, gearbox data, or bearing/friction data.

### Geometry observed in the drawing

- Overall envelope: approximately 1009 mm high, 1009 mm long, and 662 mm wide (sheet 2).
- Principal long members shown on detail sheets: approximately 400-420 mm.
- Circular joint/interface features: approximately 50 mm outside diameter with 10 mm bores on several details.
- Bill of materials: 7 `ELECTROIMAN` units and 11 magnetic laminations/plates, with the listed structural parts identified as steel.
- The assembly appears to provide bilateral lower-limb links connected to a waist ring and foot/boot interfaces.

The seven electromagnets shown in the BOM are not, by themselves, rotary actuators. They may be intended as locks, clutches, or brakes, but the drawing does not define their force, air gap, friction surface, duty cycle, or fail-safe state.

## Reference design case

The following case is used only to obtain first-pass actuator ratings:

| Parameter | Assumption |
|---|---:|
| User mass | 80 kg |
| Thigh length | 0.42 m |
| Shank length | 0.40 m |
| Foot COM distance from ankle | 0.13 m |
| One thigh mass | 10.0% of body mass = 8.00 kg |
| One shank mass | 4.65% of body mass = 3.72 kg |
| One foot mass | 1.45% of body mass = 1.16 kg |
| Thigh and shank COM position | 43.3% of segment length from proximal joint |
| Gravity | 9.81 m/s^2 |
| Dynamic multiplier | 2.0 |
| Engineering factor after dynamics | 1.5 |

These segment fractions exclude the mass of the exoskeleton itself. Actuator, gearbox, battery, and frame masses must be added when CAD mass properties become available.

## Static gravity calculation

The worst gravity moment for a segment occurs when it is approximately horizontal. For a point mass,

`T = m g r`, where `r` is the horizontal distance from the joint axis.

### Hip, one leg held horizontal

`T_hip = g [m_thigh(0.433 L_thigh) + m_shank(L_thigh + 0.433 L_shank) + m_foot(L_thigh + L_shank + r_foot)]`

`T_hip = 46.7 N.m` for the 80 kg reference user.

### Knee, shank and foot held horizontal

`T_knee = g [m_shank(0.433 L_shank) + m_foot(L_shank + r_foot)]`

`T_knee = 12.3 N.m` for the reference user.

### Ankle

Foot self-weight alone gives only about `1.5 N.m`, but this is not the governing ankle case. During standing and walking, ground-reaction force acts at a horizontal distance from the ankle. At full 80 kg load and a 50-120 mm centre-of-pressure offset:

`T_ankle = (80 x 9.81)(0.05 to 0.12) = 39 to 94 N.m`.

In symmetric double support, each side may carry about half this load; in single support, one side can approach or exceed the full value dynamically.

## Lagrange-Euler torque and power calculation

For a planar three-link leg with relative generalized coordinates `q = [q1, q2, q3]^T` for hip, knee, and ankle, define:

`L = K - V`

Applying `tau_i = d/dt(partial L / partial qdot_i) - partial L / partial q_i` gives:

`tau_act = M(q) qdd + C(q, qdot) qdot + G(q) - J_foot^T F_external`.

The external-force term is required during stance. It is zero in the following illustrative swing calculation because the drawing supplies no ground-reaction-force history.

### Numerical state

| Quantity | Hip | Knee | Ankle |
|---|---:|---:|---:|
| Joint angle | 0 deg | 0 deg | 0 deg |
| Joint speed | 1.5 rad/s | 2.0 rad/s | 2.5 rad/s |
| Joint acceleration | 3.0 rad/s^2 | 4.0 rad/s^2 | 5.0 rad/s^2 |

At the fully extended horizontal configuration:

`M = [[2.8397, 1.0382, 0.1610], [1.0382, 0.5094, 0.0976], [0.1610, 0.0976, 0.0373]] kg.m^2`

`G = [46.7308, 12.3518, 1.4793]^T N.m` and `C(q,qdot)qdot = 0` at this particular configuration.

| Joint | Inertial torque | Gravity torque | Total torque | Joint power, `P=tau*qdot` |
|---|---:|---:|---:|---:|
| Hip | 13.477 N.m | 46.731 N.m | **60.208 N.m** | **90.311 W** |
| Knee | 5.640 N.m | 12.352 N.m | **17.992 N.m** | **35.985 W** |
| Ankle | 1.060 N.m | 1.479 N.m | **2.539 N.m** | **6.348 W** |
| Total | | | | **132.644 W** |

For the first joint:

`tau_1 = 2.8397(3) + 1.0382(4) + 0.1610(5) + 46.7308 = 60.208 N.m`

`P_1 = tau_1 qdot_1 = 60.208(1.5) = 90.311 W`.

Using 80:1 hip/knee reducers, a 60:1 ankle reducer, and 75% gearbox efficiency gives motor-side values of 1.003 N.m at 1146 rpm and 120.4 W for the hip, 0.300 N.m at 1528 rpm and 48.0 W for the knee, and 0.056 N.m at 1432 rpm and 8.5 W for the ankle.

These values include human-segment mass only. Add the exoskeleton link and actuator masses to the Lagrangian and measured ground-reaction force through `J_foot^T F_external` for final stance calculations.

## Recommended joint-output ratings

The table below is a practical preliminary envelope for a device intended to support an 80 kg user. Values are output-side ratings after the gearbox. They are intentionally above the simple static limb-weight calculations because walking, starts/stops, imbalance, ground reaction, and exoskeleton mass dominate real operation.

| Joint, each side | Continuous output torque | Peak output torque | Useful output speed | Approx. actuator class |
|---|---:|---:|---:|---|
| Hip flexion/extension | 45-60 N.m | 120-150 N.m | 0-3 rad/s | 300-500 W BLDC + low-backlash reducer |
| Knee flexion/extension | 30-45 N.m | 80-110 N.m | 0-4 rad/s | 250-400 W BLDC + low-backlash reducer |
| Ankle plantar/dorsiflexion | 45-60 N.m | 120-150 N.m | 0-5 rad/s | 350-600 W BLDC + reducer or remote transmission |

For partial assistance, multiply the joint torque targets by the desired assistance fraction. For example, 40% assistance at a 150 N.m ankle peak requires approximately 60 N.m actuator output, before adding friction and transmission losses.

For user mass `M` different from 80 kg, a first approximation is:

`T_new = T_table (M / 80) + torque caused by exoskeleton and payload masses`.

## Motor and gearbox sizing

For a gearbox ratio `N` and efficiency `eta`, required motor torque is:

`T_motor = T_joint / (N eta)`.

Example for a 120 N.m peak joint, 80:1 reduction, and 75% efficiency:

`T_motor,peak = 120 / (80 x 0.75) = 2.0 N.m`.

At 60:1 and 80% efficiency, the same joint requires `2.5 N.m` motor peak torque. Select the motor using its thermal continuous-torque curve, not stall torque. A reducer in the approximate 50:1-100:1 range is a reasonable starting point, but the final ratio must satisfy both torque and joint-speed requirements.

Recommended architecture:

- 48 V BLDC/PMSM actuators to reduce current and cable mass.
- Absolute joint encoder plus motor encoder.
- Low-backlash harmonic, cycloidal, or well-supported planetary reducer.
- Torque sensing through a strain element or series-elastic element.
- Normally engaged mechanical brake at load-bearing joints.
- Mechanical hard stops independent of software.

High reduction improves torque density but reduces backdrivability and increases reflected inertia. For a wearable system, torque sensing and compliance are strongly preferred.

## Electromagnet feasibility check

If an electromagnet at a nominal 50 mm diameter interface is expected to resist joint torque through friction at an effective radius of roughly 25 mm, the required tangential force is:

`F_t = T / r`.

| Joint torque | Tangential force at 25 mm radius |
|---:|---:|
| 60 N.m | 2400 N |
| 120 N.m | 4800 N |
| 150 N.m | 6000 N |

The required magnetic normal force is even higher because friction torque is `T = mu F_normal r`. With a dry friction coefficient of 0.3, holding 120 N.m at 25 mm requires about `16,000 N` normal force. Air gaps, contamination, heating, and loss of electrical power further reduce reliability.

Therefore, the drawn electromagnets should not be treated as the sole load-bearing joint brakes without a demonstrated force/torque test. If they are used for mode selection or locking, use a mechanically positive, normally locked feature so power loss cannot release the user.

## Structural and integration observations

- Several joint details show 10 mm bores. A 10 mm pin carrying human-support loads requires a combined shear, bearing, bending, fatigue, and retaining-feature analysis; diameter alone is not enough.
- Long 400-420 mm links and offset cuffs create bending and torsion in addition to in-plane joint torque.
- The BOM marks all parts as steel, but does not give grade, thickness for every part, heat treatment, weld details, or allowable stress.
- Joint alignment must follow the user's anatomical hip, knee, and ankle axes. Misalignment can produce harmful cuff forces even when motor torque is within limits.
- Motor mass mounted distally greatly increases swing torque. Knee motors should preferably be proximal, and ankle actuation may benefit from a cable/Bowden or linkage transmission.
- The design should not rely on motor torque to prevent collapse during power loss.

## Data required for final sizing

1. Maximum user mass and any carried payload.
2. Intended function: rehabilitation assistance, full weight support, sit-to-stand, walking, or static locking.
3. Required degrees of freedom and angular ranges at each joint.
4. Target gait speed and motion profiles (angle, velocity, acceleration versus time).
5. CAD mass and centre of mass for every moving link, motor, gearbox, battery, and attachment.
6. Electromagnet model, force-versus-gap curve, coil power, friction material, effective radius, and fail-safe behavior.
7. Gearbox ratio, efficiency, backlash, rated output moment, and life.
8. Ground-reaction-force/load cases and required assistance percentage.

## Preliminary conclusion

For an 80 kg design user and full lower-limb support, size each side approximately for 120-150 N.m peak at the hip, 80-110 N.m peak at the knee, and 120-150 N.m peak at the ankle, with the continuous ratings shown above. Six powered flexion/extension joints are implied if both legs' hip, knee, and ankle are actuated. The seven electromagnets shown in the drawing are better treated as auxiliary locks or clutches, not as substitutes for torque-controlled motors or certified fail-safe brakes.

Do not fabricate or test on a person from these preliminary numbers alone. Complete multibody load simulation, structural FEA, brake validation, electrical/thermal analysis, and controlled bench testing first.
