"""Euler-Lagrange torque analysis for the ExoEskeleto lower-limb frame.

The model is a sagittal-plane, three-link human-exoskeleton leg:

    q1 - thigh absolute angle measured from the global horizontal
    q2 - knee angle relative to the thigh
    q3 - ankle/foot angle relative to the shank

The updated manufacturing drawing states that the hip is passive and the knee
and ankle house the actuators.  The inverse-dynamics hip result is therefore a
required passive joint/pelvis reaction, not a motor command.

Two load cases are evaluated:

1. Swing: kinematics from ``ExoWalk_frames/blended.json`` over the 2.4 s GIF
   cycle, with no ground reaction force.
2. Bent-knee support / sit-to-stand: 60 degree thigh and shank inclination,
   1.2 body-weight total vertical support shared equally by both feet, and a
   60 mm centre-of-pressure offset from the ankle.

The script writes reproducible CSV, JSON, and PNG outputs when ``--write`` is
used.  It intentionally keeps unprovided motor mass, friction, and rotor
inertia out of the nominal result and calls those omissions out in the report.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter


G_ACCEL = 9.81
JOINT_NAMES = ("Hip reaction", "Knee motor", "Ankle motor")
SHORT_JOINT_NAMES = ("Hip", "Knee", "Ankle")

# Updated manufacturing drawing, Note 5.2.
LINK_LENGTHS = np.array([0.400, 0.420, 0.260], dtype=float)

# Dempster-style body-segment estimates retained from the v1 calculation.
HUMAN_MASS_FRACTIONS = np.array([0.1000, 0.0465, 0.0145], dtype=float)
HUMAN_COM_FRACTIONS = np.array([0.433, 0.433, 0.500], dtype=float)
HUMAN_GYRATION_FRACTIONS = np.array([0.323, 0.302, 0.475], dtype=float)

# The drawing gives only the 6.49 kg total frame mass, not per-part mass
# properties.  The nominal moving-link allocation below totals 2.25 kg/leg;
# the remaining 1.99 kg is assigned to the fixed pelvis/waist structure.
DEFAULT_EXO_LINK_MASSES = np.array([0.750, 0.900, 0.600], dtype=float)
DEFAULT_FRAME_MASS = 6.49

DEFAULT_USER_MASS = 80.0
DEFAULT_CYCLE_TIME = 2.4
DEFAULT_COP_DISTANCE = 0.060
DEFAULT_SUPPORT_MULTIPLIER_PER_LEG = 0.60  # 1.2 BW total / two legs
DEFAULT_ENGINEERING_FACTOR = 1.50

DEFAULT_REDUCTION_RATIOS = {"knee": 80.0, "ankle": 60.0}
DEFAULT_GEAR_EFFICIENCY = {"knee": 0.75, "ankle": 0.75}

GAIT_JOINT_MAP = {
    "left": ("Revolute 4", "Revolute 30", "Revolute 67"),
    "right": ("Revolute 2", "Revolute 19", "Revolute 29"),
}


@dataclass(frozen=True)
class RigidComponent:
    """A rigid mass component attached to one serial link."""

    name: str
    link: int
    mass: float
    com_from_proximal: float
    inertia_about_com: float


@dataclass(frozen=True)
class JointSummary:
    """Peak and RMS values for one gait-side/joint combination."""

    peak_abs_torque_nm: float
    rms_torque_nm: float
    peak_abs_speed_rad_s: float
    peak_abs_accel_rad_s2: float
    peak_abs_power_w: float
    rms_power_w: float


@dataclass(frozen=True)
class MotorRating:
    joint: str
    continuous_output_nm: float
    peak_output_nm: float
    reduction_ratio: float
    efficiency: float
    continuous_motor_nm: float
    peak_motor_nm: float
    measured_peak_joint_speed_rad_s: float
    measured_peak_motor_rpm: float


def build_components(
    user_mass: float,
    exo_link_masses: Sequence[float] = DEFAULT_EXO_LINK_MASSES,
) -> list[RigidComponent]:
    """Return human and frame components for one leg."""

    exo_link_masses = np.asarray(exo_link_masses, dtype=float)
    if exo_link_masses.shape != (3,) or np.any(exo_link_masses < 0.0):
        raise ValueError("exo_link_masses must contain three non-negative values")

    human_masses = HUMAN_MASS_FRACTIONS * user_mass
    components: list[RigidComponent] = []
    uniform_rod_gyration_fraction = 1.0 / math.sqrt(12.0)

    for index, length in enumerate(LINK_LENGTHS):
        human_mass = float(human_masses[index])
        human_com = float(HUMAN_COM_FRACTIONS[index] * length)
        human_inertia = float(
            human_mass * (HUMAN_GYRATION_FRACTIONS[index] * length) ** 2
        )
        components.append(
            RigidComponent(
                name=f"human_{SHORT_JOINT_NAMES[index].lower()}_segment",
                link=index,
                mass=human_mass,
                com_from_proximal=human_com,
                inertia_about_com=human_inertia,
            )
        )

        exo_mass = float(exo_link_masses[index])
        exo_com = float(0.5 * length)
        exo_inertia = float(
            exo_mass * (uniform_rod_gyration_fraction * length) ** 2
        )
        components.append(
            RigidComponent(
                name=f"exo_{SHORT_JOINT_NAMES[index].lower()}_link",
                link=index,
                mass=exo_mass,
                com_from_proximal=exo_com,
                inertia_about_com=exo_inertia,
            )
        )

    return components


def cumulative_angles(q: Sequence[float]) -> np.ndarray:
    """Return the three absolute link angles."""

    q_array = np.asarray(q, dtype=float)
    return np.cumsum(q_array)


def component_com_jacobian(
    component: RigidComponent,
    q: Sequence[float],
) -> np.ndarray:
    """Return the 2 x 3 translational Jacobian of a component COM."""

    phi = cumulative_angles(q)
    jacobian = np.zeros((2, 3), dtype=float)
    link_index = component.link

    for joint in range(link_index + 1):
        for link in range(joint, link_index):
            jacobian[0, joint] -= LINK_LENGTHS[link] * math.sin(phi[link])
            jacobian[1, joint] += LINK_LENGTHS[link] * math.cos(phi[link])

        jacobian[0, joint] -= component.com_from_proximal * math.sin(
            phi[link_index]
        )
        jacobian[1, joint] += component.com_from_proximal * math.cos(
            phi[link_index]
        )

    return jacobian


def cop_jacobian(
    q: Sequence[float],
    cop_distance: float = DEFAULT_COP_DISTANCE,
) -> np.ndarray:
    """Return the 2 x 3 Jacobian at the foot centre of pressure.

    ``cop_distance`` is measured forward from the ankle along the foot link.
    It is not assumed to be the toe position.
    """

    phi = cumulative_angles(q)
    jacobian = np.zeros((2, 3), dtype=float)

    for joint in range(3):
        for link in range(joint, 2):
            jacobian[0, joint] -= LINK_LENGTHS[link] * math.sin(phi[link])
            jacobian[1, joint] += LINK_LENGTHS[link] * math.cos(phi[link])

        jacobian[0, joint] -= cop_distance * math.sin(phi[2])
        jacobian[1, joint] += cop_distance * math.cos(phi[2])

    return jacobian


def mass_matrix(
    q: Sequence[float],
    components: Iterable[RigidComponent],
) -> np.ndarray:
    """Return M(q) in kg m^2."""

    matrix = np.zeros((3, 3), dtype=float)

    for component in components:
        jacobian = component_com_jacobian(component, q)
        matrix += component.mass * jacobian.T @ jacobian
        matrix[
            : component.link + 1,
            : component.link + 1,
        ] += component.inertia_about_com

    return matrix


def gravity_vector(
    q: Sequence[float],
    components: Iterable[RigidComponent],
) -> np.ndarray:
    """Return G(q) = partial(V)/partial(q) in N m."""

    vector = np.zeros(3, dtype=float)

    for component in components:
        jacobian = component_com_jacobian(component, q)
        vector += component.mass * G_ACCEL * jacobian[1, :]

    return vector


def mass_matrix_derivatives(
    q: Sequence[float],
    components: Sequence[RigidComponent],
    step: float = 1.0e-6,
) -> np.ndarray:
    """Return dM/dq_k as an array with shape (3, 3, 3)."""

    q_array = np.asarray(q, dtype=float)
    derivatives = np.zeros((3, 3, 3), dtype=float)

    for coordinate in range(3):
        q_plus = q_array.copy()
        q_minus = q_array.copy()
        q_plus[coordinate] += step
        q_minus[coordinate] -= step
        derivatives[coordinate] = (
            mass_matrix(q_plus, components)
            - mass_matrix(q_minus, components)
        ) / (2.0 * step)

    return derivatives


def coriolis_vector(
    q: Sequence[float],
    qd: Sequence[float],
    components: Sequence[RigidComponent],
) -> np.ndarray:
    """Return C(q, qdot) qdot from the Christoffel symbols."""

    qd_array = np.asarray(qd, dtype=float)
    derivatives = mass_matrix_derivatives(q, components)
    vector = np.zeros(3, dtype=float)

    for output in range(3):
        for j in range(3):
            for k in range(3):
                gamma = 0.5 * (
                    derivatives[k, output, j]
                    + derivatives[j, output, k]
                    - derivatives[output, j, k]
                )
                vector[output] += gamma * qd_array[j] * qd_array[k]

    return vector


def generalized_external_force(
    q: Sequence[float],
    foot_force_xy: Sequence[float],
    cop_distance: float,
) -> np.ndarray:
    """Return J_COP(q)^T F in N m."""

    jacobian = cop_jacobian(q, cop_distance)
    return jacobian.T @ np.asarray(foot_force_xy, dtype=float)


def inverse_dynamics(
    q: Sequence[float],
    qd: Sequence[float],
    qdd: Sequence[float],
    components: Sequence[RigidComponent],
    foot_force_xy: Sequence[float] = (0.0, 0.0),
    cop_distance: float = DEFAULT_COP_DISTANCE,
) -> dict[str, np.ndarray]:
    """Evaluate tau = M qdd + C qdot + G - J_COP^T F."""

    q_array = np.asarray(q, dtype=float)
    qd_array = np.asarray(qd, dtype=float)
    qdd_array = np.asarray(qdd, dtype=float)

    matrix = mass_matrix(q_array, components)
    inertia = matrix @ qdd_array
    coriolis = coriolis_vector(q_array, qd_array, components)
    gravity = gravity_vector(q_array, components)
    external = generalized_external_force(
        q_array,
        foot_force_xy,
        cop_distance,
    )
    torque = inertia + coriolis + gravity - external
    power = torque * qd_array

    return {
        "mass_matrix": matrix,
        "inertia": inertia,
        "coriolis": coriolis,
        "gravity": gravity,
        "external": external,
        "torque": torque,
        "power": power,
    }


def validate_model(components: Sequence[RigidComponent]) -> dict[str, float]:
    """Run numerical consistency checks on the Euler-Lagrange model."""

    q = np.deg2rad(np.array([-67.0, 38.0, 25.0]))
    qd = np.array([0.8, -1.2, 0.6])
    qdd = np.array([1.1, -0.7, 0.9])

    matrix = mass_matrix(q, components)
    symmetry_error = float(np.max(np.abs(matrix - matrix.T)))
    minimum_eigenvalue = float(np.min(np.linalg.eigvalsh(matrix)))
    zero_velocity_coriolis = float(
        np.max(np.abs(coriolis_vector(q, np.zeros(3), components)))
    )

    derivatives = mass_matrix_derivatives(q, components)
    matrix_dot = sum(derivatives[k] * qd[k] for k in range(3))
    coriolis = coriolis_vector(q, qd, components)
    lagrange_power = float(
        qd
        @ (
            matrix @ qdd
            + coriolis
            + gravity_vector(q, components)
        )
    )
    energy_rate = float(
        qd @ matrix @ qdd
        + 0.5 * qd @ matrix_dot @ qd
        + gravity_vector(q, components) @ qd
    )
    power_identity_error = abs(lagrange_power - energy_rate)

    if symmetry_error > 1.0e-9:
        raise RuntimeError(f"Mass matrix symmetry check failed: {symmetry_error}")
    if minimum_eigenvalue <= 0.0:
        raise RuntimeError(
            f"Mass matrix positive-definiteness check failed: {minimum_eigenvalue}"
        )
    if zero_velocity_coriolis > 1.0e-10:
        raise RuntimeError(
            f"Zero-velocity Coriolis check failed: {zero_velocity_coriolis}"
        )
    if power_identity_error > 1.0e-7:
        raise RuntimeError(
            f"Euler-Lagrange power identity failed: {power_identity_error}"
        )

    return {
        "mass_matrix_symmetry_error": symmetry_error,
        "minimum_mass_matrix_eigenvalue": minimum_eigenvalue,
        "zero_velocity_coriolis_error": zero_velocity_coriolis,
        "lagrange_power_identity_error": power_identity_error,
    }


def _odd_window(sample_count: int, desired: int = 9) -> int:
    window = min(desired, sample_count - (1 - sample_count % 2))
    if window < 5:
        raise ValueError("At least five gait samples are required")
    return window if window % 2 else window - 1


def load_gait_coordinates(
    gait_json: Path,
    cycle_time: float,
) -> tuple[np.ndarray, dict[str, dict[str, np.ndarray]]]:
    """Map the CAD revolute histories to q, qdot, and qdd.

    The first animation frame is treated as the neutral calibration pose:
    thigh vertical, knee extended, and foot horizontal.  This is the only
    defensible mapping available because the JSON contains no joint-zero or
    axis-sign metadata.
    """

    payload = json.loads(gait_json.read_text(encoding="utf-8"))
    angles = payload["angles"]
    sample_count = int(payload["frames"])
    time = np.arange(sample_count, dtype=float) * cycle_time / sample_count
    sample_period = cycle_time / sample_count
    window = _odd_window(sample_count)
    result: dict[str, dict[str, np.ndarray]] = {}

    neutral = np.deg2rad(np.array([-90.0, 0.0, 90.0]))

    for side, joint_names in GAIT_JOINT_MAP.items():
        raw = np.vstack(
            [
                np.unwrap(np.deg2rad(np.asarray(angles[name], dtype=float)))
                for name in joint_names
            ]
        ).T
        q = raw - raw[0, :] + neutral
        qd = savgol_filter(
            q,
            window_length=window,
            polyorder=3,
            deriv=1,
            delta=sample_period,
            axis=0,
            mode="wrap",
        )
        qdd = savgol_filter(
            q,
            window_length=window,
            polyorder=3,
            deriv=2,
            delta=sample_period,
            axis=0,
            mode="wrap",
        )
        result[side] = {"q": q, "qd": qd, "qdd": qdd}

    return time, result


def evaluate_gait(
    gait_json: Path,
    cycle_time: float,
    components: Sequence[RigidComponent],
) -> tuple[
    np.ndarray,
    dict[str, dict[str, np.ndarray]],
    dict[str, dict[str, JointSummary]],
]:
    """Run swing-phase inverse dynamics for both animated legs."""

    time, coordinates = load_gait_coordinates(gait_json, cycle_time)
    detailed: dict[str, dict[str, np.ndarray]] = {}
    summary: dict[str, dict[str, JointSummary]] = {}

    for side, motion in coordinates.items():
        torque_rows = []
        power_rows = []
        inertia_rows = []
        coriolis_rows = []
        gravity_rows = []

        for q, qd, qdd in zip(motion["q"], motion["qd"], motion["qdd"]):
            values = inverse_dynamics(q, qd, qdd, components)
            torque_rows.append(values["torque"])
            power_rows.append(values["power"])
            inertia_rows.append(values["inertia"])
            coriolis_rows.append(values["coriolis"])
            gravity_rows.append(values["gravity"])

        torque = np.asarray(torque_rows)
        power = np.asarray(power_rows)
        detailed[side] = {
            **motion,
            "torque": torque,
            "power": power,
            "inertia": np.asarray(inertia_rows),
            "coriolis": np.asarray(coriolis_rows),
            "gravity": np.asarray(gravity_rows),
        }
        summary[side] = {}

        for index, joint in enumerate(SHORT_JOINT_NAMES):
            summary[side][joint.lower()] = JointSummary(
                peak_abs_torque_nm=float(np.max(np.abs(torque[:, index]))),
                rms_torque_nm=float(np.sqrt(np.mean(torque[:, index] ** 2))),
                peak_abs_speed_rad_s=float(
                    np.max(np.abs(motion["qd"][:, index]))
                ),
                peak_abs_accel_rad_s2=float(
                    np.max(np.abs(motion["qdd"][:, index]))
                ),
                peak_abs_power_w=float(np.max(np.abs(power[:, index]))),
                rms_power_w=float(np.sqrt(np.mean(power[:, index] ** 2))),
            )

    return time, detailed, summary


def evaluate_support_case(
    user_mass: float,
    frame_mass: float,
    components: Sequence[RigidComponent],
    cop_distance: float,
    support_multiplier_per_leg: float,
) -> dict[str, np.ndarray | float]:
    """Evaluate the knee-dominant bent-knee support posture."""

    q = np.deg2rad(np.array([-60.0, -60.0, 120.0]))
    vertical_force = (
        support_multiplier_per_leg
        * (user_mass + frame_mass)
        * G_ACCEL
    )
    values = inverse_dynamics(
        q=q,
        qd=np.zeros(3),
        qdd=np.zeros(3),
        components=components,
        foot_force_xy=(0.0, vertical_force),
        cop_distance=cop_distance,
    )
    values["q"] = q
    values["vertical_force_n"] = vertical_force
    values["cop_distance_m"] = cop_distance
    return values


def round_up(value: float, increment: float) -> float:
    return math.ceil(value / increment) * increment


def select_motor_ratings(
    support_torque: Sequence[float],
    gait_summary: dict[str, dict[str, JointSummary]],
    engineering_factor: float,
) -> list[MotorRating]:
    """Map factored output torque to conservative motor-side targets."""

    peak_speeds = {
        joint: max(
            gait_summary["left"][joint].peak_abs_speed_rad_s,
            gait_summary["right"][joint].peak_abs_speed_rad_s,
        )
        for joint in ("knee", "ankle")
    }
    factored = np.abs(np.asarray(support_torque, dtype=float)) * engineering_factor

    ratings: list[MotorRating] = []
    for joint, index in (("knee", 1), ("ankle", 2)):
        # 20 N m increments retain margin and produce transparent selections:
        # 106.1 -> 120 N m knee; 42.5 -> 60 N m ankle.
        peak_output = round_up(float(factored[index]), 20.0)
        continuous_output = 0.5 * peak_output
        ratio = DEFAULT_REDUCTION_RATIOS[joint]
        efficiency = DEFAULT_GEAR_EFFICIENCY[joint]
        peak_motor = peak_output / (ratio * efficiency)
        continuous_motor = continuous_output / (ratio * efficiency)
        peak_speed = peak_speeds[joint]
        motor_rpm = peak_speed * ratio * 60.0 / (2.0 * math.pi)
        ratings.append(
            MotorRating(
                joint=joint.title(),
                continuous_output_nm=continuous_output,
                peak_output_nm=peak_output,
                reduction_ratio=ratio,
                efficiency=efficiency,
                continuous_motor_nm=continuous_motor,
                peak_motor_nm=peak_motor,
                measured_peak_joint_speed_rad_s=peak_speed,
                measured_peak_motor_rpm=motor_rpm,
            )
        )

    return ratings


def write_gait_csv(
    output_path: Path,
    time: np.ndarray,
    gait_detail: dict[str, dict[str, np.ndarray]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    headers = [
        "time_s",
        "side",
        "hip_angle_deg",
        "knee_angle_deg",
        "ankle_angle_deg",
        "hip_speed_rad_s",
        "knee_speed_rad_s",
        "ankle_speed_rad_s",
        "hip_accel_rad_s2",
        "knee_accel_rad_s2",
        "ankle_accel_rad_s2",
        "hip_reaction_nm",
        "knee_motor_nm",
        "ankle_motor_nm",
        "hip_power_w",
        "knee_power_w",
        "ankle_power_w",
    ]

    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        for side, values in gait_detail.items():
            for row, time_value in enumerate(time):
                writer.writerow(
                    [
                        f"{time_value:.6f}",
                        side,
                        *[f"{value:.6f}" for value in np.rad2deg(values["q"][row])],
                        *[f"{value:.6f}" for value in values["qd"][row]],
                        *[f"{value:.6f}" for value in values["qdd"][row]],
                        *[f"{value:.6f}" for value in values["torque"][row]],
                        *[f"{value:.6f}" for value in values["power"][row]],
                    ]
                )


def plot_gait_torques(
    output_path: Path,
    time: np.ndarray,
    gait_detail: dict[str, dict[str, np.ndarray]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    colors = {"left": "#1565C0", "right": "#E65100"}
    fig, axes = plt.subplots(2, 1, figsize=(10.0, 6.4), sharex=True)

    for side, values in gait_detail.items():
        axes[0].plot(
            time,
            values["torque"][:, 1],
            label=f"{side.title()} knee",
            color=colors[side],
            linewidth=2.1,
        )
        axes[1].plot(
            time,
            values["torque"][:, 2],
            label=f"{side.title()} ankle",
            color=colors[side],
            linewidth=2.1,
        )

    axes[0].set_title("Euler-Lagrange swing torque from the 2.4 s CAD gait")
    axes[0].set_ylabel("Knee torque [N m]")
    axes[1].set_ylabel("Ankle torque [N m]")
    axes[1].set_xlabel("Time [s]")
    for axis in axes:
        axis.axhline(0.0, color="#263238", linewidth=0.8)
        axis.grid(True, color="#CFD8DC", linewidth=0.7, alpha=0.8)
        axis.legend(frameon=False, ncol=2, loc="upper right")
        axis.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_support_case(
    output_path: Path,
    support_values: dict[str, np.ndarray | float],
    engineering_factor: float,
    ratings: Sequence[MotorRating],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    raw = np.abs(np.asarray(support_values["torque"], dtype=float))
    factored = raw * engineering_factor
    selected = np.array(
        [0.0, ratings[0].peak_output_nm, ratings[1].peak_output_nm]
    )
    x = np.arange(3)
    width = 0.24

    fig, axis = plt.subplots(figsize=(9.4, 5.2))
    axis.bar(x - width, raw, width, label="Euler-Lagrange load", color="#90A4AE")
    axis.bar(x, factored, width, label=f"x {engineering_factor:.1f} factor", color="#FFB300")
    axis.bar(x + width, selected, width, label="Selected motor output", color="#00695C")
    axis.set_xticks(x, ["Passive hip reaction", "Knee motor", "Ankle motor"])
    axis.set_ylabel("Absolute torque [N m]")
    axis.set_title("Knee-dominant bent-knee support case")
    axis.grid(axis="y", color="#CFD8DC", linewidth=0.7, alpha=0.8)
    axis.legend(frameon=False, ncol=3, loc="upper left")
    axis.spines[["top", "right"]].set_visible(False)
    axis.set_ylim(0.0, max(selected) * 1.28)

    for offset, values in ((-width, raw), (0.0, factored), (width, selected)):
        for index, value in enumerate(values):
            if value > 0.0:
                axis.text(
                    index + offset,
                    value + 2.0,
                    f"{value:.1f}",
                    ha="center",
                    va="bottom",
                    fontsize=9,
                )

    fig.tight_layout()
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def serialize_results(
    output_path: Path,
    *,
    user_mass: float,
    frame_mass: float,
    exo_link_masses: Sequence[float],
    validation: dict[str, float],
    gait_summary: dict[str, dict[str, JointSummary]],
    support_values: dict[str, np.ndarray | float],
    engineering_factor: float,
    ratings: Sequence[MotorRating],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    moving_exo_mass = 2.0 * float(np.sum(exo_link_masses))
    payload = {
        "model": {
            "user_mass_kg": user_mass,
            "frame_mass_kg": frame_mass,
            "link_lengths_m": LINK_LENGTHS.tolist(),
            "exo_link_masses_per_leg_kg": list(map(float, exo_link_masses)),
            "moving_exo_mass_bilateral_kg": moving_exo_mass,
            "fixed_pelvis_frame_mass_kg": frame_mass - moving_exo_mass,
            "hip_actuation": "passive",
            "coordinates": [
                "q1 thigh absolute angle from horizontal",
                "q2 knee relative angle",
                "q3 ankle/foot relative angle",
            ],
        },
        "validation": validation,
        "gait_summary": {
            side: {joint: asdict(values) for joint, values in joints.items()}
            for side, joints in gait_summary.items()
        },
        "support_case": {
            "posture_deg": np.rad2deg(
                np.asarray(support_values["q"], dtype=float)
            ).tolist(),
            "vertical_force_per_leg_n": float(
                support_values["vertical_force_n"]
            ),
            "cop_distance_m": float(support_values["cop_distance_m"]),
            "inertia_nm": np.asarray(support_values["inertia"]).tolist(),
            "coriolis_nm": np.asarray(support_values["coriolis"]).tolist(),
            "gravity_nm": np.asarray(support_values["gravity"]).tolist(),
            "external_generalized_nm": np.asarray(
                support_values["external"]
            ).tolist(),
            "inverse_dynamics_nm": np.asarray(
                support_values["torque"]
            ).tolist(),
            "engineering_factor": engineering_factor,
            "factored_absolute_nm": (
                np.abs(np.asarray(support_values["torque"]))
                * engineering_factor
            ).tolist(),
        },
        "selected_motor_ratings": [asdict(rating) for rating in ratings],
        "limitations": [
            "CAD gait joint zeros and axis signs are not exported; first frame is the neutral calibration.",
            "No measured ground-reaction-force history is available.",
            "Motor and gearbox masses, rotor inertia, friction, backlash, and efficiency maps are not supplied.",
            "The 6.49 kg frame has no per-part mass properties; moving-link masses are a documented allocation.",
            "The result is for actuator sizing and is not a human-use safety certification.",
        ],
    }
    output_path.write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )


def print_summary(
    validation: dict[str, float],
    gait_summary: dict[str, dict[str, JointSummary]],
    support_values: dict[str, np.ndarray | float],
    engineering_factor: float,
    ratings: Sequence[MotorRating],
) -> None:
    print("Euler-Lagrange model checks")
    for key, value in validation.items():
        print(f"  {key}: {value:.6e}")

    print("\nCAD gait swing results (no ground reaction)")
    print(
        f"{'Side':<7}{'Joint':<8}{'Peak T [N m]':>15}"
        f"{'RMS T [N m]':>15}{'Peak speed':>13}{'Peak power [W]':>16}"
    )
    for side in ("left", "right"):
        for joint in ("hip", "knee", "ankle"):
            values = gait_summary[side][joint]
            print(
                f"{side.title():<7}{joint.title():<8}"
                f"{values.peak_abs_torque_nm:15.3f}"
                f"{values.rms_torque_nm:15.3f}"
                f"{values.peak_abs_speed_rad_s:13.3f}"
                f"{values.peak_abs_power_w:16.3f}"
            )

    print("\nBent-knee support / sit-to-stand load case")
    print(f"  Vertical force per leg: {support_values['vertical_force_n']:.3f} N")
    print(
        f"{'Joint':<16}{'Gravity':>12}{'J^T F':>12}"
        f"{'Inverse dyn.':>15}{'Factored abs.':>15}"
    )
    torque = np.asarray(support_values["torque"])
    gravity = np.asarray(support_values["gravity"])
    external = np.asarray(support_values["external"])
    factored = np.abs(torque) * engineering_factor
    for index, joint in enumerate(JOINT_NAMES):
        print(
            f"{joint:<16}{gravity[index]:12.3f}{external[index]:12.3f}"
            f"{torque[index]:15.3f}{factored[index]:15.3f}"
        )

    print("\nSelected actuator ratings (each side)")
    print(
        f"{'Joint':<8}{'Cont out':>11}{'Peak out':>11}"
        f"{'Ratio':>9}{'Cont motor':>13}{'Peak motor':>12}{'Peak rpm':>11}"
    )
    for rating in ratings:
        print(
            f"{rating.joint:<8}{rating.continuous_output_nm:11.1f}"
            f"{rating.peak_output_nm:11.1f}{rating.reduction_ratio:9.0f}"
            f"{rating.continuous_motor_nm:13.3f}"
            f"{rating.peak_motor_nm:12.3f}"
            f"{rating.measured_peak_motor_rpm:11.0f}"
        )
    print("\nHip motor torque: 0 N m (passive by the manufacturing drawing).")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user-mass", type=float, default=DEFAULT_USER_MASS)
    parser.add_argument("--frame-mass", type=float, default=DEFAULT_FRAME_MASS)
    parser.add_argument(
        "--exo-link-masses",
        type=float,
        nargs=3,
        default=DEFAULT_EXO_LINK_MASSES.tolist(),
        metavar=("THIGH", "SHANK", "FOOT"),
        help="moving frame mass per leg [kg]",
    )
    parser.add_argument(
        "--gait-json",
        type=Path,
        default=Path("ExoWalk_frames/blended.json"),
    )
    parser.add_argument(
        "--cycle-time",
        type=float,
        default=DEFAULT_CYCLE_TIME,
        help="seconds for the full gait JSON cycle",
    )
    parser.add_argument(
        "--cop-distance",
        type=float,
        default=DEFAULT_COP_DISTANCE,
        help="metres forward from ankle",
    )
    parser.add_argument(
        "--support-multiplier-per-leg",
        type=float,
        default=DEFAULT_SUPPORT_MULTIPLIER_PER_LEG,
        help="vertical force divided by combined body/frame weight",
    )
    parser.add_argument(
        "--engineering-factor",
        type=float,
        default=DEFAULT_ENGINEERING_FACTOR,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/torque_analysis"),
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="write CSV, JSON, and PNG result artifacts",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    exo_link_masses = np.asarray(args.exo_link_masses, dtype=float)
    moving_exo_mass = 2.0 * float(np.sum(exo_link_masses))
    if moving_exo_mass > args.frame_mass:
        raise ValueError(
            "Bilateral moving-link allocation exceeds the total frame mass"
        )

    components = build_components(args.user_mass, exo_link_masses)
    validation = validate_model(components)
    time, gait_detail, gait_summary = evaluate_gait(
        args.gait_json,
        args.cycle_time,
        components,
    )
    support_values = evaluate_support_case(
        user_mass=args.user_mass,
        frame_mass=args.frame_mass,
        components=components,
        cop_distance=args.cop_distance,
        support_multiplier_per_leg=args.support_multiplier_per_leg,
    )
    ratings = select_motor_ratings(
        support_values["torque"],
        gait_summary,
        args.engineering_factor,
    )
    print_summary(
        validation,
        gait_summary,
        support_values,
        args.engineering_factor,
        ratings,
    )

    if args.write:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        write_gait_csv(
            args.output_dir / "euler_lagrange_gait_results.csv",
            time,
            gait_detail,
        )
        serialize_results(
            args.output_dir / "euler_lagrange_summary.json",
            user_mass=args.user_mass,
            frame_mass=args.frame_mass,
            exo_link_masses=exo_link_masses,
            validation=validation,
            gait_summary=gait_summary,
            support_values=support_values,
            engineering_factor=args.engineering_factor,
            ratings=ratings,
        )
        plot_gait_torques(
            args.output_dir / "gait_joint_torque.png",
            time,
            gait_detail,
        )
        plot_support_case(
            args.output_dir / "support_motor_sizing.png",
            support_values,
            args.engineering_factor,
            ratings,
        )
        print(f"\nArtifacts written to: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
