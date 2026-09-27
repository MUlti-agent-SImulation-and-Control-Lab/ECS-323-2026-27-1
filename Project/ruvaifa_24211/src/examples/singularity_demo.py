"""Demo: Singularity detection and safety guard in Robot Conga controller."""

import math
import sys
from pathlib import Path

# Add project root to sys.path if not installed
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_conga import (
    CongaController,
    ControllerSingularityError,
    Pose,
    ReferenceState,
    Twist,
)


def main() -> None:
    print("=== Robot Conga: Singularity Safety Guard Experiment ===")
    threshold = 1e-3
    controller = CongaController(
        lambda1=4.5,
        lambda2=7.5,
        lambda3=2.5,
        singularity_threshold=threshold,
    )

    print(f"Controller initialized with singularity_threshold = {threshold}")
    print("Testing heading orientations approaching +/- pi/2...")

    # We will test orientations scanning from 85 deg to 90 deg
    test_angles_deg = [85.0, 88.0, 89.0, 89.9, 89.99, 90.0]

    for deg in test_angles_deg:
        theta = math.radians(deg)
        ref_state = ReferenceState(
            pose=Pose(x=0.0, y=0.0, theta=theta),
            twist=Twist(linear=0.1, angular=0.0),
            s=0.0,
            curvature=0.0,
        )
        actual_pose = Pose(x=0.01, y=0.0, theta=theta)
        cos_val = math.cos(theta)

        print(f"\nEvaluating heading theta = {deg:.2f}° (cos = {cos_val:.6e}):")
        try:
            cmd = controller.compute(actual_pose, ref_state)
            print(f"  SAFE: Commanded u = {cmd.linear:.4f} m/s, omega = {cmd.angular:.4f} rad/s")
            assert math.isfinite(cmd.linear) and math.isfinite(cmd.angular), "Output must be finite!"
        except ControllerSingularityError as e:
            print(f"  GUARD TRIGGERED: {e}")
            print("  Successfully prevented numerical instability / NaN / infinity!")

    # Test negative pi/2
    print("\nEvaluating heading theta = -90.0°:")
    neg_pi_2 = -math.pi / 2
    ref_state_neg = ReferenceState(
        pose=Pose(x=0.0, y=0.0, theta=neg_pi_2),
        twist=Twist(linear=0.1, angular=0.0),
        s=0.0,
        curvature=0.0,
    )
    actual_neg = Pose(x=0.01, y=0.0, theta=neg_pi_2)
    try:
        controller.compute(actual_neg, ref_state_neg)
        print("  Unexpectedly completed without error.")
    except ControllerSingularityError as e:
        print(f"  GUARD TRIGGERED: {e}")
        print("  Confirmed: Singularity detected correctly at -pi/2.")

    print("\nSingularity experiment finished successfully!")


if __name__ == "__main__":
    main()
