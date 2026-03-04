import numpy as np
import matplotlib.pyplot as plt
import sys

ekf_file = sys.argv[1] if len(sys.argv) > 1 else "data/ekf_output.csv"
ukf_file = sys.argv[2] if len(sys.argv) > 2 else "data/ukf_output.csv"

ekf = np.genfromtxt(ekf_file, delimiter=",", skip_header=1)
ukf = np.genfromtxt(ukf_file, delimiter=",", skip_header=1)

plt.figure(figsize=(10, 8))

# Ground truth
plt.plot(ekf[:, 1], ekf[:, 2], "b-", label="Ground Truth", linewidth=2)

# GPS
plt.scatter(ekf[:, 4], ekf[:, 5], c="r", s=5, alpha=0.3, label="GPS")

# EKF
plt.plot(ekf[:, 6], ekf[:, 7], "g--", label="EKF", linewidth=1.5)

# UKF
plt.plot(ukf[:, 6], ukf[:, 7], "m--", label="UKF", linewidth=1.5)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("EKF vs UKF: Mower Trajectory")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.tight_layout()
plt.savefig("data/ekf_ukf_comparison.png", dpi=150)
plt.show()
