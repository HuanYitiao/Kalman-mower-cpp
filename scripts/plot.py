import numpy as np
import matplotlib.pyplot as plt
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else "data/ekf_output.csv"
data = np.genfromtxt(filename, delimiter=",", skip_header=1)

true_x = data[:, 1]
true_y = data[:, 2]
gps_x = data[:, 4]
gps_y = data[:, 5]
est_x = data[:, 6]
est_y = data[:, 7]

plt.figure(figsize=(10, 8))
plt.plot(true_x, true_y, "b-", label="Ground Truth", linewidth=2)
plt.scatter(gps_x, gps_y, c="r", s=5, alpha=0.3, label="GPS")
plt.plot(est_x, est_y, "g--", label="EKF Estimate", linewidth=1.5)
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("EKF: Mower Trajectory")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.tight_layout()
plt.savefig("data/ekf_trajectory.png", dpi=150)
plt.show()