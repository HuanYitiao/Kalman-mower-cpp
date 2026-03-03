import pandas as pd
import matplotlib.pyplot as plt
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else "data/output.csv"
df = pd.read_csv(filename)

plt.figure(figsize=(10, 8))
plt.plot(df["true_x"], df["true_y"], "b-", label="Ground Truth", linewidth=2)
plt.scatter(df["gps_x"], df["gps_y"], c="r", s=5, alpha=0.3, label="GPS")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Mower Trajectory")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.tight_layout()
plt.savefig("data/trajectory.png", dpi=150)
plt.show()