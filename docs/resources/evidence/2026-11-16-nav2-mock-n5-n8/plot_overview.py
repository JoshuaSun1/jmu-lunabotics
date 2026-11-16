import json
import os
from pathlib import Path
import sys

os.environ["MPLCONFIGDIR"] = "/tmp/lb_mock_matplotlib"
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

source = Path(sys.argv[1])
destination = Path(sys.argv[2])
r = json.loads((source / "results.json").read_text())
plans = json.loads((source / "plans.json").read_text())
detours = [
    item["xy"]
    for item in plans
    if item["xy"] and max(p[1] for p in item["xy"]) > 1.9 and item["xy"][-1][0] > 3.5
]
fig = plt.figure(figsize=(11, 5.5), layout="constrained")
grid = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])
route = fig.add_subplot(grid[:, 0])
memory = fig.add_subplot(grid[0, 1])
cpu = fig.add_subplot(grid[1, 1])
route.add_patch(
    Rectangle(
        (0, 0), 6.85, 5, fill=False, edgecolor="#34495e", lw=2, label="Static arena perimeter"
    )
)
route.add_patch(
    Rectangle((2.3, 1.1), 0.4, 0.8, color="#e67e22", alpha=0.8, label="Synthetic scan obstacle")
)
if detours:
    path = detours[0]
    route.plot(
        [p[0] for p in path], [p[1] for p in path], color="#2471a3", lw=2, label="Replanned path"
    )
    route.scatter(
        [path[0][0], path[-1][0]],
        [path[0][1], path[-1][1]],
        color=["#27ae60", "#8e44ad"],
        s=40,
        zorder=4,
    )
route.set(
    xlim=(-0.2, 7.05),
    ylim=(-0.2, 5.2),
    xlabel="Arena x (m)",
    ylabel="Arena y (m)",
    title="Live replanning around a new obstacle",
)
route.set_aspect("equal")
route.legend(loc="upper right", fontsize=8)
route.grid(alpha=0.2)
samples = r["resource_samples"]
origin = samples[0]["time"]
t = [s["time"] - origin for s in samples]
memory.plot(t, [s["rss_mib"] for s in samples], color="#2471a3")
memory.set(
    xlabel="Elapsed sample time (s)",
    ylabel="Aggregate RSS (MiB)",
    title="Mock launch process memory",
)
memory.grid(alpha=0.2)
usage = [
    max(0.0, (b["cpu_s"] - a["cpu_s"]) / (b["time"] - a["time"]) * 100)
    for a, b in zip(samples, samples[1:])
]
cpu.plot(t[1:], usage, color="#16a085")
cpu.set(
    xlabel="Elapsed sample time (s)",
    ylabel="CPU core-percent",
    title="CPU use — 100% equals one core",
)
cpu.grid(alpha=0.2)
fig.suptitle("N5–N8 mock navigation acceptance | x86_64 / ROS 2 Jazzy", fontsize=13)
fig.savefig(destination, dpi=150)
print(destination)
