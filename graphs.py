import numpy as np
import matplotlib.pyplot as plt

# =====================================
# RAW DATA
# =====================================

nodes = ["25", "35", "50", "100"]
avg_nodes = np.array([74, 328.2, 1494.5, 6131.1])
inc_nodes = [False, False, False, True]

sizes = ["L", "N", "W"]
avg_sizes = np.array([1673.4, 309.9, 3805.0])
inc_sizes = [False, False, True]

req_len = ["4", "8"]
avg_req = np.array([3424.9, 408.6])
inc_req = [True, False]

# =====================================
# STYLE SETTINGS
# =====================================

plt.rcParams.update({
    "font.size": 9.5,
    "axes.labelsize": 9,
    "axes.titlesize": 11,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "figure.figsize": (4.2, 3.0),
    "axes.spines.right": False,
    "axes.spines.top": False,
    "figure.dpi": 300
})

marker_complete = "o"
marker_incomplete = "^"
color = "black"
size_complete = 38
size_incomplete = 32

def place_point(ax, x, y, incomplete):
    if incomplete:
        ax.scatter(x, y, marker=marker_incomplete, s=size_incomplete,
                   edgecolor=color, facecolor="none", linewidth=1)
    else:
        ax.scatter(x, y, marker=marker_complete, s=size_complete,
                   color=color)

legend_kwargs = dict(
    loc="upper center",
    bbox_to_anchor=(0.52, -0.34),
    frameon=False,
    fontsize=8,
    handletextpad=0.3,
    borderaxespad=0.0
)

# =====================================
# Helper for log-friendly y ticks
# =====================================
def set_human_log_ticks(ax):
    ax.set_yscale("log")
    ax.set_ylim(70, 20000)  # floor below 74
    ax.set_yticks([100, 1000, 10000])
    ax.get_yaxis().set_major_formatter(lambda x, pos: f"{int(x):,}")


# =====================================
# 1) Nodes
# =====================================

fig, ax = plt.subplots()
for x, y, inc in zip(nodes, avg_nodes, inc_nodes):
    place_point(ax, x, y, inc)

ax.plot(nodes, avg_nodes, linewidth=1, color="gray")
ax.set_xlabel("Nodes")
ax.set_ylabel("Avg. Number of Feasible Routes")
ax.set_title("Nodes")

set_human_log_ticks(ax)

ax.scatter([], [], marker=marker_incomplete, s=size_incomplete,
           edgecolor=color, facecolor="none", linewidth=1,
           label="Lower bound due to unfinished w_4_100")

ax.legend(**legend_kwargs)

fig.tight_layout()
fig.savefig("nodes_feasible_routes_log.pdf", bbox_inches="tight")

# =====================================
# 2) Time Window Size
# =====================================

fig, ax = plt.subplots()
for x, y, inc in zip(sizes, avg_sizes, inc_sizes):
    place_point(ax, x, y, inc)

ax.plot(sizes, avg_sizes, linewidth=1, color="gray")
ax.set_xlabel("Time Window Class")
ax.set_ylabel("Avg. Number of Feasible Routes")
ax.set_title("Time Window Class")

set_human_log_ticks(ax)

ax.scatter([], [], marker=marker_incomplete, s=size_incomplete,
           edgecolor=color, facecolor="none", linewidth=1,
           label="Lower bound due to unfinished w_4_100")

ax.legend(**legend_kwargs)

fig.tight_layout()
fig.savefig("size_feasible_routes_log.pdf", bbox_inches="tight")

# =====================================
# 3) Request Length
# =====================================

fig, ax = plt.subplots()
for x, y, inc in zip(req_len, avg_req, inc_req):
    place_point(ax, x, y, inc)

ax.plot(req_len, avg_req, linewidth=1, color="gray")
ax.set_xlabel("Request Length Class")
ax.set_ylabel("Avg. Number of Feasible Routes")
ax.set_title("Request Length Class")

set_human_log_ticks(ax)

ax.scatter([], [], marker=marker_incomplete, s=size_incomplete,
           edgecolor=color, facecolor="none", linewidth=1,
           label="Lower bound due to unfinished w_4_100")

ax.legend(**legend_kwargs)

fig.tight_layout()
fig.savefig("req_feasible_routes_log.pdf", bbox_inches="tight")

plt.show()
