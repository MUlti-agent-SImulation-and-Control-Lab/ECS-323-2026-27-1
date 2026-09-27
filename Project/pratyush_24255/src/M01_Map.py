import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


# OCCUPANCY VALUES

FREE = 0
UNKNOWN = 0.5
OBSTACLE = 1

# 10 × 10 MAP

grid = np.full((10, 10), FREE)

# OBSTACLES 
# Format:(X, Y)

obstacles = [

    (0, 8),
    (0, 9),
    (1, 8),
    (1, 9),

    (9, 9),

    (3, 6),
    (3, 7),

    (6, 4),
    (7, 4),
    (6, 5),
    (7, 5),

    (5, 2),

    (1, 1),
    (2, 1),
    (2, 0),

    (8, 1)
]

for x, y in obstacles:
    grid[y, x] = OBSTACLE


def show_map():

    cmap = ListedColormap([
        "white",       # 0   = FREE
        "lightgray",   # 0.5 = UNKNOWN
        "black"        # 1   = OBSTACLE
    ])

    fig, ax = plt.subplots(figsize=(8, 8))

    # Draw individual cells
    ax.pcolormesh(
        np.arange(11),
        np.arange(11),
        grid,
        cmap=cmap,
        vmin=0,
        vmax=1,
        edgecolors="black",
        linewidth=1
    )

    # ROBOT INITIAL POSITIONS

    robots = {
        "R1": (1, 3),
        "R2": (4, 4),
        "R3": (8, 5)
    }

    for name, (x, y) in robots.items():

        ax.plot(
            x + 0.5,
            y + 0.5,
            'o',
            markersize=12
        )

        ax.text(
            x + 0.5,
            y + 0.5,
            name,
            ha="center",
            va="center",
            fontsize=10
        )

    # AXIS

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)

    ax.set_xticks(np.arange(0, 11))
    ax.set_yticks(np.arange(0, 11))

    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    ax.set_title("10 × 10 Occupancy Grid")

    ax.set_aspect("equal")

    plt.savefig("../results/01_map/map_environment.png", dpi=300, bbox_inches="tight")
    plt.show()

if __name__ == "__main__":
    show_map()