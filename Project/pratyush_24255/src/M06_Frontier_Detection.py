from M04_Local_Maps import (
    local_maps,
    UNKNOWN,
    FREE,
    MAP_SIZE,
    sense_environment
)

from M02_Robot_Communication import robots

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


def find_frontiers(robot_name):

    local_map = local_maps[robot_name]

    frontiers = []

    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):

            # A frontier must be an UNKNOWN cell
            if local_map[y, x] != UNKNOWN:
                continue

            # Check the four neighboring cells
            neighbors = [
                (x, y + 1),   # UP
                (x, y - 1),   # DOWN
                (x - 1, y),   # LEFT
                (x + 1, y)    # RIGHT
            ]

            for nx, ny in neighbors:

                # Ignore cells outside the map
                if nx < 0 or nx >= MAP_SIZE:
                    continue

                if ny < 0 or ny >= MAP_SIZE:
                    continue

                # UNKNOWN cell adjacent to FREE cell
                if local_map[ny, nx] == FREE:

                    frontiers.append((x, y))
                    break

    return frontiers


def show_frontiers(robot_name):

    local_map = local_maps[robot_name]

    frontiers = find_frontiers(robot_name)

    cmap = ListedColormap([
        "white",       # FREE
        "lightgray",   # UNKNOWN
        "black"        # OBSTACLE
    ])

    fig, ax = plt.subplots(figsize=(8, 8))

    # Show local map
    ax.pcolormesh(
        range(MAP_SIZE + 1),
        range(MAP_SIZE + 1),
        local_map,
        cmap=cmap,
        vmin=0,
        vmax=1,
        edgecolors="black",
        linewidth=1
    )

    # Plot frontier cells
    for x, y in frontiers:

        ax.plot(
            x + 0.5,
            y + 0.5,
            'o',
            markersize=10,
            markerfacecolor="yellow",
            markeredgecolor="red"
        )

        ax.text(
            x + 0.5,
            y + 0.5,
            "F",
            ha="center",
            va="center",
            fontsize=9
        )

    # Plot robot position
    x, y = robots[robot_name]

    ax.plot(
        x + 0.5,
        y + 0.5,
        'o',
        markersize=14
    )

    ax.text(
        x + 0.5,
        y + 0.5,
        robot_name,
        ha="center",
        va="center",
        fontsize=10
    )

    ax.set_xlim(0, MAP_SIZE)
    ax.set_ylim(0, MAP_SIZE)

    ax.set_xticks(range(MAP_SIZE + 1))
    ax.set_yticks(range(MAP_SIZE + 1))

    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    ax.set_title(
        f"{robot_name} Local Map with Frontier Cells"
    )

    ax.set_aspect("equal")

    plt.savefig(
        f"../results/06_frontier_detection/{robot_name}_frontiers.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


# MAIN

if __name__ == "__main__":

    print("FRONTIER DETECTION")

    print("\nROBOT POSITIONS")

    for robot, position in robots.items():
        print(robot, "=", position)

    # STEP 1: Sense the environment

    print("\nSENSING ENVIRONMENT")

    for robot in robots:

        print("\n" + robot + " sensing environment...")

        sense_environment(robot)

    # STEP 2: Detect frontiers

    print("\n\nFRONTIER RESULTS")

    for robot in robots:

        print("\n" + robot + " frontiers:")

        frontiers = find_frontiers(robot)

        print("Number of frontiers =", len(frontiers))

        print("Frontier coordinates:")

        for frontier in frontiers:
            print(frontier)

        # Show and save visualization
        show_frontiers(robot)