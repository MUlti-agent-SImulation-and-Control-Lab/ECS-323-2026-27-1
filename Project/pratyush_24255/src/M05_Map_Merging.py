import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

from M04_Local_Maps import local_maps, UNKNOWN, MAP_SIZE, sense_environment
from M02_Robot_Communication import robots, calculate_distance


COMMUNICATION_THRESHOLD = 4


def merge_maps(robot1, robot2):

    distance = calculate_distance(robot1, robot2)

    print("\nMAP MERGING")

    print(robot1, "<->", robot2)
    print("Distance =", round(distance, 2))

    if distance > COMMUNICATION_THRESHOLD:

        print("Communication : LOW")
        print("Map merging : NOT ALLOWED")

        return

    print("Communication : HIGH")
    print("Map merging : ALLOWED")

    map1 = local_maps[robot1]
    map2 = local_maps[robot2]

    original_map1 = map1.copy()
    original_map2 = map2.copy()

    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):

            value1 = original_map1[y, x]
            value2 = original_map2[y, x]

            if value1 != UNKNOWN and value2 == UNKNOWN:

                map2[y, x] = value1

            elif value2 != UNKNOWN and value1 == UNKNOWN:

                map1[y, x] = value2

    print("Map merging completed.")


def show_merged_maps():

    cmap = ListedColormap([
        "white",       # FREE
        "lightgray",   # UNKNOWN
        "black"        # OBSTACLE
    ])

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    for ax, robot in zip(axes, robots):

        ax.pcolormesh(
            np.arange(MAP_SIZE + 1),
            np.arange(MAP_SIZE + 1),
            local_maps[robot],
            cmap=cmap,
            vmin=0,
            vmax=1,
            edgecolors="black",
            linewidth=0.5
        )

        x, y = robots[robot]

        ax.plot(
            x + 0.5,
            y + 0.5,
            'o',
            markersize=10
        )

        ax.text(
            x + 0.5,
            y + 0.5,
            robot,
            ha="center",
            va="center"
        )

        ax.set_xlim(0, MAP_SIZE)
        ax.set_ylim(0, MAP_SIZE)

        ax.set_xticks(np.arange(0, MAP_SIZE + 1))
        ax.set_yticks(np.arange(0, MAP_SIZE + 1))

        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        ax.set_title(robot + " Local Map")

        ax.set_aspect("equal")

    plt.tight_layout()

    plt.savefig(
        "../results/05_map_merging/map_merging_result.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


# MAIN

if __name__ == "__main__":

    print("ROBOT POSITIONS")

    for robot, position in robots.items():
        print(robot, "=", position)

    print("\nSENSING ENVIRONMENT")

    # Generate local maps for all robots
    for robot in robots:
        sense_environment(robot)

    # Test map merging between communicating robots
    merge_maps("R1", "R2")

    # Test communication-constrained cases
    merge_maps("R1", "R3")
    merge_maps("R2", "R3")

    # Show final local maps
    show_merged_maps()