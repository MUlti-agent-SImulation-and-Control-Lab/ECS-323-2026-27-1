import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations

from M02_Robot_Communication import robots, calculate_distance

COMMUNICATION_THRESHOLD = 4


def create_graph():

    graph = {}

    # Every robot is a node
    for robot in robots:

        graph[robot] = []

    # Check every pair of robots
    for robot1, robot2 in combinations(robots.keys(), 2):

        distance = calculate_distance(robot1, robot2)

        # If robots are within communication range
        if distance <= COMMUNICATION_THRESHOLD:

            graph[robot1].append(robot2)
            graph[robot2].append(robot1)

    return graph


def show_graph():

    graph = create_graph()

    print(" DYNAMIC COMMUNICATION GRAPH ")

    for robot in graph:

        print(robot, "->", graph[robot])

    # Create graph visualization
    plt.figure(figsize=(7, 5))

    positions = robots

    # Plot robots
    for robot, (x, y) in positions.items():

        plt.scatter(x, y, s=500)

        plt.text(
            x,
            y,
            robot,
            ha="center",
            va="center",
            fontsize=10
        )

    # Draw communication links
    for robot1, robot2 in combinations(robots.keys(), 2):

        if robot2 in graph[robot1]:

            x1, y1 = positions[robot1]
            x2, y2 = positions[robot2]

            plt.plot(
                [x1, x2],
                [y1, y2],
                linewidth=2
            )

    plt.title("Dynamic Communication Graph")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")

    plt.xlim(-1, 10)
    plt.ylim(-1, 10)

    plt.grid(True)
    plt.gca().set_aspect("equal")

    plt.savefig(
        "../results/03_dynamic_graph/dynamic_graph.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


# MAIN

if __name__ == "__main__":

    print(" CURRENT ROBOT POSITIONS ")

    for robot, position in robots.items():

        print(robot, "=", position)

    show_graph()