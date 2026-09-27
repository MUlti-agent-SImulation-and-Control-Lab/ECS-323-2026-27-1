import sys
import numpy as np
from itertools import combinations
from M01_Map import grid

# ROBOT POSITIONS(X, Y)

robots = {
    "R1": (1, 3),
    "R2": (4, 4),
    "R3": (8, 5)
}

COMMUNICATION_THRESHOLD = 4

MOVEMENTS = {
    "UP":    (0, 1),
    "DOWN":  (0, -1),
    "LEFT":  (-1, 0),
    "RIGHT": (1, 0)
}

def move_robot(robot_name, direction):

    x, y = robots[robot_name]

    dx, dy = MOVEMENTS[direction]

    new_x = x + dx
    new_y = y + dy

    # Check map boundary
    if new_x < 0 or new_x >= 10:
        print(robot_name, "cannot move outside the map.")
        return

    if new_y < 0 or new_y >= 10:
        print(robot_name, "cannot move outside the map.")
        return

    # Check obstacle
    if grid[new_y, new_x] == 1:
        print(
            robot_name,
            "cannot move to",
            (new_x, new_y),
            "because of obstacle."
        )
        return

    # Move robot
    robots[robot_name] = (new_x, new_y)

    print(
        robot_name,
        "moved",
        direction,
        "→",
        robots[robot_name]
    )

# DISTANCE BETWEEN ROBOTS

def calculate_distance(robot1, robot2):

    x1, y1 = robots[robot1]
    x2, y2 = robots[robot2]

    distance = np.sqrt(
        (x2 - x1)**2 +
        (y2 - y1)**2
    )

    return distance

# COMMUNICATION CHECK

def check_communication():

    print("\n COMMUNICATION ")

    for robot1, robot2 in combinations(robots.keys(), 2):

        distance = calculate_distance(robot1, robot2)

        print(
            f"\n{robot1} ↔ {robot2}"
        )

        print(
            f"Distance = {distance:.2f}"
        )

        # HIGH COMMUNICATION
        if distance <= COMMUNICATION_THRESHOLD:

            print("Communication : HIGH")

            print("Position information : SHARED")
            print("Map information      : SHARED")

        # LOW COMMUNICATION
        else:

            print("Communication : LOW")

            print("Position information : SHARED")
            print("Map information      : NOT SHARED")


if __name__ == "__main__":

    output_file = "../results/02_robot_communication/communication_test.txt"

    class Tee:
        def __init__(self, *files):
            self.files = files

        def write(self, text):
            for file in self.files:
                file.write(text)
                file.flush()

        def flush(self):
            for file in self.files:
                file.flush()

    original_stdout = sys.stdout

    with open(output_file, "w", encoding="utf-8") as file:

        sys.stdout = Tee(original_stdout, file)

        try:

            print("INITIAL POSITIONS")

            for robot, position in robots.items():
                print(robot, "=", position)

            check_communication()

            print("\nROBOT MOVEMENT")

            move_robot("R1", "RIGHT")
            move_robot("R1", "UP")

            move_robot("R2", "LEFT")
            move_robot("R2", "DOWN")

            move_robot("R3", "LEFT")

            check_communication()

        finally:
            sys.stdout = original_stdout