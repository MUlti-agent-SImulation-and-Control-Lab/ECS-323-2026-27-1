from M02_Robot_Communication import robots, move_robot
from M04_Local_Maps import sense_environment
from M03_Dynamic_Graph import show_graph


def move_and_sense(robot_name, direction):

    print("\n========================================")
    print(robot_name, "moving", direction)
    print("========================================")

    # Move robot
    move_robot(robot_name, direction)

    # Sense environment after movement
    sense_environment(robot_name)

    # Update/display communication graph
    show_graph()


if __name__ == "__main__":

    print(" INITIAL POSITIONS ")

    for robot, position in robots.items():
        print(robot, "=", position)

    print("\n INITIAL SENSING ")

    for robot in robots:
        sense_environment(robot)

    show_graph()

    # Test movement
    move_and_sense("R1", "RIGHT")

    move_and_sense("R1", "RIGHT")

    move_and_sense("R1", "UP")