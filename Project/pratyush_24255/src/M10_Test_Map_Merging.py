from M02_Robot_Communication import robots
from M04_Local_Maps import sense_environment, show_local_map
from M05_Map_Merging import merge_maps


print(" ROBOT POSITIONS ")

for robot, position in robots.items():
    print(robot, "=", position)


print("\n INITIAL SENSING ")

for robot in robots:
    sense_environment(robot)


print("\n MAPS BEFORE MERGING ")

show_local_map("R1")
show_local_map("R2")


print("\n MERGING R1 AND R2 ")

merge_maps("R1", "R2")


print("\n MAPS AFTER MERGING ")

show_local_map("R1")
show_local_map("R2")