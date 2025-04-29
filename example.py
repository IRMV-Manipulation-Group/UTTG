import numpy as np
import tg_interface_py as tg
import matplotlib.pyplot as plt

# Define a validity function for the Cartesian space
def is_valid(state):
    # Example validity check: state should be within a unit cube
    return True

# Create an instance of the SE3TrajGeneratorInterface with the validity function
interface = tg.SE3TrajGeneratorInterface(is_valid)

# Define start and goal states as 4x4 transformation matrices
start = np.eye(4)
goal = np.eye(4)
goal[0, 3] = 0.5  # Move goal 0.5 units along the x-axis

# Set the planner
interface.setPlanner("RRTConnect")

# Initialize the interface with waypoints and time limits
waypoints = [start, goal]
time_limits = 1.0
error_code, error_msg = interface.init(waypoints, time_limits)
if error_code != 0:
    print(f"Initialization failed: {error_msg}")
else:
    print("Initialization succeeded")
    # Get the discrete trajectory
    point_nums = 10
    trajectory = interface.getDiscreteTraj(point_nums)
    print("Trajectory:")
    # print(len(trajectory))
    for state in trajectory:
        print(state)