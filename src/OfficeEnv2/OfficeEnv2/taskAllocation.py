'''
    2 types of Nodes
    ->Robot Node
    ->Task Node

    Arrange all the tasks based on distances(distance travelled (d) + (distance between robot and task)d1- priority (p)) from each robot/task
    Choose the robot task pair with least distance
        Add task to Robots dictionary
        Replace task node with Robot node and change Robot distance to d+d1
    
    Recalculate all task distances and choose the robot task pair and so on until u have all tasks assigned to bots
    
    

    DS used:
        ->Dictionary of flood filled grids
        ->Array of distances of robot to task
            -Each robot has the distance travelled stored in it's node, Robot node[(x,y), d]
        ->Each Task Node [(0,1), p]
        ->Dictionary: Robot_num: List of tasks
'''



from collections import deque
import numpy as np
import math
import matplotlib.pyplot as plt
class robotNode:
    def __init__(self,robot_loc):
        self.initial_robot_loc = robot_loc
        self.robot_loc = robot_loc
        self.distance = 0
        self.task_list = []
        self.cumulative_path = []

    def assignTask(self, task_num, distance_travelled, task_loc, path):
        self.distance += distance_travelled
        self.task_list.append(task_num)
        self.robot_loc = task_loc
        self.cumulative_path.append(path)
        return self.robot_loc

class taskNode:
    def __init__(self,task_location):
        self.task_loc = task_location
        self.priority = 0 
        self.alloted = False
        self.done = False
    
    def assignTask(self):
        self.alloted = True

    def taskAborted(self,distance_covered):
        self.alloted = False
        self.priority -= distance_covered
    
    def taskDone(self):
        self.done = True

def flood_fill(grid, start_location):
    rows = len(grid)
    cols = len(grid[0])
    start_x, start_y = start_location  # Unpack the start location

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    distance_grid = [[-1 for _ in range(cols)] for _ in range(rows)]

    if grid[start_x][start_y] != 0:
        return distance_grid 
    
    queue = deque([(start_x, start_y)])
    distance_grid[start_x][start_y] = 0

    while queue:
        x, y = queue.popleft()

        # Current distance level
        current_level = distance_grid[x][y]

        # Explore all 8 neighbors
        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0 and distance_grid[nx][ny] == -1:
                distance_grid[nx][ny] = current_level + 1  # Set the level for the neighbor
                queue.append((nx, ny))  # Add the neighbor to the queue

    return distance_grid


def frames(grids):
    avg = 0.5 #Change this
    grids = np.array(grids) 
    average = np.mean(grids, axis=0)
    grid = (average >= avg).astype(int)

    return grid

def findPath(bot_loc,grid):
    x,y= bot_loc
    level = grid[x][y]
    path = [(x, y)]
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while level!=0:
        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            if nx>=0 and nx<len(grid) and ny>=0 and ny<len(grid[0]):
                if (grid[nx][ny]==level-1):
                    x = nx
                    y = ny
                    path.append((x,y))
                    level = level - 1
                    break
    return path

def euclidean_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def total_distance_traveled(path):
    return sum(euclidean_distance(path[i], path[i+1]) for i in range(len(path) - 1))

def assign_tasks(robots, tasks, distances_grid, filled_grids):
    assigned_tasks = set()

    while len(assigned_tasks) < len(tasks):
        min_distance = float('inf')
        chosen_robot_id = None
        chosen_task_id = None

        # Find the robot-task pair with the least distance
        for task_id, task in tasks.items():
            if task_id in assigned_tasks or task.alloted or task.done:
                continue
            for robot_id, robot in robots.items():
                distance = robot.distance + distances_grid[task_id - 1][robot_id - 1] - task.priority
                if distance < min_distance:
                    min_distance = distance
                    chosen_robot_id = robot_id
                    chosen_task_id = task_id

        if chosen_robot_id is None or chosen_task_id is None:
            break  # No valid robot-task pair found

        # Assign the task to the chosen robot
        robot = robots[chosen_robot_id]
        task = tasks[chosen_task_id]

        task_loc = task.task_loc
        task.assignTask()
        robot.assignTask(chosen_task_id, min_distance, task_loc,findPath(robot.robot_loc, filled_grids[chosen_task_id-1]))
        assigned_tasks.add(chosen_task_id)
        for task_id, task in tasks.items():
            distances_grid[task_id-1][chosen_robot_id-1] = total_distance_traveled(findPath(robot.robot_loc, filled_grids[task_id-1]))


def plot_all_paths(grid, robots, tasks):
    plt.figure(figsize=(8,8))
    plt.imshow(grid, cmap='gray_r', origin='upper')  # Set origin='upper' to match coordinate system
    plt.title('Robot Paths and Task Locations')

    # Track the task sequence for each robot
    task_sequence = {}

    # Plot tasks in yellow
    for task_id, task in tasks.items():
        x, y = task.task_loc
        plt.scatter(y, x, c='yellow', s=200, label=f"Task {task_id}" if task_id == 1 else None, edgecolors='black', zorder=5)
        plt.text(y + 0.2, x - 0.2, f"Task {task_id}", color='red', fontsize=8, weight='bold', zorder=10)

    # Plot all robots in green
    for robot_id, robot in robots.items():
        task_sequence[robot_id] = []

        # Plot the robot's path in blue
        complete_path = []
        for task_path in robot.cumulative_path:
            complete_path.extend(task_path)

        complete_path = np.array(complete_path)

        # Plot the robot's start position
        start_x, start_y = robot.initial_robot_loc
        plt.scatter(start_y, start_x, c='green', s=300, label=f"Robot {robot_id} Start" if robot_id == 1 else None, edgecolors='black', zorder=5)
        plt.text(start_y - 0.2, start_x + 0.2, f"Robot{robot_id}", color='orange', fontsize=8, weight='bold', zorder=10)

        # Plot the robot's path with directed edges
        for i in range(len(complete_path) - 1):
            x1, y1 = complete_path[i]
            x2, y2 = complete_path[i + 1]

            # Plot the line (edge) between points
            plt.plot([y1, y2], [x1, x2], color='blue', linewidth=2, zorder=2)

            # Add arrows to indicate direction
            plt.arrow(y1, x1, y2 - y1, x2 - x1, head_width=0.2, head_length=0.3, fc='blue', ec='blue', zorder=3)

            # Track the task sequence
            for task_id in robot.task_list:
                task_sequence[robot_id].append(f"T{task_id}")

    # Add gridlines, customize them for better visibility
    plt.grid(True, which='both', linestyle='--', linewidth=0.5, color='black', zorder=1)

    # Invert Y axis to match grid coordinates
    plt.gca().invert_yaxis()

    # Display the plot
    plt.show()


def create_maze(grid, robots, tasks, density=0.2):
    rows, cols = grid.shape
    num_objects = int(density * rows * cols)
    
    # Collect positions to exclude (robot and task positions)
    occupied_positions = set()
    for robotID, robot in robots.items():
        occupied_positions.add(tuple(robot.initial_robot_loc))
    for taskID, task in tasks.items():
        occupied_positions.add(tuple(task.task_loc))

    for _ in range(num_objects):
        while True:
            x, y = np.random.randint(0, rows - 1), np.random.randint(0, cols - 1)
            if grid[x, y] == 0 and (x, y) not in occupied_positions:
                grid[x, y] = 1
                break

grids = [[
    [1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0],
    [0,0,0,0,0,0,0,1,1,0,1,0,1,0,0,0,0,1,1,0,1,0,0,0,1,1,0,0,0,0],
    [0,1,1,0,0,0,0,0,0,1,0,1,1,0,0,0,0,1,0,0,0,0,0,0,1,0,0,1,1,0],
    [0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,1,0,0,1,1,0,1,1,0],
    [0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0],
    [1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,0,0,1,0,1,0,0,0,0,0,0],
    [0,0,0,1,0,1,0,0,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,1,0,0,0,0,1,1,0,0,0,1,0,0,0,1,1,1,0,0,1,0,0,0,0,1,0,0],
    [0,1,0,0,1,0,0,0,0,0,1,1,0,1,1,1,0,1,0,0,0,1,0,1,1,0,1,1,0,0],
    [1,0,1,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0],
    [1,0,1,0,0,1,0,0,0,1,0,1,0,1,1,1,1,0,0,0,1,1,0,1,0,0,0,0,1,0],
    [0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0],
    [0,0,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,1,1,0,0],
    [1,0,0,1,0,1,0,1,0,1,1,1,1,1,0,1,0,0,0,0,0,0,0,1,1,1,0,0,1,0],
    [0,0,0,0,0,1,0,1,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
    [1,0,1,1,1,0,0,0,0,1,0,0,1,0,0,0,0,1,0,0,0,1,0,0,1,1,0,1,0,0],
    [0,0,0,1,0,0,1,0,1,0,1,1,0,1,1,0,0,1,1,0,0,0,1,1,0,0,1,0,0,0],
    [0,1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,1,1,0,1,0,0,0,0,1,0,0,0,1,0],
    [0,0,0,0,1,1,0,0,0,0,1,1,1,0,0,1,1,1,1,1,0,1,0,0,0,1,1,1,0,0],
    [1,0,1,0,0,0,0,0,1,0,1,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0],
    [0,0,0,1,1,1,0,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,0,1,0,0,1,0,0,0],
    [0,0,0,1,1,0,0,1,1,1,0,1,1,0,0,0,1,1,0,0,0,0,0,0,0,0,1,1,1,0],
    [1,0,0,1,1,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,0,0,0],
    [0,1,0,0,1,1,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,0,0],
    [0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,1,1,0,0,0,1,1,0,0],
    [1,0,0,0,1,0,0,0,1,0,0,1,1,0,0,0,0,0,0,0,0,1,0,1,1,0,0,1,0,0],
    [0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1,1,0],
    [1,0,0,0,0,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]
]
grid = frames(grids)
robots = {
    1:robotNode((27,7)),
2:robotNode((3,2)),
3:robotNode((28,3)),
4:robotNode((29,9)),
5:robotNode((15,13)),
6:robotNode((17,2)),
7:robotNode((8,0)),
8:robotNode((5,7)),
9:robotNode((14,4)),
10:robotNode((20,22))
}
tasks = {
    1:taskNode((28,12)),
2:taskNode((7,29)),
3:taskNode((13,28)),
4:taskNode((22,22)),
5:taskNode((9,7)),
6:taskNode((27,24)),
7:taskNode((13,20)),
8:taskNode((15,0)),
9:taskNode((6,16)),
10:taskNode((15,3)),
11:taskNode((5,7)),
12:taskNode((20,7)),
13:taskNode((29,15)),
14:taskNode((25,10)),
15:taskNode((2,26)),
16:taskNode((11,17)),
17:taskNode((7,29)),
18:taskNode((2,7)),
19:taskNode((3,26)),
20:taskNode((1,0))
}
filled_grids = []
distances_grid = []
paths_grid = []

for task_id, task in tasks.items():
    filled_grid = flood_fill(grid, task.task_loc)
    filled_grids.append(filled_grid)
    distances = []
    paths = []
    for bot_id, bot in robots.items():
        path = findPath(bot.robot_loc, filled_grid)
        paths.append(path)
        distance = total_distance_traveled(path)
        distances.append(distance)
    distances_grid.append(distances)
    paths_grid.append(paths)

assign_tasks(robots, tasks, distances_grid, filled_grids)

plot_all_paths(grid, robots, tasks)