import heapq
import time


class Node:
    def __init__(self, x, y, cost=0, heuristic=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)


def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)


def a_star(start, goal, obstacles):
    open_list = []
    closed_list = set()
    start_node = Node(*start)
    goal_node = Node(*goal)
    heapq.heappush(open_list, start_node)
    node_costs = {(start_node.x, start_node.y): start_node.cost}

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add((current_node.x, current_node.y))

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        neighbors = [
            (current_node.x - 1, current_node.y),
            (current_node.x + 1, current_node.y),
            (current_node.x, current_node.y - 1),
            (current_node.x, current_node.y + 1),
        ]

        for next_x, next_y in neighbors:
            if (next_x, next_y) in obstacles or (next_x, next_y) in closed_list:
                continue

            neighbor_node = Node(
                next_x,
                next_y,
                current_node.cost + 1,
                heuristic(Node(next_x, next_y), goal_node),
                current_node,
            )

            if (next_x, next_y) in node_costs and node_costs[
                (next_x, next_y)
            ] <= neighbor_node.cost:
                continue

            node_costs[(next_x, next_y)] = neighbor_node.cost
            heapq.heappush(open_list, neighbor_node)

    return None


# Convert coordinates to grid bins
def to_grid(x, y, width, height, grid_width, grid_height):
    grid_x = int((x + width / 2) / width * grid_width)
    grid_y = int((y + height / 2) / height * grid_height)
    return grid_x, grid_y


# Convert grid bins back to coordinates
def to_coordinates(grid_x, grid_y, width, height, grid_width, grid_height):
    x = grid_x / grid_width * width - width / 2
    y = grid_y / grid_height * height - height / 2
    return x, y


def select_duck(duck_locations, idx):
    return duck_locations[idx], duck_locations[:idx] + duck_locations[idx + 1 :]


# Define the grid size
grid_width = 100
grid_height = 60

start = (3000, 300)
goal, obstacles = select_duck(1)

A = 9000
B = 6000

# Convert start, goal, and obstacles to grid bins
start_grid = to_grid(*start, A, B, grid_width, grid_height)
goal_grid = to_grid(*goal, A, B, grid_width, grid_height)
obstacles_grid = {to_grid(x, y, A, B, grid_width, grid_height) for x, y in obstacles}

# Measure the time taken to find the path
start_time = time.time()
path_grid = a_star(start_grid, goal_grid, obstacles_grid)
end_time = time.time()

# Convert the path back to coordinates
path_coordinates = [
    to_coordinates(x, y, A, B, grid_width, grid_height) for x, y in path_grid
]

# Print the path and time taken
print("Path:", path_coordinates)
print("Time taken:", end_time - start_time, "seconds")