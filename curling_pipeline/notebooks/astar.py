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


def manhattan_distance(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)


def euclidian_distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def a_star(start, goal, obstacles, heuristic="manhattan"):
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
                0,
                current_node,
            )
            neighbor_node.heuristic = (
                manhattan_distance(neighbor_node, goal_node)
                if heuristic == "manhattan"
                else euclidian_distance(neighbor_node, goal_node)
            )

            if (next_x, next_y) in node_costs and node_costs[
                (next_x, next_y)
            ] <= neighbor_node.cost:
                continue

            node_costs[(next_x, next_y)] = neighbor_node.cost
            heapq.heappush(open_list, neighbor_node)

    return None


class PathFinder:
    def __init__(
        self,
        grid_width,
        grid_height,
        global_width,
        global_height,
    ):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.global_width = global_width
        self.global_height = global_height

    # Convert coordinates to grid bins
    def to_grid(self, x, y):
        grid_x = int(x / self.global_width * self.grid_width)
        grid_y = int(y / self.global_height * self.grid_height)
        return grid_x, grid_y

    # Convert grid bins back to coordinates
    def to_coordinates(self, grid_x, grid_y):
        x = grid_x / self.grid_width * self.global_width
        y = grid_y / self.grid_height * self.global_height
        return x, y

    def find_path(
        self, start, goal, obstacles, distance_metric="manhattan", debug=False
    ):
        start_grid = self.to_grid(*start)
        goal_grid = self.to_grid(*goal)
        obstacles_grid = {self.to_grid(x, y) for x, y in obstacles}

        if debug:
            ddict = {}
            tick = time.time()
            path_grid = a_star(start_grid, goal_grid, obstacles_grid, distance_metric)
            ddict["elapsed"] = time.time() - tick
            ddict["path"] = path_grid
        else:
            path_grid = a_star(start_grid, goal_grid, obstacles_grid)

        coordinate_path = [self.to_coordinates(x, y) for x, y in path_grid]
        return coordinate_path, ddict if debug else coordinate_path


if __name__ == "__main__":
    # Define the grid size
    grid_width = 100
    grid_height = 60

    start = (3000, 300)
    goal, obstacles = (1000, 500), []

    A = 9000
    B = 6000

    path_finder = PathFinder(grid_width, grid_height, A, B)
    path, debug = path_finder.find_path(start, goal, obstacles, debug=True)

    print("Path:", path)
    print("Time taken:", debug["elapsed"], "seconds")
