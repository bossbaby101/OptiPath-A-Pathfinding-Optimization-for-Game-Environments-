import heapq

class Node():
    def __init__(self, state, parent, action, cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

class PriorityQueueFrontier():
    def __init__(self):
        self.frontier = []
        self.state_to_node = {}

    def add(self, node):
        heapq.heappush(self.frontier, node)
        self.state_to_node[node.state] = node

    def contains_state(self, state):
        return state in self.state_to_node

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = heapq.heappop(self.frontier)
            del self.state_to_node[node.state]
            return node

class MarioMaze():
    def __init__(self, maze_layout, warp_pipes):
        self.maze_layout = maze_layout
        self.warp_pipes = warp_pipes
        self.height = len(maze_layout)
        self.width = len(maze_layout[0])

    def heuristic(self, state, goal):
        return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

    def neighbors(self, state):
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1))
        ]

        result = []
        for action, (r, c) in candidates:
            if 0 <= r < self.height and 0 <= c < self.width:
                if self.maze_layout[r][c] != "O":
                    if (r, c) in self.warp_pipes:
                        result.append(("warp", (r, c)))
                    else:
                        result.append((action, (r, c)))
        return result

    def aStarSearch(self, start, goal):
        start_node = Node(start, None, None, 0)
        frontier = PriorityQueueFrontier()
        frontier.add(start_node)
        explored = set()

        while not frontier.empty():
            current_node = frontier.remove()
            current_state = current_node.state

            if current_state == goal:
                path = []
                while current_node.parent is not None:
                    path.append(current_node.action)
                    current_node = current_node.parent
                path.reverse()
                return path

            explored.add(current_state)

            for action, next_state in self.neighbors(current_state):
                if next_state not in explored and not frontier.contains_state(next_state):
                    cost = current_node.cost + 1 if action != "warp" else current_node.cost + 2
                    priority = cost + self.heuristic(next_state, goal)
                    next_node = Node(next_state, current_node, action, cost)
                    frontier.add(next_node)

        raise Exception("No solution found")

def read_maze_from_file(maze.txt):
    with open(filename, "r") as file:
        maze_layout = [line.strip() for line in file.readlines()]
    return maze_layout

# Specify the filename containing the maze layout
maze_filename = "maze.txt"

# Read the maze layout from the file
maze_layout = read_maze_from_file(maze_filename)

# Warp pipe destinations (known in deterministic setting)
warp_pipes = {
    (1, 0): (3, 1),
    (1, 3): (1, 0),
    (3, 1): (1, 3),
    (4, 2): (3, 1)
}

# Input starting and goal points
start = (0, 0)
goal = (4, 4)

# Create MarioMaze instance
mario_maze = MarioMaze(maze_layout, warp_pipes)

# Solve the maze using A* algorithm
print("Maze:")
for row in mario_maze.maze_layout:
    print(row)
print("Finding Shortest Path...")
try:
    shortest_path = mario_maze.aStarSearch(start, goal)
    print("Shortest Path:", shortest_path)
except Exception as e:
    print(e)