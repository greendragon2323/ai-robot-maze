# sys, select, time, random,
import queue
import copy
import re


def l2(xi, yi, xf, yf):
    return ((xi - xf)**2 + (yi - yf)**2)**0.5


def l1(xi, yi, xf, yf):
    return (abs(xi - xf) + abs(yi - yf))


class Robot:

    def __init__(self, x, y):
        # store robot position
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f"robot pos: ({self.x},{self.y})"

    def __hash__(self):
        h = 0
        h *= 3
        h ^= hash(self.robot.x)
        h *= 3
        h ^= hash(self.robot.y)
        return h

    def move(self, x, y):
        self.x = x
        self.y = y

    def update_pos(self, x, y):
        self.move(x, y)

    def get_pos(self):
        return self.x, self.y

    def actions(self, walls):
        a = frozenset()
        x, y = int(self.x), int(self.y)

        # down
        if ((x, y+1, x+1, y+1) not in walls):
            a |= {"D"}
        # right
        if (x+1, y, x+1, y+1) not in walls:
            a |= {"R"}
        # up
        if (x, y, x+1, y) not in walls:
            a |= {"U"}
        # left
        if ((x, y, x, y+1) not in walls):
            a |= {"L"}

        return a


class Maze:

    def __init__(self, x, y, goal_x, goal_y, walls) -> None:
        self.robot = Robot(x, y)

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.h = l1(self.robot.x, self.robot.y, goal_x, goal_y)

        # store wall positions
        self.walls = copy.deepcopy(walls)

    def __lt__(self, other):  # TODO: nto sure if this is correct
        return self.h < other.h

    def __eq__(self, other):
        if self.goal_x != other.goal_x or self.goal_y != other.goal_y:
            return False

        return self.robot == other.robot  # TODO: should compare walls?

    def __hash__(self):
        h = 0
        for wall in self.walls:   # TODO: was commented out
            h *= 3
            h ^= hash(wall[0])
            h *= 3
            h ^= hash(wall[1])

        h *= 3
        h ^= hash(self.robot.x)
        h *= 3
        h ^= hash(self.robot.y)

        h *= 3
        h ^= hash(self.goal_x)
        h *= 3
        h ^= hash(self.goal_y)

        return h

    def get_robo_pos(self):
        return (self.robot.x, self.robot.y)

    def update_walls(self, new_walls):
        self.walls = copy.deepcopy(new_walls).union(self.walls)

    def at_goal(self):
        if self.robot.x == self.goal_x and self.robot.y == self.goal_y:
            return True
        else:
            return False

    # return new maze resulting from action
    def step(self, action):
        assert (action is not None)

        new_maze = copy.deepcopy(self)
        x, y = new_maze.robot.x, new_maze.robot.y

        # move down
        if action == "D":
            new_maze.robot.move(x, y+1)
        # move right
        elif action == "R":
            new_maze.robot.move(x+1, y)
        # move up
        elif action == "U":
            new_maze.robot.move(x, y-1)
        # move left
        elif action == "L":
            new_maze.robot.move(x-1, y)

        return new_maze

    def actions(self):
        return self.robot.actions(self.walls)


class State:

    def __init__(self, maze, moves, pathcost):
        self.maze = maze
        self.moves = moves
        self.path_cost = pathcost

    def __eq__(self, other):
        return self.maze == other.maze

    def __hash__(self):
        h = 0
        for i in range(len(self.moves)):
            h *= 3
            h ^= hash(self.moves[i][0])
            h *= 3
            h ^= hash(self.moves[i][1])
        return hash(self.maze)

    def __lt__(self, other):
        return self.eval() < other.eval()

    def get_moves(self):
        return self.moves

    def step(self, act):
        x, y = self.maze.robot.x, self.maze.robot.y

        # move down
        if act == "D":
            move = (x, y+1)
        # move right
        elif act == "R":
            move = (x+1, y)
        # move up
        elif act == "U":
            move = (x, y-1)
        # move left
        elif act == "L":
            move = (x-1, y)

        return State(self.maze.step(act), self.moves+[move], self.path_cost+1)

    def successors(self):
        successors_set = frozenset({})
        actions = self.maze.actions()  # get possible actions

        # create a new successor state for every possible action
        for act in actions:
            successors_set |= {self.step(act)}
        return successors_set

    def at_goal(self):
        # goal is specifieed in maze intialized with
        return self.maze.at_goal()

    def eval(self):

        x1, y1 = self.maze.get_robo_pos()
        x2 = self.maze.goal_x
        y2 = self.maze.goal_y

        heu = l1(x1, y1, x2, y2)

        return self.path_cost + heu


class ASTAR:

    def __init__(self, start_maze):

        self.frontier = queue.PriorityQueue()
        self.astar_seen = set()

        # shifts robot position to center of tile for astar calculation
        x, y = start_maze.robot.get_pos()
        start_maze.robot.update_pos(int(x)+0.5, int(y)+0.5)

        # startx, starty, goalx, goaly included in starting board
        self.start = State(start_maze, [], 0)

        self.frontier.put(self.start)

        self.goal_state = None

    def __str__(self) -> str:

        if self.goal_state is None:
            return "An end state was either not found or evaluated"

        moves = list(self.goal_state.moves)

        move_string = ''
        for move in moves:
            entry = ''.join(('(', str(move[0]), ',', str(move[1]), ') '))
            move_string = ''.join([move_string, entry])

        return move_string

    def run(self):
        # init takes care of intial setup

        while self.frontier:
            if self.frontier.empty():
                return None
            state = self.frontier.get()

            if state.at_goal():
                self.goal_state = state
                # what should normally be returned
                return state.get_moves()

            sucessors = state.successors()      # find sucessors
            # add unseen sucessors to frontier
            for suc in sucessors:
                # don't add seen nodes to frontier
                if state in self.astar_seen:
                    continue
                self.frontier.put(suc)

            self.astar_seen.add(state)

        return None  # only returns None if gets stuck and no solution


def load_walls(filename):

    # set of known walls
    maze_walls = set()
    for i in range(0, 11):
        maze_walls |= {(i, 0, i+1, 0), (i, 11, i+1, 11),
                       (0, i, 0, i+1), (11, i, 11, i+1)}

    # load walls from maze
    pattern = re.compile(r"\w\w\w\w (?P<x1>\d+) (?P<y1>\d+) (?P<x2>\d+) (?P<y2>\d+)")
    with open(filename) as maze_in:
        for line in maze_in:
            line = line.rstrip('\r\n')

            m = pattern.search(line)
            x1, y1, x2, y2 = m.group('x1'), m.group('y1'), m.group('x2'), m.group('y2')

            maze_walls |= {(int(x1), int(y1), int(x2), int(y2))}
    return maze_walls


# helper functions
def print_cmd_line(obs, bot=True, walls=True, ebwb=False, empty=False):

    # see only bot
    if len(obs) <= 1:  # skips empty outputs by checking len(obs) > 1
        pass
    # print bot
    elif bot is True and obs[0:3] == 'bot':
        print(f"comment bot == {obs}", flush=True)
    # print walls
    elif walls is True and obs[0:4] == 'wall':
        print(f"comment bot == {obs}", flush=True)
    # print everything else
    elif ebwb is True and len(obs) > 1 and obs[0:4] != 'wall' and obs[0:3] != 'bot':

        print(f"comment bot == {obs}", flush=True)  
