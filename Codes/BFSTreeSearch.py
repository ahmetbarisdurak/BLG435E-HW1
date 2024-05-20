from collections import deque
from .base import SearchAlgorithmBase

class BFSTreeSearch(SearchAlgorithmBase):

    def __init__(self) -> None:
        super().__init__()
        
    def reset(self, grid, start, goal):
        # Set initial attributes for BFS
        self._grid = grid
        self._start = start
        self._goal = goal
        self._frontier = deque([start]) # it should be deque since we will pop with an order
        self._explored = [] # Explored order doesn't matter it can be set
        self._path = []
        self._cost = 0
        self._done = False
        self.came_from = {start: None}

    def _is_valid(self, pos):
        #Check if the position is wall or not
        x, y = pos
        if 0 <= x < len(self._grid) and 0 <= y < len(self._grid[0]):
            # check if this position is explored or in frontier. If it is explored don't look again
            if self._grid[x][y] != 1 and pos not in self._frontier:  # 1 represents a wall, 2 and 3 represent start and end point
                return True
        return False

    def step(self):
        if not self._frontier:
            self._done = True
            return 

        current = self._frontier.popleft() # Popping the node which is in order
    
        self._explored.insert(0, current) # Adding the node to explored set

        # If the goal is reached
        if current == self._goal:
            self.backtrack_path()
            self._done = True
            return

        # Fetch neighbors
        x, y = current
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)] # Neighbors are in top bottom right and left

        # List comprehension and creating valid neighbors
        # If neighbor is valid it is added to valid neighbors list
        valid_neighbors = [neighbor for neighbor in neighbors if self._is_valid(neighbor)]
  
        # Add valid neighbors to the frontier
        for neighbor in valid_neighbors:
                self._frontier.append(neighbor)
                if neighbor not in self.came_from:
                    self.came_from[neighbor] = current

    def backtrack_path(self):
        current = self._goal
        while current is not None:
            self._path.append(current)
            current = self.came_from[current]
        self._path.reverse()  # Reverse the path to get it from start to goal
        self._cost = len(self._path)  # Cost is the length of the path minus 1 (start node)
