from collections import deque
from .base import SearchAlgorithmBase

class DFSTreeSearch(SearchAlgorithmBase):

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
        # Check if the position is a wall or has been visited
        x, y = pos
        if 0 <= x < len(self._grid) and 0 <= y < len(self._grid[0]):
            if self._grid[x][y] != 1 and pos not in self._frontier:  # 1 represents a wall
                return True
        return False

    def step(self):
        if not self._frontier:
            self._done = True
            return False

        current = self._frontier.popleft()  # Pop from deque, which is the end of the deque

        self._explored.insert(0, current) # Adding the node to explored set


        if current == self._goal:
            self.backtrack_path()
            self._done = True
            return True

        x, y = current
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]  # Neighbors

        # List comprehension and creating valid neighbors
        # If neighbor is valid it is added to valid neighbors list
        valid_neighbors = [neighbor for neighbor in neighbors if self._is_valid(neighbor)]
        
        # Only add unvisited and valid neighbors
        for neighbor in valid_neighbors:
            if neighbor not in self._frontier:
                self._frontier.append(neighbor)  # Add to the deque
                if neighbor not in self.came_from:
                    self.came_from[neighbor] = current



    def backtrack_path(self):
        # Reconstruct the path from goal to start
        current = self._goal
        while current is not None:
            self._path.append(current)
            current = self.came_from[current]
        self._path.reverse()  # Reverse path to start from the beginning
        self._cost = len(self._path)  # Cost is path length minus 1
