from collections import deque
from .base import SearchAlgorithmBase

class IDSGraphSearch(SearchAlgorithmBase):

    def reset(self, grid, start, goal): # Reset the grid firstly
        # Set initial attributes for BFS
        self._grid = grid
        self._start = start
        self._goal = goal
        self._frontier = [start] # it should be stack since it should be LIFO
        self._explored = set() # Explored order doesn't matter it can be set
        self._path = []
        self._cost = 0
        self._done = False
        self.came_from = {start: None}

    def depth(self, node):
        # A simple cycle-check: If any ancestor state matches the current state, return True
        depth = 0
        while node is not None:
            depth = depth + 1
            node = self.came_from[node]
        return depth - 1

    def _is_valid(self, pos, depth_limit):
        # Check if the position is valid and within the depth limit
        x, y = pos
        if 0 <= x < len(self._grid) and 0 <= y < len(self._grid[0]):
            if self._grid[x][y] != 1 and pos not in self._explored:  # 1 represents a wall
                # Only consider positions within the current depth limit don't overextend the depth limit
                return True
        return False

    def is_cycle(self, node):
    # A simple cycle-check: If any ancestor state matches the current state, return True
        if node not in self.came_from:
            return False 
        
        current = self.came_from[node]
        while current:
            if node == current:
                return True
            current = self.came_from[current]
        return False    

    # Depth Limited Search is on its own
    def depth_limited_search(self, depth_limit):
        #self._frontier.append(self._start) # Actually i am appending the start in the reset function
        result = False

        while self._frontier:

            current = self._frontier.pop()
            self._explored.add(current)

            if(self._goal == current):
                return current
            
            if self.depth(current) > depth_limit:  # Only expand nodes within the depth limit
                return "cutoff"
            
            elif not self.is_cycle(current):
                    x, y = current
                    neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
                    
                    valid_neighbors = [neighbor for neighbor in neighbors if self._is_valid(neighbor, depth_limit)]

                    for neighbor in valid_neighbors:
                        self._frontier.append(neighbor)
                        self.came_from[neighbor] = current
        return result
                        
    # Iterative Deepening Search inside step function
    def step(self):
        depth_limit = 0

        while True:
            self._explored.clear()

            self.came_from.clear()
            self.came_from[self._start] = None # parent nodes
            
            self._frontier.clear()
            self._frontier.append(self._start)

            result = self.depth_limited_search(depth_limit)

            if result != "cutoff": # If the node has been found stop the iterative process
                self.backtrack_path() 
                self._done = True
                break
            else:
                # If node hasn't been found increase the depth limit for the next iteration
                depth_limit += 1

    def backtrack_path(self):
        # Reconstruct the path from goal to start
        current = self._goal
        while current is not None:
            self._path.append(current)
            current = self.came_from[current]
        self._path.reverse()  # Reverse path to start from the beginning
        self._cost = len(self._path)  # Cost is path length minus 1
