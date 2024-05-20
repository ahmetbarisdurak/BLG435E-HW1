import heapq
from .base import SearchAlgorithmBase

class AstarRobotPath(SearchAlgorithmBase):
    def __init__(self) -> None:
        super().__init__()
        self._open_set = []

    def reset(self, grid, start, goal):
        self._grid = grid
        self._start = start
        self._goal = goal
        self._frontier = []
        self._explored = []
        self._path = []
        self._cost = 0
        self._done = False
        heapq.heappush(self._open_set, (self._heuristic(start), 0, start)) # adding h(n), g(n), node
        self.came_from = {start: None}
        
    def _heuristic(self, position):
        
        orientation_cost = 0

        if position != self._goal:
            orientation_cost = 5
        
        distance =  abs(position[0] - self._goal[0]) + abs(position[1] - self._goal[1]) + orientation_cost

        return distance 
    
    def _is_valid(self, pos):
        # Check if the position is valid and within the depth limit
        x, y = pos
        if 0 <= x < len(self._grid) and 0 <= y < len(self._grid[0]):
            if self._grid[x][y] != 1 and pos not in self._explored:  # 1 represents a wall
                # Only consider positions within the current depth limit don't overextend the depth limit
                return True
        return False

    def step(self):
        if not self._open_set:
            self._done = True
            return

        current_f, current_cost, current = heapq.heappop(self._open_set)

        if current == self._goal:
            self._done = True
            self.backtrack_path(current)
            return

        self._explored.append(current)

        x, y = current
        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        
        valid_neighbors = [neighbor for neighbor in neighbors if self._is_valid(neighbor)]

        for neighbor in valid_neighbors:
            if neighbor in self._explored:
                continue

            if neighbor not in [i[2] for i in self._open_set]:  # Not in open set
                heapq.heappush(self._open_set, (self._heuristic(neighbor), 0, neighbor))
                self._frontier.append(neighbor)
            else:
                existing_node = next((x for x in self._open_set if x[2] == neighbor), None)
                if 0 < existing_node[1]:
                    self._open_set.remove(existing_node)
                    heapq.heapify(self._open_set)
                    heapq.heappush(self._open_set, (self._heuristic(neighbor), 0, neighbor))
            self.came_from[neighbor] = current

    def backtrack_path(self, current):
        # Reconstruct the path from goal to start
        current = self._goal
        while current is not None:
            self._path.append(current)
            current = self.came_from[current]
        self._path.reverse()  # Reverse path to start from the beginning
        self._cost = len(self._path)  # Cost is path length