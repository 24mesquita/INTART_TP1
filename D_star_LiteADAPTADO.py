import pandas as pd
import math
import time

class DStarCSV:
    def __init__(self, s_start, s_goal, heuristic_type="euclidean"):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.graph = self.load_csv("cities_nodes_special.csv")

        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0
        self.history = []  # To store the history of explored nodes

        for node in self.graph:
            self.g[node] = float("inf")
            self.rhs[node] = float("inf")

        self.rhs[s_goal] = 0.0
        self.U[s_goal] = self.calculate_key(s_goal)

    def load_csv(self, path):
        data = pd.read_csv(path)
        graph = {}
        for _, row in data.iterrows():
            orig = row['origin_city']
            dest = row['destination_city']
            if orig not in graph:
                graph[orig] = {}
            graph[orig][dest] = {
                'toll': row['toll'],
                'fuel': row['fuel'],
                'distance_km': row['distance_km']
            }
        return graph

    def cost(self, a, b):
        if a in self.graph and b in self.graph[a]:
            edge = self.graph[a][b]
            return 0.5 * edge['distance_km'] + 0.3 * edge['fuel'] + 0.2 * edge['toll']
        return float('inf')

    def calculate_key(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def h(self, s1, s2):
        return abs(hash(s1) - hash(s2)) if self.heuristic_type == "manhattan" else math.sqrt((hash(s1) - hash(s2)) ** 2)

    def get_neighbors(self, s):
        return list(self.graph[s].keys()) if s in self.graph else []

    def update_vertex(self, u):
        if u != self.s_goal:
            self.rhs[u] = min(self.cost(u, s) + self.g[s] for s in self.get_neighbors(u))
        if u in self.U:
            del self.U[u]
        if self.g[u] != self.rhs[u]:
            self.U[u] = self.calculate_key(u)

    def compute_path(self):
        start_time = time.perf_counter()  # Start timing

        while self.U:
            u = min(self.U, key=self.U.get)
            self.history.append(u)  # Record the current node being processed

            if self.U[u] >= self.calculate_key(self.s_start) and self.rhs[self.s_start] == self.g[self.s_start]:
                break
            k_old = self.U[u]
            del self.U[u]

            if k_old < self.calculate_key(u):
                self.U[u] = self.calculate_key(u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = float("inf")
                self.update_vertex(u)
                for s in self.get_neighbors(u):
                    self.update_vertex(s)

        end_time = time.perf_counter()  # End timing
        self.execution_time = end_time - start_time  # Calculate execution time

    def extract_path(self):
        path = [self.s_start]
        s = self.s_start
        while s != self.s_goal:
            min_cost = float('inf')
            next_s = None
            for neighbor in self.get_neighbors(s):
                c = self.cost(s, neighbor) + self.g[neighbor]
                if c < min_cost:
                    min_cost = c
                    next_s = neighbor
            if next_s is None:
                return []
            path.append(next_s)
            s = next_s
        return path

    def calculate_metrics(self, path):
        toll = fuel = distance_km = 0
        for i in range(len(path) - 1):
            edge = self.graph[path[i]][path[i + 1]]
            toll += edge['toll']
            fuel += edge['fuel']
            distance_km += edge['distance_km']
        return toll, fuel, distance_km

if __name__ == '__main__':
    s_start, s_goal = 'Berlin', 'Madrid'
    dstar_csv = DStarCSV(s_start, s_goal, heuristic_type="euclidean")
    dstar_csv.compute_path()
    path = dstar_csv.extract_path()

    if path:
        print("Best path found:", path)
        toll, fuel, distance_km = dstar_csv.calculate_metrics(path)
        print(f"Total toll: {toll}")
        print(f"Total fuel: {fuel} L")
        print(f"Total distance: {distance_km} km")
    else:
        print("Path not found.")

    print("History of explored nodes:", dstar_csv.history)
    print(f"Execution time: {dstar_csv.execution_time:.4f} seconds")