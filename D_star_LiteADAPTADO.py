import pandas as pd
import math

class DStarCSV:
    def __init__(self, s_start, s_goal, heuristic_type="euclidean"):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.graph = self.load_csv("Search_based_Planning/Search_2D/graph.csv")

        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0

        for node in self.graph:
            self.g[node] = float("inf")
            self.rhs[node] = float("inf")

        self.rhs[s_goal] = 0.0
        self.U[s_goal] = self.calculate_key(s_goal)

    def load_csv(self, path):
        data = pd.read_csv(path)
        graph = {}
        for _, row in data.iterrows():
            orig = row['Origin']
            dest = row['Destination']
            if orig not in graph:
                graph[orig] = {}
            graph[orig][dest] = {
                'km': row['Distance_km'],
                'litros': row['Fuel_L'],
                'tempo': row['Time_min']
            }
        return graph

    def cost(self, a, b):
        if a in self.graph and b in self.graph[a]:
            edge = self.graph[a][b]
            return 0.5 * edge['km'] + 0.3 * edge['litros'] + 0.2 * edge['tempo']
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
        while self.U:
            u = min(self.U, key=self.U.get)
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
        km = litros = tempo = 0
        for i in range(len(path) - 1):
            edge = self.graph[path[i]][path[i + 1]]
            km += edge['km']
            litros += edge['litros']
            tempo += edge['tempo']
        return km, litros, tempo

if __name__ == '__main__':
    s_start, s_goal = 'P', 'C'
    dstar_csv = DStarCSV(s_start, s_goal, heuristic_type="euclidean")
    dstar_csv.compute_path()
    path = dstar_csv.extract_path()

    if path:
        print("Melhor caminho encontrado:", path)
        km, litros, tempo = dstar_csv.calculate_metrics(path)
        print(f"Distância total: {km} km")
        print(f"Combustível total: {litros} L")
        print(f"Tempo total: {tempo} min")
    else:
        print("Caminho não encontrado.")
