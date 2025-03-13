import pandas as pd
import math
import heapq

class AStarCSV:
    def __init__(self, s_start, s_goal, heuristic_type='euclidean'):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.graph = self.load_csv("Search_based_Planning/Search_2D/graph.csv")

        self.OPEN = []
        self.CLOSED = set()
        self.PARENT = {}
        self.g = {}

    def load_csv(self, path):
        data = pd.read_csv(path)
        graph = {}
        for _, row in data.iterrows():
            orig, dest = row['Origin'], row['Destination']
            graph.setdefault(orig, {})[dest] = {
                'km': row['Distance_km'],
                'litros': row['Fuel_L'],
                'tempo': row['Time_min']
            }
        return graph

    def heuristic(self, s):
        return abs(hash(s) - hash(self.s_goal)) if self.heuristic_type == 'manhattan' else math.sqrt((hash(s) - hash(self.s_goal))**2)

    def cost(self, s_start, s_goal):
        if s_start in self.graph and s_goal in self.graph[s_start]:
            edge = self.graph[s_start][s_goal]
            return 0.5 * edge['km'] + 0.3 * edge['litros'] + 0.2 * edge['tempo']
        return float('inf')

    def searching(self, s_start, s_goal, heuristic_type='euclidean'):
        self.s_start, self.s_goal, self.heuristic_type = s_start, s_goal, heuristic_type

        self.g[s_start] = 0
        heapq.heappush(self.OPEN, (self.heuristic(s_start), s_start))
        self.PARENT[s_start] = None

        while self.OPEN:
            _, current = heapq.heappop(self.OPEN)
            if current == self.s_goal:
                return self.extract_path()

            self.CLOSED.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.CLOSED:
                    continue

                tentative_g = self.g[current] + self.cost(current, neighbor)

                if neighbor not in self.g or tentative_g < self.g[neighbor]:
                    self.g[neighbor] = tentative_g
                    self.PARENT[neighbor] = current
                    heapq.heappush(self.OPEN, (tentative_g + self.heuristic(neighbor), neighbor))

        return []

    def get_neighbors(self, node):
        return list(self.graph[node].keys()) if node in self.graph else []

    def extract_path(self):
        node, path = self.s_goal, []
        while node != self.s_start:
            path.append(node)
            node = self.PARENT[node]
        path.append(self.s_start)
        return path[::-1]

    def calcular_metricas(self, path):
        km, litros, tempo = 0, 0, 0
        for i in range(len(path) - 1):
            edge = self.graph[path[i]][path[i + 1]]
            km += edge['km']
            litros += edge['litros']
            tempo += edge['tempo']
        return km, litros, tempo


if __name__ == '__main__':
    s_start, s_goal = 'P', 'C'
    astar = AStarCSV(s_start, s_goal, heuristic_type="euclidean")

    path = astar.searching(s_start, s_goal)
    if path:
        print("Melhor caminho encontrado:", path)
        km, litros, tempo = astar.calcular_metricas(path)
        print(f"Total km: {km}, Total litros: {litros}, Tempo total: {tempo} min")
    else:
        print("Caminho não encontrado.")
