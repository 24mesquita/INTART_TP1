import pandas as pd
import math
import heapq
import time

class AStarCSV:
    def __init__(self, s_start, s_goal, heuristic_type='euclidean'):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.graph = self.load_csv("cities_nodes_special.csv")

        self.OPEN = []
        self.CLOSED = set()
        self.PARENT = {}
        self.g = {}
        self.history = []  # To store the history of explored paths

    def load_csv(self, path):
        data = pd.read_csv(path)
        graph = {}
        for _, row in data.iterrows():
            orig, dest = row['origin_city'], row['destination_city']
            graph.setdefault(orig, {})[dest] = {
                'km': row['distance_km'],
                'litros': row['fuel'],
                'portagens': row['toll']
            }
        return graph

    def heuristic(self, s):
        return abs(hash(s) - hash(self.s_goal)) if self.heuristic_type == 'manhattan' else math.sqrt((hash(s) - hash(self.s_goal))**2)

    def cost(self, s_start, s_goal):
        if s_start in self.graph and s_goal in self.graph[s_start]:
            edge = self.graph[s_start][s_goal]
            return 0.3 * edge['km'] + 0.4 * edge['litros'] + 0.3 * edge['portagens']
        return float('inf')

    def searching(self, s_start, s_goal, heuristic_type='euclidean'):
        self.s_start, self.s_goal, self.heuristic_type = s_start, s_goal, heuristic_type

        self.g[s_start] = 0
        heapq.heappush(self.OPEN, (self.heuristic(s_start), s_start))
        self.PARENT[s_start] = None

        start_time = time.perf_counter()  # Start timing

        while self.OPEN:
            _, current = heapq.heappop(self.OPEN)
            self.history.append(current)  # Record the current node being processed

            if current == self.s_goal:
                end_time = time.perf_counter()  # End timing
                self.execution_time = end_time - start_time
                return self.extract_path()

            self.CLOSED.add(current)

            for neighbor in self.get_neighbors(current):
                self.history.append(neighbor)  # Record all neighbors being evaluated

                if neighbor in self.CLOSED:
                    continue

                tentative_g = self.g[current] + self.cost(current, neighbor)

                if neighbor not in self.g or tentative_g < self.g[neighbor]:
                    self.g[neighbor] = tentative_g
                    self.PARENT[neighbor] = current
                    heapq.heappush(self.OPEN, (tentative_g + self.heuristic(neighbor), neighbor))

        end_time = time.perf_counter()  # End timing
        self.execution_time = end_time - start_time
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
        km, litros, portagens = 0, 0, 0
        for i in range(len(path) - 1):
            edge = self.graph[path[i]][path[i + 1]]
            km += edge['km']
            litros += edge['litros']
            portagens += edge['portagens']
        return km, litros, portagens


if __name__ == '__main__':
    s_start, s_goal = 'Berlin', 'Valencia'
    astar = AStarCSV(s_start, s_goal, heuristic_type="euclidean")

    path = astar.searching(s_start, s_goal)
    if path:
        print("Melhor caminho encontrado:", path)
        km, litros, portagens = astar.calcular_metricas(path)
        print(f"Total km: {km}, Total litros: {litros}, Gasto em portagens: {portagens}")
    else:
        print("Caminho não encontrado.")

    print("Histórico de nós explorados:", astar.history)
    print(f"Tempo de execução: {astar.execution_time:.4f} segundos")