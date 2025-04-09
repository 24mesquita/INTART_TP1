import math
import pandas as pd
import time
from queue import PriorityQueue

class LrtAStarN:
    def __init__(self, s_start, s_goal, N, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.N = N
        self.grafo = self.ler_csv("cities_nodes_special.csv")
        self.h_table = {}
        self.visited = []
        self.path = []
        self.history = []  # To store the history of explored nodes
        self.execution_time = 0  # To store the execution time

    def ler_csv(self, caminho):
        dados = pd.read_csv(caminho)
        grafo = {}
        for _, linha in dados.iterrows():
            origem = linha['origin_city']
            destino = linha['destination_city']
            distancia = linha['distance_km']
            combustivel = linha['fuel']
            portagens = linha['toll']

            if origem not in grafo:
                grafo[origem] = {}
            grafo[origem][destino] = {'km': distancia, 'litros': combustivel, 'portagens': portagens}
        return grafo

    def init(self):
        for no in self.grafo:
            self.h_table[no] = self.h(no)

    def h(self, s):
        if self.heuristic_type == "manhattan":
            return abs(hash(self.s_goal) - hash(s))
        else:
            return math.sqrt((hash(self.s_goal) - hash(s)) ** 2)  # Euclidean approximation

    def cost(self, s_start, s_goal):
        if s_start not in self.grafo or s_goal not in self.grafo[s_start]:
            return float("inf")
        data = self.grafo[s_start][s_goal]
        return 0.3 * data['km'] + 0.4 * data['litros'] + 0.3 * data['portagens']

    def get_neighbor(self, s):
        return list(self.grafo[s].keys()) if s in self.grafo else []

    def searching(self):
        self.init()
        s_start = self.s_start

        start_time = time.perf_counter()  # Start timing

        while True:
            result, CLOSED = self.AStar(s_start, self.N)
            if result == "FOUND":
                self.path = self.extract_path(s_start, CLOSED)
                break

            h_value = self.iteration(CLOSED)
            for x in h_value:
                self.h_table[x] = h_value[x]

            s_start, path_k = self.extract_path_in_CLOSE(s_start, h_value)
            self.path = path_k

        end_time = time.perf_counter()  # End timing
        self.execution_time = end_time - start_time  # Calculate execution time

    def AStar(self, x_start, N):
        OPEN = PriorityQueue()
        OPEN.put((self.h(x_start), x_start))
        CLOSED = []
        g_table = {x_start: 0}
        PARENT = {x_start: x_start}
        count = 0

        while not OPEN.empty():
            count += 1
            _, s = OPEN.get()
            CLOSED.append(s)
            self.history.append(s)  # Record the current node being processed

            if s == self.s_goal:
                return "FOUND", PARENT

            for s_n in self.get_neighbor(s):
                self.history.append(s_n)  # Record all neighbors being evaluated
                new_cost = g_table[s] + self.cost(s, s_n)
                if s_n not in g_table or new_cost < g_table[s_n]:
                    g_table[s_n] = new_cost
                    PARENT[s_n] = s
                    OPEN.put((new_cost + self.h_table[s_n], s_n))

            if count == N:
                break

        return OPEN, CLOSED

    def extract_path_in_CLOSE(self, s_start, h_value):
        path = [s_start]
        s = s_start

        while True:
            h_list = {s_n: self.h_table[s_n] for s_n in self.get_neighbor(s) if s_n in self.h_table}
            if not h_list:
                break

            s_key = min(h_list, key=h_list.get)
            path.append(s_key)
            s = s_key

            if s_key not in h_value:
                return s_key, path

    def iteration(self, CLOSED):
        h_value = {s: float("inf") for s in CLOSED}

        while True:
            h_value_rec = h_value.copy()
            for s in CLOSED:
                h_list = [self.cost(s, s_n) + (self.h_table[s_n] if s_n not in CLOSED else h_value[s_n]) for s_n in self.get_neighbor(s)]
                if h_list:
                    h_value[s] = min(h_list)
            if h_value == h_value_rec:
                return h_value

    def extract_path(self, x_start, parent):
        path_back = [self.s_goal]
        x_current = self.s_goal

        while x_current != x_start:
            x_current = parent[x_current]
            path_back.append(x_current)

        return list(reversed(path_back))

    def calcular_metricas(self, caminho):
        total_km, total_litros, total_portagens = 0, 0, 0

        for i in range(len(caminho) - 1):
            origem, destino = caminho[i], caminho[i + 1]
            if origem in self.grafo and destino in self.grafo[origem]:
                data = self.grafo[origem][destino]
                total_km += data['km']
                total_litros += data['litros']
                total_portagens += data['portagens']

        return total_km, total_litros, total_portagens

if __name__ == '__main__':
    s_start, s_goal = 'Berlin', 'Madrid '

    lrta = LrtAStarN(s_start, s_goal, N=10, heuristic_type="euclidean")
    lrta.searching()
    print("Caminho encontrado:", lrta.path)

    km, litros, portagens = lrta.calcular_metricas(lrta.path)
    print(f"Total de km percorridos: {km}")
    print(f"Total de litros gastos: {litros}")
    print(f"Total gasto em portagens: {portagens}")
    print("Histórico de nós explorados:", lrta.history)
    print(f"Tempo de execução: {lrta.execution_time:.4f} segundos")