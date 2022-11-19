from __future__ import annotations
from typing import Protocol, Iterator, Tuple, TypeVar, Optional

import collections
import heapq
import time
import csv

class Graph(Protocol):
    def neighbors(self, id: Location) -> list[Location]: pass

class SimpleGraph:
    def __init__(self):
        self.edges: dict[Location, list[Location]] = {}
    
    def neighbors(self, id: Location) -> list[Location]:
        return self.edges[id]


class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, x: T):
        self.elements.append(x)
    
    def get(self) -> T:
        return self.elements.popleft()

def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if id in graph.walls: r = "###"
    if id in graph.packets: r = "€€€"
    if 'path' in style and id in style['path']:   r = " @ "
    if 'start' in style and id == style['start']: r = " A "
    if 'goal' in style and id == style['goal']:   r = " Z "

    return r

def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)

class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: list[GridLocation] = []
    
    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls
    
    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

class WeightedGraph(Graph):
    def cost(self, from_id: Location, to_id: Location) -> float: pass

class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: dict[GridLocation, float] = {}
    
    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)

class PriorityQueue:
    def __init__(self):
        self.elements: list[tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

def dijkstra_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far


def reconstruct_path(came_from: dict[Location, Location],
                     start: Location, goal: Location) -> list[Location]:

    current: Location = goal
    path: list[Location] = []
    if goal not in came_from: # no path was found
        return []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path


def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def greedy(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    
    frontier = PriorityQueue()
    frontier.put(start, 0)
    while not frontier.empty():
        current: Location = frontier.get()
        if current == goal:
            break
        priority=0
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current
        
    return came_from, cost_so_far

def a_star_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    
    frontier = PriorityQueue()
    frontier.put(start, 0)
    while not frontier.empty():
        current: Location = frontier.get()
        if current == goal:
            break
            
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current
        
    return came_from, cost_so_far

def breadth_first_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = Queue()
    frontier.put(start)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in came_from:
                cost_so_far[next] = new_cost
                frontier.put(next)
                came_from[next] = current
    
    return came_from, cost_so_far

def tipo_algoritmo(i):
    if i==1:
        alg="Breadth-First Search"
    elif i==2:
        alg="Dijkstra Search (Uniform Cost)"
    elif i==3:
        alg="Greedy Search"
    elif i==4:
        alg="A* Search"
    return alg

altura = 20
largura = 40
T = TypeVar('T')
Location = TypeVar('Location')

CAPACITY = 4
GridLocation = Tuple[int, int]
diagram4 = GridWithWeights(largura, altura)
walls = [(3, 2), (4, 2), (5, 2), (10, 8), (11, 8), (12, 8), (17, 2), (18, 2), (19, 2), (24, 4), (25, 4), (26, 4),
                  (3, 3), (4, 3), (5, 3), (10, 9), (11, 9), (12, 9), (17, 3), (18, 3), (19, 3), (24, 5), (25, 5), (26, 5),
                  (3, 4), (4, 4), (5, 4), (10, 4), (11, 4), (12, 4), (17, 4), (18, 4), (19, 4), (24, 6), (25, 6), (26, 6),
                  (3, 5), (4, 5), (5, 5), (10, 5), (11, 5), (12, 5), (17, 5), (18, 5), (19, 5), (24, 7), (25, 7), (26, 7),
                  (3, 6), (4, 6), (5, 6), (10, 6), (11, 6), (12, 6), (17, 6), (18, 6), (19, 6), (24, 8), (25, 8), (26, 8),
                  (3, 7), (4, 7), (5, 7), (10, 7), (11, 7), (12, 7), (17, 7), (18, 7), (19, 7), (24, 9), (25, 9), (26, 9),
                  (3,12), (4,12), (5,12), (10,12), (11,12), (12,12), (17,12), (18,12), (19,12), (24,10), (25,10), (26,10),
                  (3,13), (4,13), (5,13), (10,13), (11,13), (12,13), (17,13), (18,13), (19,13), (24,11), (25,11), (26,11),
                  (3,14), (4,14), (5,14), (10,14), (11,14), (12,14), (17,14), (18,14), (19,14), (24,12), (25,12), (26,12),
                  (3,15), (4,15), (5,15), (10,15), (11,15), (12,15), (17,15), (18,15), (19,15), (24,13), (25,13), (26,13),
                  (3,16), (4,16), (5,16), (10,10), (11,10), (12,10), (17,16), (18,16), (19,16), (24,14), (25,14), (26,14),
                  (3,17), (4,17), (5,17), (10,11), (11,11), (12,11), (17,17), (18,17), (19,17), (24,15), (25,15), (26,15),
                  (32,6), (33,6), (34,6), (35, 6), (36, 6), (37, 6), (38, 6), (39, 6),
                  (32,7), (33,7), (34,7), (35, 7), (36, 7), (37, 7), (38, 7), (39, 7),
                  (32,12), (33,12), (34,12), (35,12), (36,12), (37,12), (38,12), (39,12),
                  (32,13), (33,13), (34,13), (35,13), (36,13), (37,13), (38,13), (39,13)]



packets = {(2,4): 0, (2,16): 0, (6,14): 0, (9,6): 0, (13,14): 0, (16,5): 0, (20,8): 0, (20,13): 0, (23,9): 0, (27,5): 0, (27,14): 0, (29,7): 0, (33,14): 0, (38,5): 0}
start, goal = (39, 10), (39, 10)

diagram4.walls=walls
diagram4.packets=packets

opcao = 0

while opcao!=9:
    print("""
    --------------------------------------------
            Agente 007 das Encomendas
    --------------------------------------------
    ------------------ MENU --------------------

    1) Iniciar Recolha

    2) Alterar Matriz do Espaço de Recolha
    3) Alterar Obstáculos
    4) Alterar Posição das Encomendas
    5) Alterar Capacidade do Robot
    6) Alterar Posição do Local de Entrega 
    7) Definir Posição Inicial do Robot

    9) Sair

    --------------------------------------------""")

    opcao=int(input("Escolha a opção pretendida: "))

    if opcao==2:
        print("""
        ----- Valores Atuais -----
        Altura: {}
        Largura: {}
        --------------------------""".format(altura,largura))
        altura=int(input("Indique a altura da matriz: "))
        largura=int(input("Indique a largura da matriz: "))
        diagram4 = GridWithWeights(largura, altura)
        diagram4.walls=walls
        diagram4.packets=packets
    elif opcao==3:
        print("""
        ----- Valores Atuais -----
        {}
        --------------------------""".format(walls))
        obstaculos = input("Indique as posições dos novos obstaculos (separado por ;): ")
        obst_array = obstaculos.split(";")
        diagram4.walls = []
        for o in obst_array:
            t = o.split(",")
            diagram4.walls.append(tuple(map(int, t)))
        print(diagram4.walls)
    elif opcao==4:
        encomendas = ""
        for keys, value in packets.items():
           encomendas += str(keys)
        print("""
        ----- Valores Atuais -----
        {}
        --------------------------""".format(encomendas))
        encomendas = input("Indique as posições das novas encomendas (separado por ;): ")
        obst_array = encomendas.split(";")
        diagram4.packets = {}
        for o in obst_array:
            t = o.split(",")
            diagram4.packets[tuple(map(int, t))] = 0
    elif opcao==5:
        print("""
        ----- Valores Atuais -----
        Capacidade: {}
        --------------------------""".format(CAPACITY))
        CAPACITY=int(input("Indique a capacidade do robot: "))
    elif opcao==6:
        print("""
        ----- Valores Atuais -----
        Posição Entrega: {}
        --------------------------""".format(goal))
        posicao = input("Indique a posição para entrega das encomendas: ")
        t = posicao.split(",")
        goal=tuple(map(int, t))
    elif opcao==7:
        print("""
        ----- Valores Atuais -----
        Posição Inicial: {}
        --------------------------""".format(start))
        posicao = input("Indique a posição inicial do robot: ")
        t = posicao.split(",")
        start=tuple(map(int, t))
    elif opcao==1:
        algoritmo=0
        while algoritmo not in [1,2,3,4]:
            print("""
            ------------------ Algoritmo --------------------

            1) Breadth-First Search
            2) Dijkstra Search (Uniform Cost)
            3) Greedy Search
            4) A* Search

            -------------------------------------------------""")

            algoritmo=int(input("Escolha o algoritmo pretendido: "))
            
        f = open("report.csv", 'a', newline='')
        writer = csv.writer(f, delimiter = ",")
        #writer.writerow(["Timestamp","Tempo","Algoritmo","Start","Goal","Capacidade","Load","Custo","Custo Total","Restantes Encomendas","Tipo de Trajeto"])
        
        start_time = time.time()

        backup = diagram4.packets.copy()
        all_locations = diagram4.packets
        #all_locations.append(goal)

        #METHOD = 'Breadth'
        total_load = 0
        custo_total = 0
        tempo_pesquisa_fim = 0
        pesquisa_fim = 0
        while len(all_locations)>0:
            while total_load < CAPACITY and len(all_locations)>0:
                tempo_pesquisa_inicio = time.time()
                for location in all_locations:
                    if algoritmo==3:
                        came_from, cost_so_far = greedy(diagram4, start, location)
                        all_locations[location] = heuristic(start, location)
                    elif algoritmo==4:
                        came_from, cost_so_far = a_star_search(diagram4, start, location)
                        all_locations[location] = cost_so_far[location] + heuristic(start, location)
                    elif algoritmo==2:
                        came_from, cost_so_far = dijkstra_search(diagram4, start, location)
                        all_locations[location] = cost_so_far[location]
                    
                    #all_locations[location] = cost_so_far[location]
                
                left_locations = sorted(all_locations, key = all_locations.get)
                tempo_pesquisa_fim += (time.time() - tempo_pesquisa_inicio)
                print("Artigos por Recolher : " + str(len(all_locations)))
                print("Localizações : ")
                print(all_locations)

                location = left_locations[0]
                print("Capacidade : " + str(total_load) +" / "+str(CAPACITY))
                print("Posição Inicial : " + str(start))
                print("Próximo Destino : " + str(location))
                #draw_grid(diagram4, point_to=came_from, start=start, goal=location)
                #print(came_from)
                pesquisa_inicio = time.time()
                if algoritmo==1:
                    came_from, cost_so_far = breadth_first_search(diagram4, start, location)
                elif algoritmo==2:
                    came_from, cost_so_far = dijkstra_search(diagram4, start, location)
                elif algoritmo==3:
                    came_from, cost_so_far = greedy(diagram4, start, location)
                elif algoritmo==4:
                    came_from, cost_so_far = a_star_search(diagram4, start, location)
                pesquisa_fim += (time.time() - pesquisa_inicio)
                draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=location))
                total_load += 1
                custo_total += cost_so_far[location]
                writer.writerow([time.time(),(time.time() - start_time),tempo_pesquisa_fim,pesquisa_fim,tipo_algoritmo(algoritmo),start,location,CAPACITY,total_load,cost_so_far[location],custo_total,len(all_locations),"Recolha de Encomenda"])
                print(custo_total)
                del all_locations[location]
                #draw_grid(diagram4, number=cost_so_far, start=start, goal=location)
                start = location
                print()
                #input()

            print("Retorno à base")
            print("Motivo:")
            if len(all_locations)==0:
                print("- Todas as encomendas foram recolhidas.")
            if total_load == CAPACITY:
                print("- Capacidade do robot está completa.")
                
            print("Capacidade : " + str(total_load) +" / "+str(CAPACITY))
            print("Posição Inicial : " + str(start))
            print("Próximo Destino : " + str(goal))
            if algoritmo==1:
                came_from, cost_so_far = breadth_first_search(diagram4, start, goal)
            elif algoritmo==2:
                came_from, cost_so_far = dijkstra_search(diagram4, start, goal)
            elif algoritmo==3:
                came_from, cost_so_far = greedy(diagram4, start, goal)
            elif algoritmo==4:
                came_from, cost_so_far = a_star_search(diagram4, start, goal)
            draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal))
            total_load = 0
            custo_total += cost_so_far[goal]
            writer.writerow([time.time(),(time.time() - start_time),tempo_pesquisa_fim,pesquisa_fim,tipo_algoritmo(algoritmo),start,goal,CAPACITY,total_load,cost_so_far[location],custo_total,len(all_locations),"Retorno à Base"])
            start = goal
            print()

        print("Percurso finalizado")
        print(tipo_algoritmo(algoritmo))
        print("Custo total :: " + str(custo_total))
        print("Duração: --- %s --- segundos" % (time.time() - start_time))
        custo_total =0
        diagram4.packets = backup.copy()
        f.close()
    elif opcao==9:
        print("----------- Programa Encerrado ------------")
    else:
        print("Opcao Inválida")
        







