# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

from __future__ import annotations
# some of these types are deprecated: https://www.python.org/dev/peps/pep-0585/
from typing import Protocol, Iterator, Tuple, TypeVar, Optional
T = TypeVar('T')

Location = TypeVar('Location')
class Graph(Protocol):
    def neighbors(self, id: Location) -> list[Location]: pass

class SimpleGraph:
    def __init__(self):
        self.edges: dict[Location, list[Location]] = {}
    
    def neighbors(self, id: Location) -> list[Location]:
        return self.edges[id]

example_graph = SimpleGraph()
example_graph.edges = {
    'A': ['B'],
    'B': ['C'],
    'C': ['B', 'D', 'F'],
    'D': ['C', 'E'],
    'E': ['F'],
    'F': [],
}

import collections

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, x: T):
        self.elements.append(x)
    
    def get(self) -> T:
        return self.elements.popleft()

# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)

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

# data from main article
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
CAPACITY = 4
GridLocation = Tuple[int, int]

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

diagram4 = GridWithWeights(40, 20)
diagram4.walls = [(3, 2), (4, 2), (5, 2), (10, 8), (11, 8), (12, 8), (17, 2), (18, 2), (19, 2), (24, 4), (25, 4), (26, 4),
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

diagram4.weights = {loc: 1 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}


diagram4.packets = {(2,4): 0, (2,16): 0, (6,14): 0, (9,6): 0, (13,14): 0, (16,5): 0, (20,8): 0, (20,13): 0, (23,9): 0, (27,5): 0, (27,14): 0, (38,5): 0, (34,14): 0}

import heapq

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

# thanks to @m1sp <Jaiden Mispy> for this simpler version of
# reconstruct_path that doesn't have duplicate entries

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

diagram_nopath = GridWithWeights(10, 10)
diagram_nopath.walls = [(5, row) for row in range(10)]

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

def breadth_first_search2(graph: Graph, start: Location, goal: Location):
    frontier = Queue()
    frontier.put(start)
    came_from: dict[Location, Optional[Location]] = {}
    came_from[start] = None
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
    
    return came_from

class SquareGridNeighborOrder(SquareGrid):
    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x + dx, y + dy) for (dx, dy) in self.NEIGHBOR_ORDER]
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return list(results)

def test_with_custom_order(neighbor_order):
    if neighbor_order:
        g = SquareGridNeighborOrder(30, 15)
        g.NEIGHBOR_ORDER = neighbor_order
    else:
        g = SquareGrid(30, 15)
    g.walls = DIAGRAM1_WALLS
    start, goal = (8, 7), (27, 2)
    came_from = breadth_first_search(g, start, goal)
    draw_grid(g, path=reconstruct_path(came_from, start=start, goal=goal),
              point_to=came_from, start=start, goal=goal)

class GridWithAdjustedWeights(GridWithWeights):
    def cost(self, from_node, to_node):
        prev_cost = super().cost(from_node, to_node)
        nudge = 0
        (x1, y1) = from_node
        (x2, y2) = to_node
        if (x1 + y1) % 2 == 0 and x2 != x1: nudge = 1
        if (x1 + y1) % 2 == 1 and y2 != y1: nudge = 1
        return prev_cost + 0.001 * nudge

start, goal = (39, 10), (39, 10)
all_locations=diagram4.packets
#all_locations.append(goal)

total_load = 0

while len(all_locations)>0:
    while total_load < CAPACITY and len(all_locations)>0:
        for location in all_locations:
            came_from, cost_so_far = a_star_search(diagram4, start, location)
            all_locations[location] = cost_so_far[location]

        left_locations = sorted(all_locations, key = all_locations.get)
        print("Artigos por Recolher : " + str(len(all_locations)))
        print("Localizações : ")
        print(all_locations)

        location = left_locations[0]
        print("Capacidade : " + str(total_load) +" / "+str(CAPACITY))
        print("Posição Inicial : " + str(start))
        print("Próximo Destino : " + str(location))
        #draw_grid(diagram4, point_to=came_from, start=start, goal=location)
        #print(came_from)
        came_from, cost_so_far = a_star_search(diagram4, start, location)
        draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=location))
        total_load += 1
        del all_locations[location]
        #draw_grid(diagram4, number=cost_so_far, start=start, goal=location)
        start = location
        print()
        input()

    print("Retorno à base")
    print("Motivo:")
    if len(all_locations)==0:
        print("- Todas as encomendas foram recolhidas.")
    if total_load == CAPACITY:
        print("- Capacidade do robot está completa.")
        
    print("Capacidade : " + str(total_load) +" / "+str(CAPACITY))
    print("Posição Inicial : " + str(start))
    print("Próximo Destino : " + str(goal))
    came_from, cost_so_far = a_star_search(diagram4, start, goal)
    draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal))
    total_load = 0
    start = goal
    print()
        







