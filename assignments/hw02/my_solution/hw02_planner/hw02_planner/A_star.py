import math
import heapq


def a_star(points, graph, start=0, goal=1):
    #calculating the heuristic distances
    h = {i: math.dist(points[goal], points[i]) for i in range(len(points))}

    # Priority queue: (f_cost, g_cost, point)
    heap = [(h[start], 0.0, start)]

    #hashing the costs to get to a specific point from start and parents
    g_cost = {start: 0.0}
    parent = {}

    while heap:
        _, road_cost, point = heapq.heappop(heap)

        # Ignore outdated queue entries
        if road_cost > g_cost[point]:
            continue

        # Goal reached: reconstruct the path
        if point == goal:
            path = [point]
            while point != start:
                point = parent[point]
                path.append(point)
            return path[::-1]

        # Explore neighboring nodes
        for neighbor, edge_cost in graph[point].items():
            new_cost = road_cost + edge_cost

            if new_cost < g_cost.get(neighbor, math.inf):
                g_cost[neighbor] = new_cost
                parent[neighbor] = point
                heapq.heappush(heap,(new_cost + h[neighbor], new_cost, neighbor))

    # No path exists in the generated roadmap
    return None