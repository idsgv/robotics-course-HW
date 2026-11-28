import math
import random

from .collisions import isFree, is_edge_free


def sample_points(N, cspace, mapInfo, seed=0):

    rng = random.Random(seed)

    # Map dimensions in meters
    xmin = mapInfo.origin.position.x
    ymin = mapInfo.origin.position.y
    width = mapInfo.width * mapInfo.resolution
    height = mapInfo.height * mapInfo.resolution

    points = []

    for i in range(N):
        x = xmin + rng.random() * width
        y = ymin + rng.random() * height
        if isFree(x, y, cspace, mapInfo):
            points.append((x, y))

    return points


def create_edges(points, R, cspace, mapInfo):

    graph = {i: {} for i in range(len(points))}
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            distance = math.dist(points[i], points[j])
            if distance <= R:
                if is_edge_free(*points[i], *points[j],cspace, mapInfo):
                    graph[i][j] = distance
                    graph[j][i] = distance
    return graph


def build_prm(start, goal, cspace, mapInfo, N, R, seed):

    if not isFree(*start, cspace, mapInfo): raise ValueError("Start is not free")
    if not isFree(*goal, cspace, mapInfo): raise ValueError("Goal is not free")

    points = [start, goal]
    points += sample_points(N, cspace, mapInfo, seed)
    graph = create_edges(points, R, cspace, mapInfo)

    return points, graph