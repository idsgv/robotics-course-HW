import math

def world_to_grid(x, y, map_info):
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    resolution = map_info.resolution

    col = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)

    return row, col

#is (x, y) collision-free in cspace?
def isFree (x, y, cspace, mapInfo):
    if not math.isfinite(x) or not math.isfinite(y):
        return False
    
    row, col = world_to_grid(x, y, mapInfo)

    if not (0 <= row < mapInfo.height and 0 <= col < mapInfo.width):
        return False

    return cspace[row, col] == 0

#UNFINISHED
def is_edge_free(x1, y1, x2, y2, cspace, mapInfo):

    #start and end should be free
    if not is_free(x1, y1, cspace, mapInfo) or not is_free(x2, y2, cspace, mapInfo):
        return False

    
    