import math

def world_to_grid(x, y, mapInfo):
    origin_x = mapInfo.origin.position.x
    origin_y = mapInfo.origin.position.y
    resolution = mapInfo.resolution

    col = (x - origin_x) / resolution
    row = (y - origin_y) / resolution

    return row, col

#is (x, y) collision-free in cspace?
def isFree (x, y, cspace, mapInfo):
    if not math.isfinite(x) or not math.isfinite(y):
        return False
    
    row, col = world_to_grid(x, y, mapInfo)

    col = math.floor(col)
    row = math.floor(row)

    if not (0 <= row < mapInfo.height and 0 <= col < mapInfo.width):
        return False

    return cspace[row, col] == 0

#UNFINISHED
def is_edge_free(x1, y1, x2, y2, cspace, mapInfo):

    # Check both endpoints
    if not isFree(x1, y1, cspace, mapInfo): return False
    if not isFree(x2, y2, cspace, mapInfo): return False

    # Convert meters to continuous grid coordinates
    gx1, gy1 = world_to_grid(x1, y1, mapInfo)
    gx2, gy2= world_to_grid(x2, y2, mapInfo)

    # Find all columns touched by the edge
    col_min = max(0, math.floor(min(gx1, gx2) - 1e-9))
    col_max = min(mapInfo.width - 1,math.floor(max(gx1, gx2) + 1e-9))

    for col in range(col_min, col_max + 1):
        # Find where the edge enters and exits the column
        if (gx2 - gx1) == 0:
            t_min = 0.0
            t_max = 1.0
        else:
            t1 = (col - 1e-9 - gx1) / (gx2 - gx1)
            t2 = (col + 1 + 1e-9 - gx1) / (gx2 - gx1)

            t_min = max(0.0, min(t1, t2))
            t_max = min(1.0, max(t1, t2))

        if t_min > t_max:
            continue

        # Calculate the corresponding y coordinates
        ya = gy1 + t_min * (gy2 - gy1)
        yb = gy1 + t_max * (gy2 - gy1)

        # Find all rows touched inside this column
        row_min = max(0, math.floor(min(ya, yb) - 1e-9))
        row_max = min(mapInfo.height - 1,math.floor(max(ya, yb) + 1e-9))

        # Check every touched cell
        for row in range(row_min, row_max + 1):
            if cspace[row, col] != 0: return False

    return True