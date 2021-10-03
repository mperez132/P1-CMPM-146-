import queue
from typing import NewType
from heapq import heappop, heappush, heappushpop

from heapq import heappop, heappush
from math import inf, sqrt

def find_path (source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    path = []
    boxes = []

    source = find_box(source_point, mesh)
    dest = find_box(destination_point, mesh)

    if not source or not dest:
        print("No Path Found!")
        return (path, boxes)

    boxes, visited = astar(source, dest, mesh, heuristic)
    emptyBox = False
    if (not boxes) and (source != dest):
        emptyBox = True

    boxes.append(dest)

    path = find_corners(boxes, source_point, destination_point)

    if not emptyBox:
        path.insert(0, source_point)
        path.append(destination_point)

    #print("\n")

    return path, visited

def find_corners(boxes, source_point, destination_point):
    del boxes[0]
    if (len(boxes) >= 1):
        del boxes[len(boxes) - 1]

    newPath = []
    #look at next box
    #get coordinates for all 4 corners
    #select the closest corner for path
    curr = source_point
    for box in boxes:
        topLeft = (box[0], box[2])
        topRight = (box[2], box[3])
        bottomLeft = (box[1], box[2])
        bottomRight = (box[1], box[3])

        TLcost = EuclidianDistance(curr, topLeft)
        TRcost = EuclidianDistance(curr, topRight)
        BLcost = EuclidianDistance(curr, bottomLeft)
        BRcost = EuclidianDistance(curr, bottomRight)

        cornersList = [TLcost, TRcost, BLcost, BRcost]
        bestCost = min(cornersList)

        if bestCost == TLcost:
            newPath.append(topLeft)
        if bestCost == TRcost:
            newPath.append(topRight)
        if bestCost == BLcost:
            newPath.append(bottomLeft)
        if bestCost == BRcost:
            newPath.append(bottomRight)

        curr = center(box)

    return newPath



def EuclidianDistance(cell1, cell2):
    return sqrt((cell1[0] - cell2[0])**2 + (cell1[1] - cell2[1])**2)


#n = curr
#f(n) = g(n) + h(n) : heuristic + cost from beginning to curr
#g(n) = cost from beginning to curr (curr + all parents)
#h(n) = heuristic: estimate from curr to dest 
#current is the highest priority (lowest number) in the queue
def astar(source, dest, mesh, adj):
    #myQueue = queue.PriorityQueue() #open set
    #myQueue.put(source, 0)
    newQueue = []
    heappush(newQueue, (0, source))
    parent = dict() #came from
    total_cost = dict() #f(g) cost_so_far (gCost)
    parent[source] = None
    total_cost[source] = 0

    while newQueue:
        #curr = myQueue.get()
        priority, curr = heappop(newQueue)
        if (curr == dest):
            boxes = parentPath(parent, source, dest)
            return boxes, parent.keys()

        for neighbor in mesh['adj'][curr]:
            
            new_cost = total_cost[curr] + EuclidianDistance(center(curr), center(neighbor))
            if neighbor not in total_cost or new_cost < total_cost[neighbor]:
                total_cost[neighbor] = new_cost
                priority = new_cost + heuristic(dest, neighbor)
                #myQueue.put(neighbor, priority)
                heappush(newQueue, (priority, neighbor))
                parent[neighbor] = curr

    print("Path Not Found!")
    return ([], [])

def parentPath(parent, source, dest):
    #start from destination
    #find dest parent
    #loop and find every parent before
    path = []
    temp = dest
    while temp != source:
        path.insert(0, parent[temp])
        temp = parent[temp]

    return path


def find_center(boxes):
    path = []
    for cell in boxes:
        x = (cell[3] + cell[2])/2
        y = (cell[1] + cell[0])/2
        path.append((y, x))
    
    return path

def center(box):
    x = (box[3] + box[2])/2
    y = (box[1] + box[0])/2
    return (y, x) 


def heuristic(b, a):
   return abs(a[0] - b[2]) + abs(a[1] - b[2])


def find_box(point, mesh):
    for box in mesh['boxes']:
        if(point[0] >= box[0] and point[0] <= box[1] and point[1] >= box[2] and point[1] <= box[3]):
            return box
    return False

