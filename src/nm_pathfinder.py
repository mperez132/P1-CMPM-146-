import queue
from typing import NewType
#import Dijkstra_forward_search
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

    boxes, visited = bi_astar(source, dest, mesh, heuristic)
    #boxes, visited = astar(source, dest, mesh, heuristic)
    emptyBox = False
    if (not boxes) and (source != dest):
        emptyBox = True

    boxes.append(dest)
    #print("\nBOXES:", boxes)

    path = find_corners(boxes, source_point, destination_point)

    if not emptyBox:
        path.insert(0, source_point)
        path.append(destination_point)
    
    print("\n")

    return path, visited

#n = curr
#f(n) = g(n) + h(n) : heuristic + cost from beginning to curr
#g(n) = cost from beginning to curr (curr + all parents)
#h(n) = heuristic: estimate from curr to dest 
#current is the highest priority (lowest number) in the queue
def bi_astar(source, dest, mesh, adj):
    #myQueue = queue.PriorityQueue() #open set
    #myQueue.put(source, 0)
    newQueue = []
    secondQueue = []
    heappush(newQueue, (0, source))
    heappush(secondQueue, (0, dest))
    fParent = dict() #came from
    bParent = dict()
    fTotal_cost = dict() #f(g) cost_so_far (gCost)
    bTotal_cost = dict()
    fParent[source] = None
    bParent[dest] = None
    fTotal_cost[source] = 0
    bTotal_cost[dest] = 0

    tempSource = source
    tempDest = dest
    

    while newQueue and secondQueue:
        #curr = myQueue.get()
        fPriority, fCurr = heappop(newQueue)
        bPriority, bCurr = heappop(secondQueue)

        tempSource = bCurr
        tempDest = fCurr

        if fCurr == bCurr:
            print("fCurr:", fCurr)
            print("bCurr:", bCurr)
            fBoxes = fParentPath(fParent, source, bCurr)
            bBoxes = bParentPath(bParent, dest, fCurr)
            boxes = combinePath(fBoxes, bBoxes)
            print("Combined Boxes:", boxes)

            return boxes, fParent.keys()

        for neighbor in mesh['adj'][fCurr]:
            new_cost = fTotal_cost[fCurr] + EuclidianDistance(center(fCurr), center(neighbor))
            if neighbor not in fTotal_cost or new_cost < fTotal_cost[neighbor]:
                fTotal_cost[neighbor] = new_cost
                fPriority = new_cost + heuristic(tempSource, neighbor)
                #myQueue.put(neighbor, priority)
                heappush(newQueue, (fPriority, neighbor))
                fParent[neighbor] = fCurr

        for neighbor in mesh['adj'][bCurr]:
            new_cost = bTotal_cost[bCurr] + EuclidianDistance(center(bCurr), center(neighbor))
            if neighbor not in bTotal_cost or new_cost < bTotal_cost[neighbor]:
                bTotal_cost[neighbor] = new_cost
                bPriority = new_cost + heuristic(tempDest, neighbor)
                heappush(secondQueue, (bPriority, neighbor))
                bParent[neighbor] = bCurr


    print("Path Not Found!")
    return ([], [])

def combinePath(fPath, bPath):
    path = []
    for box in fPath:
        path.append(box)
    for box in bPath:
        path.append(box)
    
    return path
        
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

def fParentPath(parent, source, dest):
    #start from destination
    #find dest parent
    #loop and find every parent before
    path = []
    temp = dest
    while temp != source:
        path.insert(0, parent[temp])
        temp = parent[temp]

    return path

def bParentPath(parent, source, dest):
    path = []
    temp = dest
    while temp != source:
        path.append(parent[temp])
        temp = parent[temp]

    return path

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
        topRight = (box[1], box[3])
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

def astar(source, dest, mesh, adj):
    #myQueue = queue.PriorityQueue() #open set
    myQueue = []
    #myQueue.put(source, 0)
    heappush(myQueue, (0, source))
    parent = dict() #came from
    total_cost = dict() #f(g) cost_so_far (gCost)
    parent[source] = None
    total_cost[source] = 0

    while myQueue:
        prio, curr, = heappop(myQueue)
        #myQueue.pop()
        # boxes.append(curr)
        if (curr == dest): #path found
            #print("BOX TEST 4", parent)
            boxes = parentPath(parent, source, dest)
            #print("BOX TEST 2", boxes)
            return boxes, parent.keys()

        for neighbor in mesh['adj'][curr]: #goes through each neighbor of current
            # if neighbor in boxes: #if neighbor has already been visited/evaluated
            #     continue
            #print("BOX TEST 5", parent)
            new_cost = total_cost[curr] + EuclidianDistance(center(curr), center(neighbor))
            if neighbor not in total_cost or new_cost < total_cost[neighbor]:
                total_cost[neighbor] = new_cost
                prio = new_cost + heuristic(dest, neighbor)
                #myQueue.put(neighbor, priority)
                heappush(myQueue, (prio, neighbor))
                parent[neighbor] = curr
                #print("ANOTHER TEST", curr)

    print("Path not found!")
    return ([], [])