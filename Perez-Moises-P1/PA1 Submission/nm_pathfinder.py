import queue
from typing import NewType
import Dijkstra_forward_search
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
    sameBox = []

    source = find_box(source_point, mesh)
    dest = find_box(destination_point, mesh)

    if not source or not dest:
        print("No Path Found!")
        return (path, [])

    if (source == dest):
        path.insert(0, source_point)
        path.append(destination_point)
        sameBox.append(source)

        return path, sameBox

    fBoxes, bBoxes, visited = astar(source, dest, mesh, heuristic)

    if not fBoxes and not bBoxes and not visited:
        return [], mesh['boxes']

    emptyBox = False #update this
    if (not fBoxes or not bBoxes) and (source != dest):
        emptyBox = True

    tempBool = False
    fSave = -1
    bSave = -1
    for fBox in fBoxes:
        fSave += 1
        bSave = 0
        for bBox in bBoxes:
            bSave += 1
            if fBox == bBox:
                tempBool = True
                break
        if tempBool:
            break
    
    fCounter = fSave 
    while fCounter >= fSave and fCounter < len(fBoxes):
        if (len(fBoxes) >= 1):
            del fBoxes[fSave]
    
    bCounter = bSave
    while bCounter >= bSave and bCounter < len(bBoxes):
        if (len(bBoxes) >= 1):
            del bBoxes[bSave]
    
    bBoxes.reverse()

    path = combinePath(fBoxes, bBoxes)

    path = find_detail_point(path, source_point)
    if (len(path) >= 1):
        del path[0]
    #UNCOMMENT LATER
    # for box in path:
    #     if (len(path) >= 1) and (box == source or box == dest):
    #         del box

    if not emptyBox:
        path.insert(0, source_point)
        path.append(destination_point)
    
    #print("PATH:", path)
    print("\n")

    return path, visited


def find_detail_point(boxes, start):
    newList = []
    currPoint = start

    for i in range(0, len(boxes), 1):
        pointDist = dict()
        if i < len(boxes) - 1:
            xRange, yRange = find_range(boxes[i], boxes[i + 1])

            point1 = (xRange[0], yRange[0])
            point2 = (xRange[1], yRange[1])

            detailPoints = all_detail_points(point1, point2)

            for point in detailPoints: #get list of all Euclidian distances
                pointDist[point] = (EuclidianDistance(currPoint, point))

            if (len(detailPoints) >= 1):
                bestPoint = min(pointDist)
                #print("Best Point:", bestPoint)
                currPoint = bestPoint

            newList.append(bestPoint)

    return newList



def find_range(curr, dest):
    xRange = [max(curr[2], dest[2]), min(curr[3], dest[3])]
    yRange = [max(curr[0], dest[0]), min(curr[1], dest[1])]

    return xRange, yRange

def all_detail_points(point1, point2):
    detailPoints = []
    if (point1[0] == point2[0]): # vertical edge (x values are the same)
        for i in range(point1[1], point2[1] + 1, 1):
            detailPoints.append((i, point1[0]))
                
    elif (point1[1] == point2[1]): # horizontal edge (y values are the same)
        for i in range(point1[0], point2[0] + 1, 1):
            detailPoints.append((point1[1], i))

    return detailPoints

#n = curr
#f(n) = g(n) + h(n) : heuristic + cost from beginning to curr
#g(n) = cost from beginning to curr (curr + all parents)
#h(n) = heuristic: estimate from curr to dest 
#current is the highest priority (lowest number) in the queue
def astar(source, dest, mesh, adj):
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
        fPriority, fCurr = heappop(newQueue)
        bPriority, bCurr = heappop(secondQueue)

        tempSource = bCurr
        tempDest = fCurr

        # if fCurr == bCurr or tempBool:
        if fCurr == bCurr:
            fBoxes = fParentPath(fParent, source, bCurr)
            bBoxes = bParentPath(bParent, dest, fCurr)
            fParent.update(bParent) #combine both parent dicts

            return fBoxes, bBoxes, fParent.keys()

        #SWAP TEMPSOURCE AND TEMPDEST on EUCLIDIANDISTANCE FUNCTION CALLS
        for neighbor in mesh['adj'][fCurr]:
            new_cost = fTotal_cost[fCurr] + EuclidianDistance(center(tempDest), center(neighbor))
            if neighbor not in fTotal_cost or new_cost < fTotal_cost[neighbor]:
                fTotal_cost[neighbor] = new_cost
                fPriority = new_cost + heuristic(tempSource, neighbor)
                heappush(newQueue, (fPriority, neighbor))
                fParent[neighbor] = fCurr

        for neighbor in mesh['adj'][bCurr]:
            new_cost = bTotal_cost[bCurr] + EuclidianDistance(center(tempSource), center(neighbor))
            if neighbor not in bTotal_cost or new_cost < bTotal_cost[neighbor]:
                bTotal_cost[neighbor] = new_cost
                bPriority = new_cost + heuristic(tempDest, neighbor) #goal to neighbor
                heappush(secondQueue, (bPriority, neighbor))
                bParent[neighbor] = bCurr

    print("Path Not Found!")
    return ([], [], [])


def heuristic(a, b): #a-b was swapped from b-a
   return abs(a[2] - b[3]) + abs(a[0] - b[1])


def find_corners(boxes, source_point):
    # if (len(boxes) >= 1):
    #     del boxes[0]
    #     del boxes[len(boxes) - 1]

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


def combinePath(fPath, bPath):
    path = []
    for box in fPath:
        path.append(box)
    for box in bPath:
        path.append(box)
    
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
        path.insert(0, parent[temp])
        temp = parent[temp]

    return path


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


def find_box(point, mesh):
    for box in mesh['boxes']:
        if(point[0] >= box[0] and point[0] <= box[1] and point[1] >= box[2] and point[1] <= box[3]):
            return box
    return False

