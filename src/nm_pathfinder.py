import queue
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

    #if there is no path then we must display
    if(not source or not dest):
        print("Path not found!")
        return (path,boxes)

    boxes, visited = astar(source, dest, mesh, heuristic)

    emptyBox = False
    if (not boxes) and (source != dest):
        emptyBox = True

    boxes.append(dest)
    #print("BOX TEST 1", boxes)
    
    # path = find_center(boxes)
    path = find_corners(boxes, source_point, destination_point)
    #print("PATH", path)

    if not emptyBox:
        path.insert(0, source_point)
        path.append(destination_point)

    #print("TEST", boxes)
    #print("TEST 2", path)
    #print("\n")
   
    return path, visited


def parentPath(parent, source, dest):
    path = []
    temp = dest
    while temp != source:
        path.insert(0,parent[temp])
        temp = parent[temp]

    return path


def find_center(boxes):
    path = []
    for cell in boxes:
        if cell != None:
            x = (cell[3] + cell[2])/2
            y = (cell[1] + cell[0])/2
            path.append((y, x))
    return path


# def path_to_cell(cell, paths): #creates path from end back to the beginning
#     if cell == []:
#         return []
#     return path_to_cell(paths[cell], paths) + [cell]

def heuristic(b, a):
   return abs(a[0] - b[2]) + abs(a[1] - b[2])


def find_box(point, mesh):
    for box in mesh['boxes']:
        if(point[0] >= box[0] and point[0] <= box[1] and point[1] >= box[2] and point[1] <= box[3]):
            return box
    return False


def EuclidianDistance(cell1, cell2):
    return sqrt((cell1[0] - cell2[0])**2 + (cell1[1] - cell2[1])**2)
    

def find_corners(boxes, source_point, destination_point):
    del boxes[0]
    if(len(boxes) >= 1):
        del boxes[len(boxes) -1 ]
    
    curr = source_point
    newPath = []
    #look at next box
    #get coordinates for all 4 corners
    #select the closest corner for path
    for box in boxes: 
        topLeft = (box[0], box[2])
        topRight = (box[2], box[3])
        botLeft = (box[1], box[2])
        botRight = (box[1], box[3])

        TLcost = EuclidianDistance(curr, topLeft)
        TRcost = EuclidianDistance(curr, topRight)
        BLcost = EuclidianDistance(curr, botLeft)
        BRcost = EuclidianDistance(curr, botRight)

        cornerList = [TLcost, TRcost, BLcost, BRcost]
        bestCost = min(cornerList)

        if(bestCost == TLcost):
            newPath.append(topLeft)
        if(bestCost == TRcost):
            newPath.append(topRight)
        if(bestCost == BLcost):
            newPath.append(botLeft)
        if(bestCost == BRcost):
            newPath.append(botRight)

        curr = center(box)

    return newPath


def center(box):
    x = (box[3] + box[2])/2
    y = (box[1] + box[0])/2
    return (y, x)


def BFS(mesh, source_point, destination_point):
    queue = []
    boxes = []
    path = []
    tempBool = False
    #print our source and our goal coordinates
    print(source_point)
    print(destination_point)
    #find the box
    source_box = find_box(source_point, mesh)
    destination_box = find_box(destination_point, mesh)
    #if there is no path then we must display
    if(not source_box or not destination_box):
        print("No path found!")
        return (path,boxes)
    
    queue.append(source_box)
    boxes.append(source_box)
    while queue:
        curr_box = heappop(queue)
        if curr_box == destination_box:
            print(curr_box)
            print(destination_box)
            tempBool = True
            return path, boxes
        path.append(curr_box)
        for adj_box in mesh['adj'][curr_box]:
            if adj_box not in boxes:
                queue.append(adj_box)
                boxes.append(adj_box)
    
    if tempBool == True:
        print("There is a path with BFS!")
    else:
        print("No path found with BFS!")
    
    return path, boxes


#n = curr
#f(n) = g(n) + h(n) : heuristic + cost from beginning to curr
#g(n) = cost from beginning to curr (curr + all parents)
#h(n) = heuristic: estimate from curr to dest 
#current is the highest priority (lowest number) in the queue
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

def bi_astar(source, dest, mesh, adj):
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

