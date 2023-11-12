import heapq
import math
import time
from queue import PriorityQueue
import random


# Used for states generation (getChildren())
dx = [-1, 1, 0, 0]
dy = [0, 0, 1, -1]

# Global variables holding algorithms
dfs_counter = 0
bfs_counter = 0
ucs_counter=0
idfs_counter=0
euclid_counter = 0
manhattan_counter = 0
idfs_counter=0
hc_counter=0

dfs_path = []
bfs_path = []
ucs_path=[]
idfs_path=[]
euclid_path = []
manhattan_path = []
idfs_path=[]
hc_path=[]


dfs_cost = 0
bfs_cost = 0
ucs_cost=0
idfs_cost=0
euclid_cost = 0
manhattan_cost = 0
idfs_cost=0
hc_cost=0


dfs_depth = 0
bfs_depth = 0
ucs_depth=0
idfs_depth=0
euclid_depth = 0
manhattan_depth = 0
idfs_depth=0
hc_depth=0

time_dfs = 0
time_bfs = 0
time_ucs=0
time_idfs=0
time_euclid = 0
time_manhattan = 0
time_idfs=0
time_hc=0


# function to get String representation
def getStringRepresentation(x):
    if int(math.log10(x)) + 1 == 9:
        return str(x)
    else:
        return "0" + str(x)


# function to generate all valid children of a certain node
def getChildren(state):
    children = []
    idx = state.index('0')
    i = int(idx / 3)
    j = int(idx % 3)
    for x in range(0, 4):
        nx = i + dx[x]
        ny = j + dy[x]
        nwIdx = int(nx * 3 + ny)
        if checkValid(nx, ny):
            listTemp = list(state)
            listTemp[idx], listTemp[nwIdx] = listTemp[nwIdx], listTemp[idx]
            children.append(''.join(listTemp))
    return children


# function to get the path to the goal state
def getPath(parentMap, inputState):
    path = []
    temp = 123654780
    while temp != inputState:
        path.append(temp)
        temp = parentMap[temp]
    path.append(inputState)
    path.reverse()
    return path


# function to print the path to goal
def printPath(path):
    for i in path:
        print(getStringRepresentation(i))


# function to check the goal state
def goalTest(state):
    if state == 123654780:
        return True
    return False


# function to check if the start state solvable or not
def isSolvable(digit):
    count = 0
    for i in range(0, 9):
        for j in range(i, 9):
            if digit[i] > digit[j] and digit[i] != 9:
                count += 1
    return count % 2 == 0


# breadth first search algorithm
def BFS(inputState):
    # generating start states of variables and data structures used in the algorithm
    start_time = time.time()
    q = []
    explored = {}
    parent = {}
    parent_cost = {}
    integer_state = int(inputState)
    q.append(integer_state)  # here you place the input
    cnt = 0
    global bfs_counter
    global bfs_path
    global bfs_cost
    global bfs_depth
    global time_bfs
    bfs_depth = 0
    parent_cost[integer_state] = 0
    while q:
        cnt += 1
        state = q.pop(0)
        explored[state] = 1
        bfs_depth = max(bfs_depth, parent_cost[state])
        if goalTest(state):
            path = getPath(parent, int(inputState))
            # printPath(path)
            bfs_counter = cnt
            bfs_path = path
            bfs_cost = len(path) - 1
            time_bfs = float(time.time() - start_time)
            return 1
        # generating childeren
        children = getChildren(getStringRepresentation(state))
        for child in children:
            child_int = int(child)
            if child_int not in explored:
                q.append(child_int)
                parent[child_int] = state
                explored[child_int] = 1
                parent_cost[child_int] = 1 + parent_cost[state]
    bfs_path = []
    bfs_cost = 0
    bfs_counter = cnt
    time_bfs = float(time.time() - start_time)
    return 0




def UCS(inputState):
    # Khởi tạo các biến và cấu trúc dữ liệu
    start_time = time.time()
    pq = PriorityQueue()  # Sử dụng hàng đợi ưu tiên
    explored = {}
    parent = {}
    cost = {}
    integer_state = int(inputState)
    pq.put((0, integer_state))  # Đưa trạng thái đầu vào hàng đợi ưu tiên với chi phí ban đầu là 0
    cnt = 0

    # Các biến toàn cục để lưu thông tin về lời giải
    global ucs_counter
    global ucs_path
    global ucs_cost
    global ucs_depth
    global time_ucs
    ucs_depth = 0
    cost[integer_state] = 0

    while not pq.empty():
        cnt += 1
        current_cost, state = pq.get()
        explored[state] = 1
        ucs_depth = max(ucs_depth, cost[state])

        if goalTest(state):
            path = getPath(parent, int(inputState))
            ucs_counter = cnt
            ucs_path = path
            ucs_cost = len(path) - 1
            time_ucs = float(time.time() - start_time)
            return 1

        # Tạo trạng thái con
        children = getChildren(getStringRepresentation(state))
        for child in children:
            child_int = int(child)
            new_cost = cost[state] + 1

            if child_int not in explored or new_cost < cost[child_int]:
                pq.put((new_cost, child_int))
                parent[child_int] = state
                explored[child_int] = 1
                cost[child_int] = new_cost

    # Không tìm thấy lời giải
    ucs_path = []
    ucs_cost = 0
    ucs_counter = cnt
    time_ucs = float(time.time() - start_time)
    return 0

def IDFS(inputState, maxDepth):
    # Khởi tạo các biến và cấu trúc dữ liệu
    start_time = time.time()
    explored = {}
    parent = {}
    parent_cost = {}
    integer_state = int(inputState)
    parent_cost[integer_state] = 0
    explored[integer_state] = 1

    # Các biến toàn cục để lưu thông tin về lời giải
    global idfs_counter
    global idfs_path
    global idfs_cost
    global idfs_depth
    global time_idfs

    idfs_depth = 0

    for depth in range(1, maxDepth + 1):
        if DLS(integer_state, depth, explored, parent, parent_cost):
            path = getPath(parent, int(inputState))
            idfs_counter = len(explored)
            idfs_path = path
            idfs_cost = len(path) - 1
            time_idfs = float(time.time() - start_time)
            return 1

    # Không tìm thấy lời giải
    idfs_path = []
    idfs_cost = 0
    idfs_counter = len(explored)
    time_idfs = float(time.time() - start_time)
    return 0

def DLS(state, depth, explored, parent, parent_cost):
    if depth == 0 and goalTest(state):
        return True

    if depth > 0:
        children = getChildren(getStringRepresentation(state))
        for child in children:
            child_int = int(child)
            if child_int not in explored:
                explored[child_int] = 1
                parent[child_int] = state
                parent_cost[child_int] = 1 + parent_cost[state]
                if DLS(child_int, depth - 1, explored, parent, parent_cost):
                    return True
                explored.pop(child_int)
                parent.pop(child_int)
                parent_cost.pop(child_int)
    
    return False


def DFS(inputState):
    # generating start states of variables and data structures used in the algorithm
    start_time = time.time()
    stack = []
    explored = {}
    parent = {}
    parent_cost = {}
    integer_state = int(inputState)
    parent_cost[integer_state] = 0
    explored[integer_state] = 1
    stack.append(integer_state)
    cnt = 0
    global dfs_counter
    global dfs_path
    global dfs_cost
    global dfs_depth
    global time_dfs
    dfs_depth = 0
    while stack:
        cnt += 1
        state = stack[-1]
        stack.pop()
        dfs_depth = max(dfs_depth, parent_cost[state])
        if goalTest(state):
            path = getPath(parent, int(inputState))
            # printPath(path)
            dfs_counter = cnt
            dfs_path = path
            dfs_cost = len(path) - 1
            time_dfs = float(time.time() - start_time)
            return 1
        # generating childeren
        children = getChildren(getStringRepresentation(state))
        for child in children:
            child_int = int(child)
            if child_int not in explored:
                stack.append(child_int)
                parent[child_int] = state
                explored[child_int] = 1
                parent_cost[child_int] = 1 + parent_cost[state]
    dfs_path = []
    dfs_cost = 0
    dfs_counter = cnt
    time_dfs = float(time.time() - start_time)
    return 0


# function checking if state is valid or out of bounds
def checkValid(i, j):
    if i >= 3 or i < 0 or j >= 3 or j < 0:
        return 0
    return 1


# heuristic function using manhattan distance
def getManhattanDistance(state):
    tot = 0
    for i in range(1, 9):
        goalX = int(i / 3)
        goalY = i % 3
        idx = state.index(str(i))
        itemX = int(idx / 3)
        itemY = idx % 3
        tot += (abs(goalX - itemX) + abs(goalY - itemY))
    return tot


# heuristic function using manhattan distance

def getEuclideanDistance(state):
    tot = 0
    for i in range(1, 9):
        goalX = int(i / 3)
        goalY = i % 3
        idx = state.index(str(i))
        itemX = int(idx / 3)
        itemY = idx % 3
        tot += math.sqrt(pow((goalX - itemX), 2) + pow((goalY - itemY), 2))
    return tot

'''
def AStarSearch_manhattan(inputState):
    # generating start states of variables and data structures used in the algorithm
    start_time = time.time()
    integer_state = int(inputState)
    heap = []
    explored = {}
    parent = {}
    cost_map = {}
    heapq.heappush(heap, (getManhattanDistance(inputState), integer_state))
    cost_map[integer_state] = getManhattanDistance(inputState)
    heap_map = {}
    heap_map[integer_state] = 1
    global manhattan_counter
    global manhattan_path
    global manhattan_cost
    global manhattan_depth
    global time_manhattan
    manhattan_depth = 0
    while heap:
        node = heapq.heappop(heap)
        state = node[1]
        string_state = getStringRepresentation(state)
        parent_cost = node[0] - getManhattanDistance(string_state)
        # handling the nodes that was renewed
        if not state in explored:
            manhattan_depth = max(parent_cost, manhattan_depth)
        explored[state] = 1

        if goalTest(state):
            path = getPath(parent, int(inputState))
            # printPath(path)
            manhattan_path = path
            manhattan_counter = (len(explored))
            manhattan_cost = len(path) - 1
            time_manhattan = float(time.time() - start_time)

            return 1

        # generating childeren
        children = getChildren(string_state)
        for child in children:
            new_cost = getManhattanDistance(child)
            child_int = int(child)
            if child_int not in explored and child not in heap_map:
                heapq.heappush(heap, (parent_cost + new_cost + 1, child_int))
                heap_map[child_int] = 1
                cost_map[child_int] = parent_cost + new_cost + 1
                parent[child_int] = state
            elif child_int in heap_map:
                if (new_cost + parent_cost + 1) < cost_map[child_int]:
                    parent[child_int] = state
                    cost_map[child_int] = new_cost + parent_cost + 1
                    heapq.heappush(heap, (parent_cost + 1 + new_cost, child_int))
    manhattan_cost = 0
    manhattan_path = []
    manhattan_counter = (len(explored))
    time_manhattan = float(time.time() - start_time)

    return 
'''


def GreedyManhattan(inputState):
    # generating start states of variables and data structures used in the algorithm
    start_time = time.time()
    integer_state = int(inputState)
    heap = []
    explored = {}
    parent = {}
    heapq.heappush(heap, (getManhattanDistance(inputState), integer_state))
    heap_map = {}
    heap_map[integer_state] = 1
    global manhattan_counter
    global manhattan_path
    global manhattan_cost
    global manhattan_depth
    global time_manhattan
    manhattan_depth = 0
    while heap:
        node = heapq.heappop(heap)
        state = node[1]
        string_state = getStringRepresentation(state)
        # handling the nodes that were renewed
        manhattan_depth = max(node[0], manhattan_depth)
        explored[state] = 1

        if goalTest(state):
            path = getPath(parent, int(inputState))
            manhattan_path = path
            manhattan_counter = (len(explored))
            manhattan_cost = len(path) - 1
            time_manhattan = float(time.time() - start_time)
            return 1

        # generating children
        children = getChildren(string_state)
        for child in children:
            child_int = int(child)
            if child_int not in explored and child not in heap_map:
                heapq.heappush(heap, (getManhattanDistance(child), child_int))
                heap_map[child_int] = 1
                parent[child_int] = state
    manhattan_cost = 0
    manhattan_path = []
    manhattan_counter = (len(explored))
    time_manhattan = float(time.time() - start_time)
    return 0


def getHeuristics(state):
    # Assuming state is a string representation of the puzzle
    goal_state = "123654780"  # Replace with your actual goal state
    # Ensure the lengths are the same
    if len(state) != len(goal_state):
        raise ValueError("States must have the same length")

    # Count the number of different positions excluding 0
    diff_count = sum(1 for i, j in zip(state, goal_state) if i != j and i != '0')

    return diff_count


def hillClimbing(inputState):
    start_time = time.time()
    current_state = int(inputState)
    explored = set()
    parent = {}
    global hc_counter
    global hc_path
    global hc_cost
    global hc_depth
    global time_hc
    hc_depth = 0

    while True:
        explored.add(current_state)
        hc_depth += 1
        if goalTest(current_state):
            path = getPath(parent, int(inputState))
            hc_counter = len(explored)
            hc_path = path
            hc_cost = len(path) - 1
            time_hc = float(time.time() - start_time)
            return 1

        neighbors = getChildren(getStringRepresentation(current_state))
        best_neighbor = None
        best_heuristic = getHeuristics(getStringRepresentation(current_state))
        
        print("Neighbors:", neighbors)
        print("Current State:", current_state)
        print("Explored States:", explored)
        print("Best Heuristic:", best_heuristic)

        for neighbor in neighbors:
            neighbor_int = int(neighbor)
            if neighbor_int not in explored:
                heuristic_value = getHeuristics(neighbor)
                print(f"Neighbor: {neighbor_int}, Heuristic: {heuristic_value}")
                if heuristic_value < best_heuristic:
                    best_heuristic = heuristic_value
                    best_neighbor = neighbor_int

        if best_neighbor is not None:
            parent[best_neighbor] = current_state
            current_state = best_neighbor
            print("Moving to Best Neighbor:", current_state)
            print("Explored States:", explored)
        else:
            # Stuck in a local minimum or no better neighbor
            break

    hc_path = []
    hc_cost = 0
    hc_counter = len(explored)
    time_hc = float(time.time() - start_time)
    return 0
# Example usage:
input_state = "613025784"
result = hillClimbing(input_state)
if result == 1:
    print("Hill Climbing Path:")
    printPath(hc_path)
    print("Hill Climbing Cost:", hc_cost)
    print("Hill Climbing Counter:", hc_counter)
    print("Hill Climbing Depth:", hc_depth)
    print("Hill Climbing Time:", time_hc)
else:
    print("Hill Climbing không tìm thấy giải pháp.")


def hillClimbingWithRandomRestart(inputState, max_restarts=1000):
    for _ in range(max_restarts):
        result = hillClimbing(inputState)
        if result == 1:
            return 1  # Solution found
        else:
            # Generate a new random initial state
            inputList = list(inputState)
            random.shuffle(inputList)
            inputState = ''.join(inputList)

    return 0  # No solution found after multiple restarts
# ... (your existing code)


def AStarSearch_euclid(inputState):
    # generating start states of variables and data structures used in the algorithm
    start_time = time.time()
    integer_state = int(inputState)
    heap = []
    explored = {}
    parent = {}
    cost_map = {}
    heapq.heappush(heap, (getEuclideanDistance(inputState), integer_state))
    cost_map[integer_state] = getEuclideanDistance(inputState)
    heap_map = {}
    heap_map[integer_state] = 1
    global euclid_counter
    global euclid_path
    global euclid_cost
    global euclid_depth
    global time_euclid
    euclid_depth = 0
    while heap:
        node = heapq.heappop(heap)
        state = node[1]
        string_state = getStringRepresentation(state)
        parent_cost = node[0] - getEuclideanDistance(string_state)
        # handling the nodes that was renewed
        if not state in explored:
            euclid_depth = max(parent_cost, euclid_depth)
        explored[state] = 1

        if goalTest(state):
            path = getPath(parent, int(inputState))
            # printPath(path)
            euclid_path = path
            euclid_counter = (len(explored))
            euclid_cost = len(path) - 1
            time_euclid = float(time.time() - start_time)

            return 1
        # generating childeren
        children = getChildren(string_state)
        for child in children:
            new_cost = getEuclideanDistance(child)
            child_int = int(child)
            if child_int not in explored and not child in heap_map:
                heapq.heappush(heap, (parent_cost + new_cost + 1, child_int))
                heap_map[child_int] = 1
                cost_map[child_int] = parent_cost + new_cost + 1
                parent[child_int] = state
            elif child_int in heap_map:
                if (new_cost + parent_cost + 1) < cost_map[child_int]:
                    parent[child_int] = state
                    cost_map[child_int] = new_cost + parent_cost + 1
                    heapq.heappush(heap, (parent_cost + 1 + new_cost, child_int))
    euclid_cost = 0
    euclid_path = []
    euclid_counter = (len(explored))
    time_euclid = float(time.time() - start_time)

    return 0

# start_time=time.time()
# for i in range(0,10000):
#     print(1)
# print(start_time-time.time())

# print(DFS("702853641"))
# print(time_dfs)
# print(BFS("702853641"))
# print(time_bfs)
# print(AStarSearch_euclid("702853641"))
# print(time_euclid)
# print(AStarSearch_manhattan("702853641"))
# print(time_manhattan)


# unsolvable 103245678, 702853641
