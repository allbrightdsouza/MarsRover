import time
start_time = time.time()

import queue
import math
from heapq import heappush, heappop
ACost = 10
DCost = 14


#Extractors
def get_tuple_frm_text(line):
    (m,n) = line.split(" ")
    return (int(m),int(n))
def get_val_frm_text(line):
    return (int(line))
def get_array_from_text(line):
    temp  = line.split(" ")
    temp = list(map(int, temp)) 
    return temp

#Helpers
def get_path(start,end,came_from):
    output_txt = ""
    try:
        if start == end:
            output_txt = str(start[0]) +"," + str(start[1]) + " " + str(end[0]) +"," + str(end[1])
        else:
            while start != end:
                new_end = came_from[end]
                output_txt = str(end[0]) +"," + str(end[1]) + " "+ output_txt
                end = new_end
            output_txt = str(end[0]) +"," + str(end[1]) + " "+ output_txt
            output_txt = output_txt[0:len(output_txt)-1]
        print(output_txt,"")
        return output_txt
    except:
        print("FAIL")
        return("FAIL")
    
def gen_neighbors(g,max):
    neighbors = []
    height = len(g)
    width = len(g[0])
    neighbor_dict = {}
    for i in range(height):
        for j in range(width):
            cur_neighbor = [(j-1,i-1),(j-1,i),(j-1,i+1),(j,i-1),(j,i+1),(j+1,i-1),(j+1,i),(j+1,i+1)]
            cur_neighbor = list(filter(lambda x : x[0] >= 0 and x[0] < width and x[1] >= 0 and x[1] < height,cur_neighbor))
            cur_neighbor = list(filter(lambda x : abs(g[x[1]][x[0]] - g[i][j]) <= max,cur_neighbor))
            neighbor_dict[(j,i)] = cur_neighbor
    return neighbor_dict
def get_neighbors(cur,g):
    neighbors = []
    height = len(g)
    width = len(g[0])
    for i in range(-1,2):
        for j in range(-1,2):
            if not (cur[0] + i < 0 or cur[1] + j < 0 or cur[0] + i >= width or cur[1] + j >= height or i == j == 0):
                neighbors.append((cur[0]+i,cur[1]+j))
    return neighbors

#Algorithms
def bfs(neighbor_dict,start,dest,max):
    frontier = queue.Queue()
    frontier.put(start)
    
    came_from = {}
    came_from[start] = None
    count = 0
    while not frontier.empty():
        cur = frontier.get()
        
        if cur == dest:
            break
        for next in neighbor_dict[cur]:
            if (next not in came_from):
                frontier.put(next)
                came_from[next] = cur

    return came_from
def dijkstras(neighbor_dict,start,dest,max):
    frontier = []
    heappush(frontier,(0,start))
    
    came_from = {}
    came_from[start] = None
    
    cost_sofar = {}
    cost_sofar[start] = 0
    
    visited = {}
    while len(frontier) != 0:
        cur = heappop(frontier)[1]
        while cur in visited:
#             print("Ive actually entered HEEEEEEEEEEERE!!!")
            cur = heappop(frontier)[1]
        visited[cur] = True
        if cur == dest:
            print("UCS Cost so far",cost_sofar[cur])
            break
        for next in neighbor_dict[cur]:
            travel = DCost if abs(next[1]-cur[1]) + abs(next[0] - cur[0]) == 2 else ACost
 
            new_cost = cost_sofar[cur] + travel
#             if(next[0] == next[1] == 3):
#                 print("cur" ,cur, "next" , next,"slope",slope,"travel",travel,cost_sofar[cur], " = ", new_cost )
            if (next not in cost_sofar or cost_sofar[next] > new_cost):
                heappush(frontier,(new_cost,next))
                came_from[next] = cur
                cost_sofar[next] = new_cost

    return came_from

def a_star(g,neighbor_dict,start,dest,max):
    frontier_front = []
    heappush(frontier_front,(0,start))
    frontier_back = []
    heappush(frontier_back,(0,dest))
    
    came_from = {}
    came_from[start] = None
    came_through = {}
    came_through[dest] = None
    
    cost_sofar_front = {}
    cost_sofar_front[start] = 0
    cost_sofar_back = {}
    cost_sofar_back[dest] = 0
    
    visited_front = {}    
    visited_back = {}

    
    intersect = ()
    intersect_cost = -1
    while len(frontier_front) != 0 and len(frontier_back) != 0:
        cur_front = heappop(frontier_front)[1]
        cur_back = heappop(frontier_back)[1]
        
        while cur_front in visited_front:
#             print("Ive actually entered HEEEEEEEEEEERE!!!")
            cur_front = heappop(frontier_front)[1]
        while cur_back in visited_back:
#             print("Ive actually entered HEEEEEEEEEEERE!!! BaCk")
            cur_back = heappop(frontier_back)[1]
            
        visited_front[cur_front] = True
        visited_back[cur_back] = True
        
        if visited_front.keys() & visited_back.keys() != set():

            intersections = visited_front.keys() & visited_back.keys()
#         if cost_sofar_front.keys() & cost_sofar_back.keys() != set():
#             intersections = cost_sofar_front.keys() & cost_sofar_back.keys()
            for i in intersections:
                intersect = i
                break
            print("this is intersection",intersect,len(intersections))
            new_cost = cost_sofar_front[intersect] + cost_sofar_back[intersect]
#             print("inter total Cost" , cost_sofar_front[intersect], cost_sofar_back[intersect])
            if(intersect_cost == -1 or intersect_cost > new_cost ):
                intersect_cost = new_cost
            else:
                print("ATSRAR final total Cost" , cost_sofar_front[intersect], cost_sofar_back[intersect], new_cost)
                begin = intersect 
                while begin != dest:
                    came_from[came_through[begin]] = begin
                    begin = came_through[begin]

                break
        
        for next in neighbor_dict[cur_front]:
            slope = abs(adj[next[1]][next[0]] - adj[cur_front[1]][cur_front[0]])
            travel = DCost if abs(next[1]-cur_front[1]) + abs(next[0] - cur_front[0]) == 2 else ACost
            new_cost = cost_sofar_front[cur_front] + slope + travel
            if (next not in cost_sofar_front or cost_sofar_front[next] > new_cost):
                total_cost = new_cost + diag_heuristic(next,dest)
                heappush(frontier_front,(total_cost,next))
                came_from[next] = cur_front
                cost_sofar_front[next] = new_cost
                
        for next in neighbor_dict[cur_back]:
            slope = abs(adj[next[1]][next[0]] - adj[cur_back[1]][cur_back[0]])
            travel = DCost if abs(next[1]-cur_back[1]) + abs(next[0] - cur_back[0]) == 2 else ACost
            new_cost = cost_sofar_back[cur_back] + slope + travel
            if (next not in cost_sofar_back or cost_sofar_back[next] > new_cost):
                total_cost = new_cost + diag_heuristic(next,start)
                heappush(frontier_back,(total_cost,next))
                came_through[next] = cur_back
                cost_sofar_back[next] = new_cost

    return came_from
def man_heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return 10*(abs(x1 - x2) + abs(y1 - y2))
def diag_heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return DCost*min(dx,dy) + ACost*abs(dx-dy)


#Get Values from text
try :
    f = open("input.txt")
    lines = f.read().splitlines()
    algo = lines[0]
    (width,height) = get_tuple_frm_text(lines[1])
    start = get_tuple_frm_text(lines[2])
    maxElevation =get_val_frm_text(lines[3])
    noOfDests =get_val_frm_text(lines[4])
    dests = []
    for i  in range (5, 5 + noOfDests):
        dests.append(get_tuple_frm_text(lines[i]))
    adj = []
    for i in range (5 + noOfDests, len(lines)):
        adj.append(get_array_from_text(lines[i]))


    
except:
    print("FAIL")


neighbor_dict = gen_neighbors(adj,maxElevation)

f= open("output.txt","w+")
output_string = ""
for dest in dests:
    if algo == "BFS":
        true_path = bfs(neighbor_dict,start,dest,maxElevation)
    elif algo == "UCS":
        true_path = dijkstras(neighbor_dict,start,dests,maxElevation)
    else:
        true_path = a_star(adj,neighbor_dict,start,dest,maxElevation)
    output_string = output_string + get_path(start,dest,true_path) + "\n"


f.write(output_string[0:len(output_string)-1])
f.close()

print("Program Runtime = ", time.time() - start_time)
