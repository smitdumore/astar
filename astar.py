import heapq
import sys
import numpy as np
import cv2
import time
import math
sys.setrecursionlimit(100000)


tab_height = 250
tab_width = 600

def wall(x,y,clearance):
    if ((x >= (tab_width - clearance)) or (y >= (tab_height - clearance)) or (x <= clearance) or (y <= clearance)):
        return True, x,y
    return False, None

def rect_bottom(x,y,clearance):
    if ((y >= 0) and (x <= (tab_height - 100 + clearance)) and (x >= (100 - clearance)) and (y <= (100 + clearance))):
        return True, x,y
    return False, None

def rect_top(x,y,clearance):
    if (y <= tab_height) and (x >= (100 - clearance)) and (y >= (tab_height - 100 - clearance)) and (x <= (tab_height - 100 + clearance)): 
        return True, x,y
    return False, None

def hexagon(x,y, clearance):
    # inequalities equation of hexagon edges 
    # ratio of the radius of a circumscribed circle to the length of the side of a regular hexagon
    a = 1 / math.sqrt(3) 

    if( (y <= (-a*x + clearance + 1866/5)) and \
        (y >= (a*x - clearance - 616/5)) and \
        (y >= (-a*x - clearance + 1116/5)) and \
        (y <= (a*x + clearance + 2679/100)) and \
        (x <= (clearance + 7299/20)) and \
        (x >= (-clearance + 235))) :
        
            return True, x,y
    return False, None
        
def triangle(x,y,clearance):
    # slope of isoceles edges
    m = math.tan(1.107)

    if  (x >= (2*230 - clearance)) and\
        (y <= (-m*x + clearance + 1145)) and \
        (y >= (m*x - clearance - 895)):
            return True, x,y
    return False, None

def get_inquality_obstacles(clearance):
    
    obstacle_xy_list = []

    for x in range(tab_width + 1):
        for y in range(tab_height + 1):
            
            if(wall(x,y, clearance)[0]):
                obstacle_xy_list.append((x,y))
            elif (rect_top(x,y, clearance)[0]):
                obstacle_xy_list.append((x,y))
            elif (rect_bottom(x,y,clearance)[0]):
                obstacle_xy_list.append((x,y))
            elif (hexagon(x,y,clearance)[0]):
                obstacle_xy_list.append((x,y))
            elif (triangle(x,y,clearance)[0]):
                obstacle_xy_list.append((x,y))

    return obstacle_xy_list

class Astar:
    def __init__(self, start, goal, obstacles, step_size, robot_radius, clearance):
        # x,y,theta,h  
        self.start = (start[0], start[1], start[2])
        self.goal =  (goal[0], goal[1], goal[2])
        self.obstacles = obstacles
        self.step_size = step_size
        self.robot_radius = robot_radius
        self.clearance = clearance
        
        self.heap = []
        self.came_from = {}
        self.cost_so_far = {}
        self.visited = []

    def search(self):

        heapq.heappush(self.heap, (0, self.start))
        self.came_from[self.start] = None
        self.cost_so_far[self.start] = 0

        while self.heap:
            current = heapq.heappop(self.heap)[1]

            print("searching", current[0], tab_height - current[1])

            self.visited.append(current)

            if self.is_goal_reached(current):
                self.came_from[self.goal] = current
                print("Goal found")
                self.backtrack()
                break

            for next in self.neighbors(current):

                if next in self.visited:
                    continue
                
                edge_cost = self.step_size
                new_cost = self.cost_so_far[current] + edge_cost

                if next not in self.cost_so_far or new_cost < self.cost_so_far[next]:         
                    self.cost_so_far[next] = new_cost
                    h = self.heuristic(next[0], next[1])
                    priority = new_cost + h
                    heapq.heappush(self.heap, (priority, next))
                    self.came_from[next] = current

        # Search ended
        # Goal not found
        return


def main():

    # Get start pos from user
    start_x = int(input("Enter start x coordinate: "))
    start_y = int(input("Enter start y coordinate: "))
    start_ori = int(input("Enter start orientation: "))
    # Change origin
    start_y = tab_height - start_y
    start = (start_x, start_y, start_ori)
      
    # get goal pos from user
    goal_x = int(input("Enter goal x coordinate: "))
    goal_y = int(input("Enter goal y coordinate: "))
    goal_ori = int(input("Enter goal orientation: "))
    # Change origin
    goal_y = tab_height - goal_y
    goal = (goal_x, goal_y, goal_ori)

    step_size = int(input("Enter step size between 0 and 10: "))
    robot_radius = 5
    clearance = 5

    # Generate obstacles
    occupied = get_inquality_obstacles(clearance + robot_radius)

    if start in occupied:
        print("start position occupied. Please enter a valid position and run code again.")
        return
    elif 0 < start[0] and start[0] >= tab_width and 0 < start[1] and start[1] >= tab_height:
        print("start position out of bounds. Please enter a valid position and run code again.")
        return

    if goal in occupied:
        print("goal position occupied. Please enter a valid position and run code again.")
        return
    elif 0 < goal[0] and goal[0] >= tab_width and 0 < goal[1] and goal[1] >= tab_height:
        print("goal position out of bounds. Please enter a valid position and run code again.")

            
    ##########
    # A-Star #
    ##########
    obj = Astar(start, goal, occupied, step_size, robot_radius, clearance)
    obj.search()
    print("---program ended---")

if __name__ == "__main__":
    main()
