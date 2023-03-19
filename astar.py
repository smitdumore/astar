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
        # x,y,theta
        self.start = (start[0], start[1], start[2])
        self.goal =  (goal[0], goal[1], goal[2])
        self.obstacles = obstacles
        self.step_size = step_size
        self.robot_radius = robot_radius
        self.clearance = clearance
        self.image = np.zeros((250, 600, 3), np.uint8)
        
        self.heap = []
        self.came_from = {}
        self.cost_so_far = {}
        self.visited = np.zeros((2*tab_height,2*tab_width,12))
        
    def search(self):

        heapq.heappush(self.heap, (0, self.start))
        self.came_from[self.start] = None
        self.cost_so_far[self.start] = 0

        while self.heap:
            current = heapq.heappop(self.heap)[1]

            print("searching", current[0], tab_height - current[1])

            self.visited[int(current[0]*2)][int(current[1]*2)][int(current[2]/30)] = 1

            if self.is_goal_reached(current):
                #self.came_from[self.goal] = current
                print("Goal found.")
                self.backtrack(current)
                break

            for next in self.neighbors(current):

                if self.is_visited(next):
                    continue
                
                cv2.line(self.image, (next[0], next[1]) , (current[0], current[1]), (0, 255, 0), 1)

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
        print("no path from start to goal")
        return
    
    def is_visited(self, current):
        
        if self.visited[int(current[0]*2)][int(current[1]*2)][int(current[2]/30)] == 1:
            return True
        
        return False
        
    def is_goal_reached(self, current):

        if np.sqrt((current[0] - self.goal[0])**2 + (current[1] - self.goal[1])**2) <= 1.5\
              and current[2] == self.goal[2]:
            return True
        return False
     
    def backtrack(self, current):
        
        path = []
        #current = self.goal

        while current != self.start:
            path.append(current)    
            current = self.came_from[current]
        
        path.append(self.start)
        path.reverse()
        print("backtracking done.  visualising ...")
        self.visualise(path)
        return path
    
    def neighbors(self, curr_node):

        sucessors = []
        angle_action_set = [-60, -30, 0, 30, 60]

        x = curr_node[0]
        y = curr_node[1]
        ori = curr_node[2]

        for theta in angle_action_set:
            neigh_x = self.normalize( x + self.step_size*(math.cos(math.radians(ori + theta))))
            neigh_y = self.normalize( y + self.step_size*(math.sin(math.radians(ori + theta))))
            
            neigh_x = round(neigh_x)
            neigh_y = round(neigh_y)
            
            neigh_ori = ori + theta

            if neigh_ori < 0:
                neigh_ori += 360 
            neigh_ori %= 360

            n = (neigh_x, neigh_y, neigh_ori)
            n_xy = (neigh_x, neigh_y)
            
            if n_xy not in self.obstacles and\
                    0 <= n[0] < tab_width and\
                    0 <= n[1] < tab_height:

                sucessors.append(n)

        return sucessors
        
    def heuristic(self, n_x, n_y):
        return math.sqrt(((n_x - self.goal[0])**2) + ((n_y - self.goal[1])**2))

    def normalize(self, val):
        return round(val*2)/2
    
    def visualise(self, path):
    
        White = (255, 255, 255)
        Blue = (255, 0, 0)
        Red = (0, 0, 255)

        #visualise padded obs
        for point in self.obstacles:
            cv2.rectangle(self.image, (point[0], point[1]), (point[0]+1, point[1]+1), White, -1)
        
        # Visualise original obstacles
        obstacle_points = get_inquality_obstacles(0)
        for point in obstacle_points:
            cv2.rectangle(self.image, (point[0], point[1]), (point[0]+1, point[1]+1), Blue, -1)

        # Path
        i = 0
        last_point = self.start
        for point in path:
            cv2.rectangle(self.image, (point[0], point[1]), (point[0]+1, point[1]+1), Red, -1)
            cv2.line(self.image, (last_point[0], last_point[1]) , (point[0], point[1]), Red, 2)
            last_point = point
            cv2.imshow("Astar", self.image)
            cv2.waitKey(1)
        
        #cv2.destroyAllWindows()
        time.sleep(50)
        cv2.waitKey(100)


def main():

    # Get start pos from user
    start_x = int(input("Enter start x coordinate: "))
    start_y = int(input("Enter start y coordinate: "))
    start_ori = int(input("Enter start orientation (multiple of 30deg): "))
    # Change origin
    if start_ori != 0:
        start_ori = 360 - start_ori
    start_y = tab_height - start_y
    start = (start_x, start_y, start_ori)
      
    # get goal pos from user
    goal_x = int(input("Enter goal x coordinate: "))
    goal_y = int(input("Enter goal y coordinate: "))
    goal_ori = int(input("Enter goal orientation (multiple of 30deg) : "))
    # Change origin
    if goal_ori != 0:
        goal_ori = 360 - goal_ori
    goal_y = tab_height - goal_y
    goal = (goal_x, goal_y, goal_ori)

    step_size = int(input("Enter step size between 0 and 10: "))
    robot_radius = int(input("Enter robot radius (5): "))
    clearance = int(input("Enter clearance (5): "))

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
        return

    
    ##########
    # A-Star #
    ##########
    obj = Astar(start, goal, occupied, step_size, robot_radius, clearance)
    obj.search()
    print("---program ended---")

if __name__ == "__main__":
    main()
    