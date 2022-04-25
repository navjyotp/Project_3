from time import time
import numpy as np
import math
import cv2
import heapq as hq
from math import sqrt
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import roslaunch

# calculate eculidean distance
def euclidean_distance(n1,n2):
    return round(sqrt((n1[0]-n2[0])**2 + (n1[1]-n2[1])**2))


def obstacle_check(x,y):    
    
    if (((x - 200)**2) + ((y - 200)**2) <= (100**2)):
        circle1 =  True
    else:
        circle1 =  False


    if (((x - 200)**2) + ((y - 800)**2) <= (100**2)):
        circle2 =  True
    else:
        circle2 =  False
    
    
    if x <= 175 and x >= 25 and y >= 425 and y <= 575:
        square = True
    else:
        square = False
    
    
    if x <= 625 and x >= 375 and y >= 425 and y <= 575:
        rectangle1 = True
    else:
        rectangle1 = False
    
    
    if x <= 875 and x >= 725 and y >= 200 and y <= 400:
        rectangle2 = True
    else:
        rectangle2 = False
    
    
    if (x < 0 or x > 1000) or (y < 0 or y > 1000):
        outMap = True
    else:
        outMap = False
    
    if(circle1 or circle2 or square or rectangle1 or rectangle2 or outMap):
        return True
    else:
        return False

def clearance_check(x,y):    
   
    if (((x - 200)**2) + ((y - 200)**2) <= (130**2)):
        circle1 =  True
    else:
        circle1 =  False
    

    if (((x - 200)**2) + ((y - 800)**2) <= (130**2)):
        circle2 =  True
    else:
        circle2 =  False
    
    #To check the point is in Square or not
    if x <= 205 and x >= 0 and y >= 295 and y <= 605:
        square = True
    else:
        square = False
    
    #To check the point is in Rectangle 1 or not
    if x <= 655 and x >= 345 and y >= 395 and y <= 605:
        rectangle1 = True
    else:
        rectangle1 = False
    
    #To check the point is in Rectangle 2 or not
    if x <= 905 and x >= 695 and y >= 170 and y <= 430:
        rectangle2 = True
    else:
        rectangle2 = False
    
    #To check the point is out of Map or not
    if (x < 0 or x > 1000) or (y > 1000 or y < 0):
        outMap = True
    else:
        outMap = False
    
    # To check if the point is in the boundary
    if x <= 30 or x >= 970 or y <= 30 or y >= 970:
        inBoundary = True
    else:
        inBoundary = False
    
    if(circle1 or circle2 or square or rectangle1 or rectangle2 or outMap or inBoundary):
        return True
    else:
        return False

def calculate_cost(X_i,Y_i,Theta_i,rpm_L,rpm_R):

    possible = True
    a = 0
    radius = 6.6
    L = 17.8
    dt = 0.1
    u_l = 2*3.14*radius*rpm_L/60

    if u_l > 20:
        u_l = 20
    
    u_r = 2*3.14*radius*rpm_R/60
    if u_r > 20:
        u_r = 20
    
    X_n=X_i
    Y_n=Y_i
    Theta_n = 3.14 * Theta_i / 180
    D=0

    while a<1:
        a = a + dt
        X_s = X_n
        Y_s = Y_n

        Delta_X_n = 0.5*radius * (u_l + u_r) * math.cos(Theta_n) * dt
        
        X_n = X_n + Delta_X_n
        Delta_Y_n = 0.5*radius * (u_l + u_r) * math.sin(Theta_n) * dt
        Y_n = Y_n + Delta_Y_n

        if clearance_check(X_n,Y_n):
            possible = False
            break

        Theta_n += (radius / L) * (u_r - u_l) * dt
        D=D+ math.sqrt(math.pow((0.5*radius * (u_l + u_r) * math.cos(Theta_n) * dt),2)+math.pow((0.5*radius * (u_l + u_r) * math.sin(Theta_n) * dt),2))

    Theta_n = 180 * (Theta_n) / 3.14
    Theta_n = Theta_n % 360
    return [int(X_n), int(Y_n), int(Theta_n)], D , possible

def path_generate(closed_list,path_dict):
    i_n = closed_list[-1][3]          
    i_child = closed_list[-1][2]
    n_path = [(closed_list[-1][1],(0,0))]
    
    while i_n > 0:
        n_path.append((path_dict[i_n][0],path_dict[i_child][2]))
        i_child = i_n
        i_n = path_dict[i_n][1]
   
    n_path.reverse()
    return n_path

def in_Goal_Threshold_Distance(Cu_rrent_n, Goal_n):
    r = 50
    
    if (Goal_n[0]-Cu_rrent_n[0])**2 + (Goal_n[1]-Cu_rrent_n[1])**2 <= r**2:
        return True
    else:
        return False


def index(n,open_list):
    for i in range(len(open_list)):
        if open_list[i][1] == n:
            return i
    return False





def backtrack(X_i,Y_i,Theta_i,rpm_L,rpm_R):
    a = 0
    radius = 6.6
    L = 17.8
    dt = 0.1
    u_l = 2*3.14*radius*rpm_L/60
    if u_l > 20:
        u_l = 20
   
    u_r = 2*3.14*radius*rpm_R/60
    if u_r > 20:
        u_r = 20
   
    X_n=X_i
    Y_n=Y_i
    Theta_n = 3.14 * Theta_i / 180

    while a<1:
        a = a + dt
        X_s = X_n
        Y_s = Y_n

        Delta_X_n = 0.5*radius * (u_l + u_r) * math.cos(Theta_n) * dt
        
        X_n = X_n + Delta_X_n
        Delta_Y_n = 0.5*radius * (u_l + u_r) * math.sin(Theta_n) * dt
        Y_n = Y_n + Delta_Y_n

        Theta_n += (radius / L) * (u_r - u_l) * dt
        ax.plot([X_s,X_n],[999-Y_s,999-Y_n],'g')
        #plot figure
        fig.canvas.draw()
        fig.canvas.flush_events()

        
while True:
    initial_x = int(float(input("Enter x coordinate of initial n (0-10): "))*100)
    initial_y = int(float(input("Enter y coordinate of initial n (0-10): "))*100)
    intial_angle = float(input("Enter start Angle: "))
    if clearance_check(initial_x,initial_y):
        print("The initial coordinates are in obstacle space.Enter coordinates again.")
        continue
    else:
        while True:
            goal_x = int(float(input("Enter x coordinate of goal n (0-10): "))*100)
            goal_y = int(float(input("Enter y coordinate of goal n (0-19): "))*100)
            if clearance_check(goal_x,goal_y):
                print("The goal coordinates are in obstacle space.Enter coordinates again.")
                continue
            else:
                rpm1 = int(input("Enter first  value of wheel RPM: "))
                rpm2 = int(input("Enter second value of wheel RPM: "))
                break
        break

initial_n = (initial_x,initial_y,intial_angle)
goal_n = (goal_x,goal_y)
actions = [[0,rpm1],[rpm1,0],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]


open_list = [[euclidean_distance(initial_n,goal_n),initial_n,1,0,0,euclidean_distance(initial_n,goal_n),(0,0)]]
o_list_set = {initial_n}
hq.heapify(open_list)

closed_list = []
closed_list_set = set()
n_num = 1            
path_dict = {}
itr = 1

vizualization_matrix = np.zeros((1000,1000,3),np.uint8)  
ns = [[initial_n]]    

for i in range(1000):
    for j in range(1000):
        if obstacle_check(i,j):
            vizualization_matrix[i][j] = (0,0,255)


#plot figure 
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.imshow(cv2.flip(cv2.transpose(vizualization_matrix),0))
fig.canvas.draw()
fig.canvas.flush_events()

start_time = time()
while True:
    curr_n = hq.heappop(open_list)
    n = curr_n[1]
    o_list_set.remove(n)
    Index_i = curr_n[2]
    calculate_cost_to_come = curr_n[4]
    calculate_cost_to_go = curr_n[5]
    p_Indec_i = curr_n[3]
    action_set = curr_n[6]
    curr_node_set = []

    if in_Goal_Threshold_Distance(n,goal_n):
        closed_list.append(curr_n)
        closed_list_set.add(n)
        path_dict[Index_i] = [n,p_Indec_i,action_set]
        break

    for action in actions:
        New_position, D, possible = calculate_cost(n[0],n[1],n[2],action[0],action[1])
        New_n = (New_position[0],New_position[1],New_position[2])
        if not possible:
            continue
        else:
            visited_closed_list = False
            for key in closed_list_set:
                if (New_n[0]-key[0])**2 + (New_n[1]-key[1])**2 < 15**2:
                    visited_closed_list = True
                    break
            
            if not visited_closed_list:
                New_calculate_cost_to_come = calculate_cost_to_come + D
                New_calculate_cost_to_go = euclidean_distance(New_n,goal_n)
                New_n_tc_i = New_calculate_cost_to_come + New_calculate_cost_to_go
                ax.plot((n[0],New_n[0]),(999-n[1],999-New_n[1]),'w')
                fig.canvas.draw()
                fig.canvas.flush_events()
                
                
                visited_open_list = False
                for key in o_list_set:
                    if (New_n[0]-key[0])**2 + (New_n[1]-key[1])**2 < 15**2:
                        visited_open_list = True
                        visited_n = key
                        break

                if visited_open_list:
                    idx = index(visited_n,open_list)
                    if New_n_tc_i < open_list[idx][0]:
                        o_list_set.remove(visited_n)
                        o_list_set.add(New_n)

                        open_list[idx][0] = New_n_tc_i
                        open_list[idx][1] = New_n
                        open_list[idx][3] = Index_i

                        open_list[idx][4] = New_calculate_cost_to_come
                        open_list[idx][5] = New_calculate_cost_to_go
                        open_list[idx][6] = (action[0],action[1])

                        hq.heapify(open_list)
                else:
                    n_num = n_num + 1

                    hq.heappush(open_list,[New_n_tc_i,New_n,n_num,Index_i,New_calculate_cost_to_come,New_calculate_cost_to_go,(action[0],action[1])])
                    hq.heapify(open_list)

                    o_list_set.add(New_n)
                    curr_node_set.append(New_n)
    
    ns.append(curr_node_set)
    if len(open_list) == 0:
        print("Solution not found")
        break
    else:
        closed_list.append(curr_n)
        closed_list_set.add(n)
        path_dict[Index_i] = [n,p_Indec_i,action_set]   

end_time = time()
n_path = path_generate(closed_list,path_dict)
n_path = np.array(n_path)
print(n_path) 
print(end_time-start_time)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.imshow(cv2.flip(cv2.transpose(vizualization_matrix),0))
fig.canvas.draw()
fig.canvas.flush_events()
plt.grid()

for i in range(len(n_path)):
    backtrack(n_path[i][0][0],n_path[i][0][1],n_path[i][0][2],n_path[i][1][0],n_path[i][1][1])
    

plt.savefig("~/catkin_ws/src/project3/Astar.png")