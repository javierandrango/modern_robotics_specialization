import numpy as np
import math
from RRTPlanner import *


# data reader
# obstacles.csv
# - x,y: center of the cylinder
# - diameter: obstacle diameter
# 
# nodes.csv
# - ID: unique integer ID number of the node (1 through N)
# - x,y: location of the node in the plane
# - heuristic-cost-to-go: is an optimistic estimate of the path 
obstacles_file = 'planning/obstacles.csv'  # x,y,diameter
nodes_file = 'planning/nodes_first_run.csv'  # ID,x,y,heuristic-cost-to-go
obstacles = np.array(csv_reader(obstacles_file))
nodes = np.array(csv_reader(nodes_file))

# initial valuess
samples_number = 200
iteration = 0
search_tree = np.full((samples_number,3),math.inf) # ID,x,y
edges = np.empty((0,3)) # ID1,ID2,edge
limits = [-0.5,0.5,-0.5,0.5] # x_min,x_max,y_min,y_max
x_samp = [0,0] 

# define first node in search tree
search_tree[0,:] = nodes[0,:3]
id_node = nodes[0,0]

# define goal node
goal = nodes[1,1:3]
goal_error = 0.15
goal_max_error = list(np.add(goal,goal_error))
goal_min_error = list(np.subtract(goal,goal_error))

while(iteration<=samples_number):
    x_samp = X_sample(limits)
    x_nearest = nearest_node(search_tree,x_samp)
    has_collided,x_new = local_planner(x_nearest[1:],x_samp,obstacles)
    
    if(not(has_collided)):
        # add x_new to search tree
        search_tree[int(id_node),:]=[id_node+1,x_new[0],x_new[1]]

        # add x_nearest and x_new id's to edges
        edge_id1 = int(id_search(search_tree,x_nearest[1:]))
        edge_id2 = int(id_search(search_tree,x_new))
        edge_distance = cartesian_distance(x_nearest[1:],x_new)
        edges = np.append(edges,[[edge_id1,edge_id2,edge_distance]],axis=0)

        id_node+=1
        
        # check if x_new is equal o near to goal
        if (goal_min_error[0]<=x_new[0]<=goal_max_error[0])and(goal_min_error[1]<=x_new[1]<=goal_max_error[1]):
    
            # delete rows without data ([math.inf]*3)
            search_tree = search_tree[np.all(search_tree!=math.inf,axis=1),:]

            # end the loop
            print("SUCESS")
            path_list = short_path(edges)
            print("short path: ",path_list)
            RRT_plot(obstacles,search_tree, edges)
            break
        else:
            print(".",end='\r')

        iteration+=1

# save csv files:
# edges csv
np.savetxt("planning/edges.csv",edges, delimiter=',', fmt='%i,%i,%.3f')

# nodes csv
heuristic_cost_to_go =[]
for node in search_tree:
    heuristic_cost_to_go.append(cartesian_distance(node[1:],search_tree[-1,1:]))
nodes_csv = np.insert(search_tree,3,heuristic_cost_to_go,axis=1)
np.savetxt("planning/nodes.csv",nodes_csv, delimiter=',', fmt='%i,%.3f,%.3f,%.3f')

# path csv
np.savetxt("planning/path.csv",[path_list], delimiter=',', fmt='%i')
