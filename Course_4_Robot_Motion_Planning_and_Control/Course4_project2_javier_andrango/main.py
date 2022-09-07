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
nodes_file = 'planning/nodes.csv'  # ID,x,y,heuristic-cost-to-go
obstacles = np.array(csv_reader(obstacles_file))
nodes = np.array(csv_reader(nodes_file))

# initial valuess
samples_number = 200
search_tree = np.full((samples_number,3),math.inf) # ID,x,y

# add first node in search tree
search_tree[0,:] = nodes[0,:3]

#print("search tree: {}".format(search_tree[0,:]))
#print("nearest node: {}".format(nearest_node(search_tree,[1,1])))

    
