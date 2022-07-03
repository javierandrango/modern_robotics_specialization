import numpy as np
import csv
#python file with A* algorithm and useful functions
from Astar import *

#CSV reader function
def csv_reader(file_path):
    '''CSV files reader function.
    Only valid for files with a specific format
    that contains comments '#' at the beginning
    of the file and data separated by commas ','.
    Input:
        - file_path: CSV file with comments
    Return:
        - node: Numpy array of data
    '''
    with open(file_path) as file:
        reader = csv.reader(file)
        data = []
        for row in reader:
            if row[0][0]!='#':
                data.append(row)
    node = np.array(data).astype("float")
    return node

# data reader
#- ID: unique integer ID number of the node (1 through N)
#- x,y: location of the node in the plane
#- heuristic-cost-to-go: is an optimistic estimate of the path
#  length from that node to the goal node
#- ID1, ID2: are the IDs of the nodes connected by the edge
#- cost: the cost of traversing that edge (in either direction)
nodes_file = 'results/nodes.csv'  # ID,x,y,heuristic-cost-to-go
edges_file = 'results/edges.csv'  # ID1,ID2,cost
nodes = np.array(csv_reader(nodes_file))
edges = np.array(csv_reader(edges_file))

#functions fetched from Astart.py
parent_dict= a_star(nodes,edges)
path = short_path(nodes,parent_dict)
print(path)

#save path in a CSV file
np.savetxt("results/path.csv",[path], delimiter=',', fmt='%i')
        
    
    




