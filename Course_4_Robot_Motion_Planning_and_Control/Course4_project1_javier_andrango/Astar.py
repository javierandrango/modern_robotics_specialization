import numpy as np
import math as mt

#function to create a neighbor node list
def neighbor_list(edges,current,CLOSED):
    '''Create a list of neighbor nodes.
    Neighbot nodes(connected nodes)that are not included
    in CLOSED list.
    Input:
        - current: actual node. 
        - edges: ID1,ID2 of the nodes connected by
          the edge. Matrix of the edges.csv file.
        - CLOSED: list of nodes already explored.
    Return:
        - nbr_list: neighbor list of nodes that are not
          in CLOSED list.
    '''
    nbr_list = []
    #searching for neighbor ID1 --> ID2
    for i in edges[:,0:2]:
        if (current == i[0])and(not(i[1]in CLOSED)):
            nbr_list.append(i[1])
      
    #searching for neighbor ID2 --> ID1
    for i in edges[:,0:2]: 
        if (current == i[1])and(not(i[0] in CLOSED))and(not(current in nbr_list)):
            nbr_list.append(i[0])

    nbr_list.sort()
    return nbr_list


# Function to return the cost between nodes
def cost_edges(edges,ID1,ID2):
    ''' Return the cost between nodes stored in the
    "edges" matrix.
    Search a specific cost value in the "edges" matrix
    Input:
        - edges: ID1,ID2 of the nodes connected by
          the edge. Matrix of the edges.csv file.
        - ID1, ID2: are the IDs of the nodes connected
          by the edge
    Return:
        - cost: the cost of traversing that edge (in
         either direction)   
    '''
    ids = [ID1,ID2]
    # sort the elements from higher to lower
    ids.sort(reverse=True)
    # search for the index and return a tuple single element(index)
    index = np.where(edges[:,0] == ids[0])[0]
    # search the ID2 element and return the cost
    for i in index:
        if(edges[i,1]==ids[1]):
            cost = edges[i,2]
        else:
            continue
    return cost


# A * Search algorithm
def a_star(nodes, edges):
    '''A* search algorithm
    A* search pseudocode described in Modern Robotics book (Frank C. Park
    & Kevin M.Lynch), Chapter 10. Motion Planning, page 367. 
    input:
        - nodes: [ID,x,y,heuristic-cost-to-go]. Numpy array of
          data from node.csv file
        - edges: [ID1,ID2,cost]. NUmpy array of data from
          edges.csv file
    output:
        - parent(dictionary): reconstructed path from parent
          links of the finnished search.
    '''
    # Initial values
    # goal,last node (last ID)
    goal = nodes[-1,0]

    # heuristic cost to go from node file
    heuristic_ctg = {}
    # initial past cost for all nodes (node1=0, node2-N=inf)
    past_cost={}
    for i in range(1,len(nodes)+1):
        heuristic_ctg[i] = nodes[i-1,-1]
        past_cost[i] = mt.inf
    past_cost[1] = 0
    
    # add first node into OPEN list
    OPEN = []
    OPEN.append(nodes[0,0])    
    
    # CLOSED list (empty at beginning)
    CLOSED = []
    
    # parent node(unknow value at beginning)
    parent={}

    # estimated total cost(unknow value at beginning)  
    est_total_cost = {}
        
    # while OPEN is not empty
    while(OPEN):
        current = OPEN[0]
        OPEN.remove(OPEN[0])
        CLOSED.append(current)
        
        if (current == goal):
            print("SUCESS")
            return parent
            break
        
        for nbr in neighbor_list(edges,current,CLOSED):
            tentative_past_cost = past_cost[current]+cost_edges(edges,current,nbr)
            if(tentative_past_cost < past_cost[nbr]):
                past_cost[nbr]= tentative_past_cost
                parent[int(nbr)]= int(current)
                est_total_cost[nbr]= past_cost[nbr]+ heuristic_ctg[nbr]
                # sort estimate total cost dictionary(item from low to high value)
                sort_est_total_cost = dict(sorted(est_total_cost.items(), key=lambda x: x[1]))
                #add sorted node(i) sort_est_total_cost into OPEN list
                #sort_est_total_cost = {node : estimation total cost}
                for i in sort_est_total_cost:
                    if(not(i in OPEN)) and (not(i in CLOSED)):
                        OPEN.append(i)
    return "FAILURE"

# function to calculate a short path
def short_path(nodes,parent_dict):
    ''' Short path of a reconstructed path retrieved
        from A* search.
        Returns a list representing the short path (nodes
        from low to high) from the reconstructed path
        retrieved from A* search
    input:
        - nodes: [ID,x,y,heuristic-cost-to-go]. Numpy array of
          data from node.csv file
        - parent_dict: reconstructed path retrieved from A*
          search
    output:
        - short_path: resulting short path (nodes from low to high)          
    '''
    short_path = []
    iteration_node = int(nodes[-1,0])
    first_node = int(nodes[0,0])
    while(iteration_node>first_node):
        short_path.append(iteration_node)
        iteration_node = parent_dict[iteration_node]
        if(iteration_node == first_node):
            short_path.append(iteration_node)
        else:
            continue
    short_path.reverse()
    return short_path

