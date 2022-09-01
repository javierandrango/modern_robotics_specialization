import csv
import random
import math
import numpy as np
import matplotlib.pyplot as plt


# CSV reader function
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
            if row[0][0] != '#':
                data.append(row)
    node = np.array(data).astype("float")
    return node

#function to plot the search tree and obstacles
def RRTplot(obstacles, tree_edges):
    '''RRT plotting function.
    General plot of the RRT planner with obstacles and the
    search tree
    Input:
        - obstacles: matrix that represent the location and
          diameter of every obstacle (x,y,diameter).
        - tree_edges: matrix that represent the.......
    Return: 
        - plt.show(): the generated plot of obstacles and
          tree_edges matrix
    '''
    plt.figure(1)
    plt.xlim(-0.5,0.5)
    plt.ylim(-0.5,0.5)
    
    for i in obstacles:
        circle = plt.Circle((i[0],i[1]),radius=i[2]/2)
        plt.gca().add_artist(circle)
    return plt.show()


# function de return a random state of X (state space)
def X_sample(limits,decimals):
    '''Random sample of X (state space)
    The function returns a random sample from a uniform distribution
    over X (state space).
    Input:
        - limits: array with the search limits of the square in the
          form of [x_min,x_max,y_min,y_max]
        - decimals: number of decimals of the sample (x,y) 
    Return:
        - sample: array of coordinates (x,y) that represent a sample
          of X (state space)
    '''
    str_format = "{:."+str(decimals)+"f}"
    x_sample = float(str_format.format(random.uniform(limits[0],limits[1])))
    y_sample = float(str_format.format(random.uniform(limits[2],limits[3])))
    sample = [x_sample, y_sample]
    
    return sample 

# funtion to return euclidean distance between 2 points P1(x1,y1) and P2(x2,y2)
def cartesian_distance(p1,p2,decimals):
    '''Find the distance between two points P1(x1,y1) and P2(x2,y2)
    The function return the euclidean distance between two point
    (cartesian coordinates) in a 2D-plane P1(x1,y1) and P2(x2,y2)
    Input:
        - p1: coordiantes of the first point [x1,y1] 
        - p2: coordinates of the secodn point [x2,y2]
        - decimal: number of decimals of the returned value
    Return:
        distance: euclidean distance P1-P2
    '''
    x1,y1 = p1
    x2,y2 = p2
    raw_distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    str_format = "{:."+str(decimals)+"f}"
    distance = float(str_format.format(raw_distance))
    
    return distance

    
# function to return the nearest node in the search tree (T)
def nearest_node(search_tree, x_samp):
    '''Find the nearest node between search tree(T) and x-sample
    Input:
        - search_tree:
        - x_samp:
    Return:
        - node: 
    '''
    distance_list = [math.inf]*len(search_tree)
    
    for i,distance in enumerate(distance_list):
        if distance == math.inf:
            distance_list[i]=cartesian_distance(search_tree[i,1:],x_samp,3)
        else:
            pass
            
    min_distance = min(distance_list)
    index_distance = distance_list.index(min_distance)
    node = list(search_tree[index_distance,:])
    
    return node

def collision_path(p1, p2, circle):
    '''
        - p1: [x1,y1]
        - p2: [x2,y2]
        - circle:[h,k,diameter]
    return:
        - has_collided: true or false
    
    '''
    # Points A(p1),B(p2),C([h,k]).
    # A,B line segment points
    # C circle center
    radius = circle[2]/2
    distance_CB = cartesian_distance(p2,circle[:2],3)
    distance_CA = cartesian_distance(p1,circle[:2],3)
    
    
    #vectors from the triangle ABC
    CB = np.subtract(p2,C[:2])
    AB = np.subtract(p2,p1)
    
    CA = np.subtract(p1,C[:2])
    BA = np.subtract(p1,p2)

    # the dot product between two vectors( two sides of the triangle),
    # with an angle less than 90 degrees between them, returns a value
    # greater than zero; in this case, that means the projection of
    # the circle center is inside of the line segment
    if np.dot(CB,AB)>0 and np.dot(CA,BA)>0:
        # min_distance <-- triangle height
        triagle_area = abs(np.cross(AB,(-1*CA))/2
        min_distance = (2*triangle_area)/cartesian_distance(p1,p2,3) 
    else:
        #min_distance <-- the smallest side of the triangle
        min_distance = min(distance_CB,distance_CA)

    # max_distance <--the bigger side of the triangle
    max_distance = max(distance_CB,distance_CA)

        
    if min_disntace<=radius and max_distance>=radius:
        has_collided = True
    else:
        has_collided = False
    
    return has_collided
    
     



    
            
            
            
        
    
    
    
    

    
    
    
    
    

    




        
