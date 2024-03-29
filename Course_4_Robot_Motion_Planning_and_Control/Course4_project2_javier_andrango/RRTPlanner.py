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
def RRT_plot(obstacles, search_tree, edges):
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

    # retrieve (x,y) point from search tree
    id1,id2,x1,y1,x2,y2 = [0]*6
    for id_array in edges:
        id1 = id_array[0]
        id2 = id_array[1]
        id1_index_tree = np.where(search_tree[:,0]==id1)[0][0]
        id2_index_tree = np.where(search_tree[:,0]==id2)[0][0]
        x1,y1 = list(search_tree[id1_index_tree,1:]) 
        x2,y2 = list(search_tree[id2_index_tree,1:])
        plt.plot([x1,x2],[y1,y2])        
    plt.show()


# function de return a random state of X (state space)
def X_sample(limits,decimals=3):
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
def cartesian_distance(p1,p2,decimals=3):
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
        - search_tree(numpy.ndarray):
        - x_samp(list): [x_samp,y_samp] cartesian coordinates
    Return:
        - node(list): [ID,x_node,y_node] id and cartesian coordinates 
    '''
    distance_list = [math.inf]*len(search_tree)
    
    for i,distance in enumerate(distance_list):
        if distance == math.inf:
            distance_list[i]=cartesian_distance(search_tree[i,1:],x_samp)
        else:
            pass
            
    min_distance = min(distance_list)
    index_distance = distance_list.index(min_distance)
    node = list(search_tree[index_distance,:])
    
    return node

# function to return a free path between a line segment and a circle (collision detection)
def collision_path(A, B, circle,offset=0):
    '''
        - A(list): [x1,y1] 
        - B(list): [x2,y2]
        - circle(list):[h,k,diameter] 
    return:
        - has_collided(bool): true or false
    
    '''
    # A,B line segment points (x,y)
    # C circle center (x,y)
    x1,y1 = A
    x2,y2 = B
    xc,yc = circle[:2]
    radius = circle[2]/2    
    
    #general line equation ax+by+c=0, slope m
    a,b,c,m = [math.inf]*4
    #projection of C point (xp,yp) into AB line segment
    xp,yp = [math.inf]*2

    # horizontal line
    if y1==y2 and x1!=x2:
        a = 0
        b = 1
        c = -y1
        
        #projection of C point
        xp = xc
        yp = y1
        
    # vertical line
    elif x1==x2 and y1!=y2:
        a = 1
        b = 0
        c = -x1

        #projection of C point
        xp = x1
        yp = yc
        
    # sloping line
    else:
        m = (y2-y1)/(x2-x1)
        a = m
        b = -1
        c = y1-m*x1
        
        # projection of C point
        # equation #1: -1/m=(yp-yc)/(xp-xc)
        # equation #2: m=(y2-yp)/(x2-xp)
        const_1 = -xc-m*yc
        const_2 = m*x2-y2
        xp = (m*const_2-const_1)/(1+m**2)
        yp = m*xp-const_2

    # vertical distance from point C to AB line segment
    vertical_distance = abs(a*xc+b*yc+c)/math.sqrt(a**2+b**2)

    # distance from center point to A and B points
    distance_CA = cartesian_distance(A,[xc,yc])
    distance_CB = cartesian_distance(B,[xc,yc])
    
    # check if projection of C point (xp,yp) belongs to 
    # AB segment using dot product of vector AB and AC
    # https://lucidar.me/en/mathematics/check-if-a-point-belongs-on-a-line-segment/
    vect_AB = np.subtract([x2,y2],[x1,y1])
    vect_AC = np.subtract([xp,yp],[x1,y1])
    k_AC = np.dot(vect_AB,vect_AC)
    k_AB = np.dot(vect_AB,vect_AB)
    if 0<=k_AC<=k_AB:
        is_segment_point = True
    else:
        is_segment_point = False
    
    # free path condition
    if vertical_distance>(radius+offset):
        has_collided = False
    elif vertical_distance<(radius+offset) and (not(is_segment_point)) and (min(distance_CA,distance_CB)>(radius+offset)):
        has_collided = False
    else:
        has_collided = True

    return has_collided

# local planner 
def local_planner(x_nearest,x_samp,obstacles,x_samp_offset=0.01):
    '''Local planner to find a motion from x_nearest to x_new
    in the direction of X_samp
    Args:
        - x_nearest:
        - x_samp:
        - obstacles: 
        - x_samp_offset(float):(defaut:0.01) offset from
          x_samp(x,y) coordinates to generate a new limits
          interval
    Returns:
        - has_collided(bool):
        - x_new(list):   
    '''
    
    xs,ys = x_samp
    xn,yn = x_nearest
    
    # define a x_new point in direction of x_samp point
    x_min = xs-x_samp_offset 
    x_max = xs+x_samp_offset
    y_min = ys-x_samp_offset
    y_max = ys+x_samp_offset
    new_limits = [x_min, x_max, y_min, y_max]
    x_new = X_sample(new_limits)

    # middle point of a segment [xn,yn],[xw,yw]
    xw,yw = x_new
    mid_point = [(xn+xw)/2,(yn+yw)/2]
    
    # use the middle point of the segment to find the nearest obstacle
    dis_point_obs_list = []
    index_obstacle = 0
    for obstacle in obstacles:
        dis_point_obs_list.append(cartesian_distance(mid_point,list(obstacle[:2])))
    index_obstacle = dis_point_obs_list.index(min(dis_point_obs_list))
    nearest_obstacle = obstacles[index_obstacle,:]       

    has_collided = collision_path(x_nearest, x_new, nearest_obstacle)

    return has_collided,x_new
    

def id_search(search_tree,search):
    """Seach for the corresponding ID in search tree given
       a point [x,y] 
    Args:
        - search_tree(numpy.ndarray):[[ID,x,y],...] search tree
        - search(list): [x,y] point to search
    Returns:
        - id_tree(list): The ID that belongs to [x,y] point in search tree
            
    """
    id_tree = math.inf
    for array_tree in search_tree:
        if list(array_tree[1:]) == search:
            id_tree = array_tree[0]  
            break

    return id_tree

    
def short_path(edges):
    """
    Args:
        -edges(numpy.ndarray): [ID1 ID2 cartesian_distance]
    Returns:
        -path(list): free path list from initial ID to goal ID
    """
    path=[]
    goal_id2 = edges[-1,1]
    init_id1 = edges[0,0]
    
    while(goal_id2 != init_id1):
        path.append(goal_id2)
        index_id1 = np.where(edges[:,1]==goal_id2)[0][0]
        goal_id2 = edges[index_id1,0]
    
    path.append(init_id1)
    path.reverse()
    return path

    
    


    
            
            
            
        
    
    
    
    

    
    
    
    
    

    




        
