from RRTPlanner import collision_path

# one segment-point inside circle 
def test_collision_1():
    assert(collision_path([1,1],[2,3],[4,4,6])==True)

# segment outside circle
def test_collision_2():
    assert(collision_path([6,1],[8,4],[4,4,6])==False)

# segment inside circle
def test_collision_3():
    assert(collision_path([4,2],[6,4],[4,4,6])==True)

# segment colineal with circle center
def test_collision_4():
    assert(collision_path([2,2],[6,6],[4,4,6])==True)
    
# vertical line segment outside circle
def test_collision_5():
    assert(collision_path([10,0],[10,-4],[4,4,6])==False)

# horizontal line segment outside circle
def test_collision_6():
    assert(collision_path([6,8],[10,8],[4,4,6])==False)

# collison out of line segment
def test_collision_7():
    assert(collision_path([8,4],[10,5],[4,4,6])==False)

# collison out of line segment - vertical line
def test_collision_8():
    assert(collision_path([2,8],[2,12],[4,4,6])==False)



