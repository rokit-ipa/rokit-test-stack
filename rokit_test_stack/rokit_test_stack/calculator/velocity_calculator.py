import math

def distance_vector(startpose, endpose):
    (x1,y1,z1) = startpose
    (x2,y2,z2) = endpose
    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2+ (z2-z1)**2)
    return distance

def calculate(startpose, startframe, endpose, endframe): 
    time_seconds = (endframe - startframe)*0.01
    distance=distance_vector(startpose, endpose)
    distance_in_metres = distance/1000
    metres_per_second = distance_in_metres/time_seconds
    return metres_per_second    
    
    
        