import colorsys
import time

import matplotlib.pyplot as plt
import matplotlib.axes as pltaxs
from matplotlib.gridspec import GridSpec
import matplotlib.lines as mlines

import math
import numpy as np
import pandas as pd
import pickle

# from beamngpy import BeamNGpy, MeshRoad, Road, Scenario, Vehicle, angle_to_quat
# from beamngpy.sensors import (
    # IMU, Camera, Damage, Electrics, State, Timer)
# from shapely.geometry import Polygon, Point

global t, t_sec, pollFrequency_sec

def my_legendHandler():
    blue_line = mlines.Line2D([], [], ls='--', color='blue', label='Left Edge')
    magenta_line =  mlines.Line2D([], [], ls='--', color='magenta', label='Right Edge')
    black_line = mlines.Line2D([], [], ls='-', color='black', label='Position of Vehicle')
    return blue_line, magenta_line, black_line
    
''' Road Functions '''
def road_maxmin(roadlist, imported):
    left_edge_x1 = np.array([])
    left_edge_y1 = np.array([])
    right_edge_x1 = np.array([])
    right_edge_y1 = np.array([])
    
    for roadstr in roadlist:  
        if imported:
            global roadGeometries
            road_geometry = roadGeometries[roadstr]
        else:
            print(' import the roadGeometries first ')
            break
            #road_geometry = beamng.scenario.get_road_edges(roadstr)    
    
        left_edge_x1 =  np.append(left_edge_x1, np.array([e['left'][0] for e in road_geometry]))
        left_edge_y1 =  np.append(left_edge_y1, np.array([e['left'][1] for e in road_geometry]))
        right_edge_x1 = np.append(right_edge_x1, np.array([e['right'][0] for e in road_geometry]))
        right_edge_y1 = np.append(right_edge_y1, np.array([e['right'][1] for e in road_geometry]))
    
    x_min = min(left_edge_x1.min(), right_edge_x1.min()) - 50  # We add/subtract 10 from the min/max coordinates to pad
    x_max = max(left_edge_x1.max(), right_edge_x1.max()) + 50  # the area of the plot a bit
    y_min = min(left_edge_y1.min(), right_edge_y1.min()) - 10
    y_max = max(left_edge_y1.max(), right_edge_y1.max()) + 10
    
    return x_min, x_max, y_min, y_max

## haven't updated this one to include the colour variables 
def plot_decalRoad(ax, roadlist, title):
    ax.set_title(title, fontsize = 'xx-large', va='bottom', fontweight='medium')
    # ax.set_aspect('equal')
    ax.set_xlim(left=x_min, right=x_max)
    ax.set_ylim(bottom=y_min, top=y_max)
    
    for roadstr in roadlist:  
        road_geometry = beamng.scenario.get_road_edges(roadstr)

        left_edge_x =  np.array([e['left'][0] for e in road_geometry])
        left_edge_y =  np.array([e['left'][1] for e in road_geometry])
        right_edge_x = np.array([e['right'][0] for e in road_geometry])
        right_edge_y = np.array([e['right'][1] for e in road_geometry])
        
        ax.plot(left_edge_x, left_edge_y, 'b-')
        ax.plot(right_edge_x, right_edge_y, 'm-')
        
def x_leftEdge(road_geometry):
    left_edge_x =  np.array([e['left'][0] for e in road_geometry])
    return left_edge_x

def x_rightEdge(road_geometry):
    right_edge_x = np.array([e['right'][0] for e in road_geometry])
    return right_edge_x

def y_leftEdge(road_geometry):
    left_edge_y =  np.array([e['left'][1] for e in road_geometry])
    return left_edge_y

def y_rightEdge(road_geometry):
    right_edge_y = np.array([e['right'][1] for e in road_geometry])
    return right_edge_y
 
def plot_road(ax, roadlist, title, left_colourAndtype, right_colourAndtype, imported):
    ax.set_title(title, fontsize = 'xx-large', va='bottom', fontweight='medium')
    # ax.set_aspect('equal')
    ax.set_xlim(left=x_min, right=x_max)
    ax.set_ylim(bottom=y_min, top=y_max)
    
    road_leftEdges = []
    road_rightEdges = []
    
    for roadstr in roadlist:  
        if imported:
            global roadGeometries
            road_geometry = roadGeometries[roadstr]
        else:        
            print(' import the roadGeometries first ')
            break
            #roadGeometries = {}
            #road_geometry = beamng.scenario.get_road_edges(roadstr)
            #roadGeometries[roadstr] = road_geometry
    
        if roadstr in roadlist[2:]:
            #print("Overlapping road edges")
            # Straights
            leftEdge_x_straights = np.append(x_leftEdge(roadGeometries.get(roadlist[0])), x_leftEdge(roadGeometries.get(roadlist[1])))
            leftEdge_y_straights = np.append(y_leftEdge(roadGeometries.get(roadlist[0])), y_leftEdge(roadGeometries.get(roadlist[1])))
            rightEdge_x_straights = np.append(x_rightEdge(roadGeometries.get(roadlist[0])), x_rightEdge(roadGeometries.get(roadlist[1])))
            rightEdge_y_straights = np.append(y_rightEdge(roadGeometries.get(roadlist[0])), y_rightEdge(roadGeometries.get(roadlist[1])))        
            # ax.plot(leftEdge_x_straights, leftEdge_y_straights, 'r.')
            # ax.plot(rightEdge_x_straights, rightEdge_y_straights, 'r.')
            
            # loop edges        
            leftEdges = np.column_stack((x_leftEdge(road_geometry), y_leftEdge(road_geometry)))
            rightEdges = np.column_stack((x_rightEdge(road_geometry), y_rightEdge(road_geometry)))
            
            # Exclusion zone
                # boundingBox_XXX = [A, B, C, D] refer to notes
            bBox_Front =  [(min(leftEdge_x_straights), max(leftEdge_y_straights)), 
                           (min(rightEdge_x_straights), max(rightEdge_y_straights)), 
                           (min(leftEdge_x_straights), min(leftEdge_y_straights)),
                           (min(rightEdge_x_straights), min(rightEdge_y_straights))]
            
            bBox_Back  =  [(max(rightEdge_x_straights), max(leftEdge_y_straights)), 
                           (max(leftEdge_x_straights), max(rightEdge_y_straights)), 
                           (max(rightEdge_x_straights), min(leftEdge_y_straights)),
                           (max(leftEdge_x_straights), min(rightEdge_y_straights))]
            # Inclusion zone
                # inclusion condition
            if roadstr == roadlist[2]:     
                condition = (leftEdges[:,0] >= bBox_Front[0][0]) & \
                            (leftEdges[:,1] >= bBox_Front[1][1])
            else:
                condition = (leftEdges[:,0] >= bBox_Front[2][0]) & \
                            (leftEdges[:,1] <= bBox_Front[2][1])
            right_edge_included = rightEdges[condition]
            left_edge_included = leftEdges[condition]
                # sort for plotting
            left_edge_included = left_edge_included[np.lexsort((left_edge_included[:,1], left_edge_included[:,0]))]
            right_edge_included = right_edge_included[np.lexsort((right_edge_included[:,1], right_edge_included[:,0]))]
                # plot
            ax.plot(right_edge_included[:,0], right_edge_included[:,1], right_colourAndtype, label="Right Edge")
            ax.plot(left_edge_included[:,0], left_edge_included[:,1], left_colourAndtype, label="Left Edge")
            
            # Add to overall road edges
    
            road_leftEdges = np.concatenate((road_leftEdges, left_edge_included), axis=0)
            road_rightEdges = np.concatenate((road_rightEdges, right_edge_included), axis=0)
            
                # Edges_nosort = np.concatenate((left_edge_included, right_edge_included), axis = 0)
                # Edges = Edges_nosort[np.lexsort((Edges_nosort[:,1], Edges_nosort[:,0]))]
                # ax.plot(Edges[:,0], Edges[:,1], 'r.')
                # print(Edges)
            
            
        else:
            #print("Road edges as normal")       
            leftEdges = np.column_stack((x_leftEdge(road_geometry), y_leftEdge(road_geometry)))
            rightEdges = np.column_stack((x_rightEdge(road_geometry), y_rightEdge(road_geometry)))
                # 
            ax.plot(leftEdges[:,0], leftEdges[:,1], left_colourAndtype, label="Left Edges")
            ax.plot(rightEdges[:,0], rightEdges[:,1], right_colourAndtype, label= "Right Edges")
                # Add to overall road edges
            if len(road_leftEdges) == 0:
                road_leftEdges  = leftEdges
                road_rightEdges = rightEdges
            else:
                road_leftEdges = np.concatenate((road_leftEdges, leftEdges), axis=0)
                road_rightEdges = np.concatenate((road_rightEdges, rightEdges), axis=0)
                
    return(road_leftEdges, road_rightEdges, roadGeometries)



''' States and Electrics '''

def plot_position(axP, formatPos, labels):
    x = [p[0] for p in positions]
    y = [p[1] for p in positions]
    axP.plot(x, y, formatPos, label="Position of Vehicle")
    if labels:
        axP.set_title('Position of Vehicle over time', fontsize='x-large')
        axP.set_xlabel('x coordinate', fontweight='semibold')
        axP.set_ylabel('y coordinate', fontweight='semibold')
        

    
def plot_speed(axS, formatSpeed):
    #t = np.linspace(1,t_sec,t_sec/pollFrequency_sec)
    axS.plot(t, wheelSpeed, formatSpeed)
    axS.set_title("Forward velocity over time", fontsize='x-large')
    axS.set_xlabel('time [s]', fontweight='semibold')
    axS.set_ylabel('speed [m/s]', fontweight='semibold')
    axS.set_xlim([0-0.5,t_sec+0.5])
    axS.grid(which='both',  linestyle='--') 
    
def plot_throttle(axT, formatThrot):
    axT.plot(t,throttle, formatThrot, label = 'Throttle')
    axT.set_title('Throttle input', fontsize='x-large')
    axT.set_ylabel('Intensity', fontweight='semibold')
    axT.set_xlabel('Time [s]', fontweight='semibold')
    axT.set_ylim([0.0,1.01])
    axT.grid(which='major', axis='x', linestyle='--') 
    
def plot_steering(axSt, formatSteer):
    axSt.plot(t,steering, formatSteer, label = 'Steering')
    axSt.set_title('Steering input', fontsize='x-large')
    axSt.set_ylabel('Angle [degrees]', fontweight='semibold')
    axSt.set_xlabel('Time [s]', fontweight='semibold')
    axSt.set_ylim([-180,180])
    axSt.grid(which='both', linestyle='--') 
    
def plot_brake(axB, formatBrake):
    axB.plot(t,brakes, formatBrake, label = 'Brake')
    axB.set_title('Brake input', fontsize='x-large')
    axB.set_ylabel('Intensity', fontweight='semibold')
    axB.set_xlabel('Time [s]', fontweight='semibold')
    axB.set_ylim([0.0,1.01])  
    axB.grid(which='major', axis='x', linestyle='--') 

# def plot_headingVectors(axD):
#     for i in range(directions.size):
#         thisTimeDirections = directions[i]
        
#         if type(thisTimeDirections) == str:
#             directions = thisTimeDirections[1:-2].split(",")
#         elif type(thisTimeDirections) == list:
#             directions = thisTimeDirections
        
        
        
#         headingAngle_degrees[i] = calc_HeadingAngle(thisTimeDirections)  
    
#     for d in directions:
        
#         x = float(directions[0])
#         yr = float(directions[1])
        
#     x = [d[0] for d in directions]
#     y = [d[1] for d in directions]
#     axD.plot(t, x, 'm-', label="Heading in X")
#     axD.plot(t, y, 'c-', label="Heading in Y")
#     axD.set_title("Heading vectors", fontsize='x-large')
#     axD.set_xlabel('time [s]', fontweight='semibold')
#     axD.set_ylabel('radians', fontweight='semibold')
#     axD.legend(("x direction", "y-direction"), loc = 'lower left')
#     axD.grid(which='both',  linestyle='--')

    
def calc_HeadingAngle(thisTimeDirections):
    if type(thisTimeDirections) == str:
        directions = thisTimeDirections[1:-2].split(",")
    elif type(thisTimeDirections) == list:
        directions = thisTimeDirections
    xdir = float(directions[0])
    ydir = float(directions[1])
    vec = np.array([xdir,ydir])
    
    uX = (vec/np.linalg.norm(vec))[0]
    uY = (vec/np.linalg.norm(vec))[1]
    
    radians = math.atan2(uX,uY)

    thisHeadingAngle_deg = radians * (180 / math.pi)
    #thres = -1
    # if thisHeadingAngle_deg < thres:
    #    thisHeadingAngle_deg = (thisHeadingAngle_deg + 180) + 180
    ##
    
    return thisHeadingAngle_deg

def import_data(inputFile, headingAngleMode = False):
    import_data = pd.read_csv(inputFile)

    mu = np.array(import_data[['time', 'throttle', 'steering', 'brakes']])
    eta = np.array(import_data[['time','x_pos', 'y_pos', 'velocity', 'directions']])


    # Saving to mu (inputs)
    t = mu[:,0]
    throttle = mu[:,1]
    steering = mu[:,2]
    brakes = mu[:,3]
    # and eta (states)
    x_pos =  eta[:,1]
    y_pos = eta[:,2]
    wheelSpeed = eta[:,3]
    directions = eta[:,4]

    ## Heading Angle or Heading Vector    
    # Turning Directions into Heading Angle
    headingAngle_degrees = np.empty(directions.size)
    for i in range(directions.size):
        thisTimeDirections = directions[i]
        headingAngle_degrees[i] = calc_HeadingAngle(thisTimeDirections)     
    #
    if headingAngleMode:
        # Replace Directions with Heading Angle
        eta = np.column_stack((eta[:,0:-1], headingAngle_degrees))
    else:
        # split directions into heading Vectors
        x = [d[0] for d in directions]
        y = [d[1] for d in directions]
        # Replace Directions with Heading Vectors
        eta = np.column_stack((eta[:,0:-1], x_head, y_head))

    positions = np.column_stack((x_pos, y_pos))
    print(f' finished importing the file from {inputFile}')
    #return import_data, mu, eta, t, throttle, steering, brakes, x_pos, y_pos, wheelSpeed, headingAngle_degrees, positions
    return mu, eta, t, throttle, steering, brakes, x_pos, y_pos, wheelSpeed, headingAngle_degrees, positions, x_head, y_head