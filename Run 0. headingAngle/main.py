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


if True: # Enable True if on BeamNGPy is installed. 
    from beamngpy import BeamNGpy, MeshRoad, Road, Scenario, Vehicle, angle_to_quat
    from beamngpy.sensors import (
        IMU, Camera, Damage, Electrics, State, Timer)
    from shapely.geometry import Polygon, Point


# Setting constants
global t, t_sec, pollFrequency_sec
t_sec = 60
pollFrequency_sec = 0.1
t = np.linspace(0,t_sec,int(t_sec/pollFrequency_sec))
# Run Functions
exec(open("emFunctions.py").read())

#% Start BeamNG
# Run a setup_XXXX.py 
map_py_fileName = "setup_straightsAndCircles.py"
exec(open(map_py_fileName).read())

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
beamng = BeamNGpy('localhost', 64256, home='C:/Users/Em/Documents/0. Honours/BeamNG.tech.v0.27.2.0')
beamng.open()
scenario.make(beamng)

# Start Scenario 
beamng.settings.set_deterministic(60) # sets such that 60 steps  = 1 second
beamng.scenario.load(scenario)
beamng.scenario.start()

# Vehicle Settings
# vehicle.connect(vehicle) # I don't think I need connect until I want to transmit
sensors = vehicle.sensors
#%% Gather States
if 'ID' not in globals():
    ID = 0
ID += 1

beamng.scenario.restart()
roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']

    # Data: Position
positions = list() 
directions = list()
wheelSpeed = list() 
brakes = list()
steering = list()
throttle = list()
    # brakes_input = list()  # steering_input = list()  # throttle_input = list()
vehicle.set_shift_mode('realistic_automatic')

for _ in range(int(t_sec/pollFrequency_sec)): # range(n) 2 x n = duration of polling
    time.sleep(pollFrequency_sec) 
    vehicle.sensors.poll() # Polls the data of all sensors attached to the vehicle
    positions.append(vehicle.state['pos'])
    directions.append(vehicle.state['dir'])
    wheelSpeed.append(sensors['electrics']['wheelspeed'])
    brakes.append(sensors['electrics']['brake'])
    steering.append(sensors['electrics']['steering'])
    throttle.append(sensors['electrics']['throttle'])
   # break this if the scenario is reset.      
        
#%% Live Data
if False:
    forHowlong = int(t_sec/pollFrequency_sec)
else:
    forHowLong = int(5)
    
for _ in range(forHowLong):
    time.sleep(pollFrequency_sec)
    vehicle.sensors.poll()
    
    if False:
        directions = vehicle.state['dir']
        dirx = directions[0] 
        diry = directions[1]
        dirz = directions[2]
        print(f'x is {dirx} and y is {diry}')
    
    if True:
        position = vehicle.state['pos']
        posx = position[0] 
        posy = position[1]
        posz = position[2]
        print(f'x is {posx}, y is {posy} and z is {posz}')
      
#%% Programatically Driving
exec(open("emFunctions.py").read())

K = np.reshape(thisResult.get('x'), [3,4])

## Vehicle Set Up
if False:
    vehicle.teleport((0,60,0.25),None, True)
    time.sleep(pollFrequency_sec)
vehicle.set_shift_mode('arcade')
time.sleep(pollFrequency_sec)
vehicle.control(steering = None, throttle = None, brake = None, parkingbrake = 0, clutch = None, gear = 3) # gear = S1

states = All_zeta[0,:]
# Init Saves
graphData = [[],[],[], [],[],[] ]
graphThrottle = []
graphBrake = []
 ###list("positions")],[list("brake")],[list("steering")],[list("posX")],[list("posY")],[list("velocity")

for t in range(int(t_sec/pollFrequency_sec)):
    # Calculate the predicted inputs using the given state
    inputs = np.matmul(K, states)
    
    throttle = inputs[0] # Throttle has a range: [0,  1]
    steering = inputs[1] # Steering has a range: [-1, 1]
    brake = inputs[2]    # Brake has a range of: [0,  1]
    
    # saturate inputs to [0, 1]
    # where 0 |-> -1 
    # use threshold e-6
    threshold_zero = 1E-3 
    threshold_brakeSuppress = 0.5
    for thisValue in [throttle, steering, brake]:
        if thisValue == inputs[0]:
            throttle = abs(throttle)
            if throttle <= threshold_zero:
                throttle = -1
        elif thisValue == inputs[1]:
            continue
        elif thisValue == inputs[2]:
            brake = abs(brake)
            if brake <= threshold_zero:
                brake = -1
            brake = threshold_brakeSuppress * brake
        else:
            print("we have a problem")
        
    # Use predicted inputs to drive the vehicle and then poll for observed data
    vehicle.control(steering, throttle, -1)
    time.sleep(pollFrequency_sec)
    vehicle.sensors.poll()
    
    debug = True
    if debug:
        throttlePolled = sensors['electrics']['throttle']
        brakePolled = sensors['electrics']['brake']
        
        print(f' Throttle is {throttlePolled} with input {throttle} \n' +
              f'Brake is {brakePolled} with input {brake} ')
        
    
    # Organise the observed data and save into states array.
    positions = vehicle.state['pos']
    posx = positions[0] 
    posy = positions[1]
    velocity = sensors['electrics']['wheelspeed']
    bearing = calc_HeadingAngle(vehicle.state['dir'])
    
    states = np.array([posx, posy, velocity, bearing])
    
    ## Add data into arrays to be plotted.
    #graphThrottle = np.row_stack(graphThrottle,throttle)
    #graphBrake = np.row_stack(graphBrake, brake)    
    
## TODO: edit the plotting functions 



    #%% Test Driving
beamng.scenario.restart()
vehicle.connect(beamng)
vehicle.set_shift_mode('arcade')
vehicle.control(gear = 2) 
vehicle.control(0, 1, -1) 
time.sleep(5)
vehicle.control(0,0.5,0.2) 
time.sleep(3)
vehicle.control(0,0.1,-1) 
time.sleep(2)
vehicle.control(0,-1,0.5) 
time.sleep(2)
vehicle.control(0,-1,1) 
time.sleep(3)
vehicle.control(0,-1,-1) 
time.sleep(1)
vehicle.control(0, 1, -1) 
time.sleep(5)
vehicle.control(1,0.5,0)

    
#%% Export Data
columnNames = ['time', 'throttle', 'steering', 'brakes', 'x_pos', 'y_pos', 'velocity', 'directions']

x_pos = [p[0] for p in positions]
y_pos = [p[1] for p in positions]


export_data = np.array([t, throttle, steering, brakes, x_pos, y_pos, wheelSpeed, directions])
states = pd.DataFrame(np.transpose(export_data), columns = columnNames) 

outputFile = f'{scenario.name}_{ID}.csv'

states.to_csv(outputFile, index=False, lineterminator='\r\n')
#%% Import Data
    # testing with ID=12
inputFile = 'Saves/straightsAndcircles_12.csv'
[mu, eta, t, throttle, steering, brakes, x_pos, y_pos, wheelSpeed, headingAngle_degrees, positions, x_head, y_head] = import_data(inputFile)

#% Import Road Dictionary
global roadGeometries
filePath = 'straightsAndcircles_roadGeometries.pkl'

with open (filePath, 'rb') as fp:
    roadGeometries = pickle.load(fp)


#% Combined Plots (6 Plots)
t = np.linspace(0,t_sec,int(t_sec/pollFrequency_sec))

# Init Graphs
fig = plt.figure(figsize= (14,12), layout="constrained")
#fig.suptitle(f'{scenario.name} {ID}', fontsize='xx-large', fontweight='extra bold')  
gs = GridSpec(3, 2, figure= fig)
axP  = fig.add_subplot(gs[0:2, 0])
axS  = fig.add_subplot(gs[2, 0])
axT  = fig.add_subplot(gs[0,1])
axSt = fig.add_subplot(gs[1,1])
axH  = fig.add_subplot(gs[2,1])

# Format
lineFormat_position = 'k-'
lineFormat_leftEdge = 'b--'
lineFormat_rightEdge = 'm--'

# Plot
imported = True
plot_speed(axS, 'k-')
plot_throttle(axT,'g-')
plot_steering(axSt,'b-')
plot_brake(axT,'r-')
axT.set_title(' Throttle and Brake input ', fontsize = 'x-large')
plot_headingVectors(axH)
[ x_min, x_max, y_min, y_max] = road_maxmin(roadlist, imported)
[road_leftEdges, road_rightEdges, roadGeometries] = plot_road(axP, roadlist, 'Vehicle Position and Road Edges', lineFormat_leftEdge, lineFormat_rightEdge, imported)
plot_position(axP, lineFormat_position, False)
[left, right, pos] = my_legendHandler()

axP.legend(handles=[left, right, pos], loc='center', bbox_to_anchor=(.78, 0.051))
axP.set_xlabel('x coordinates', fontsize='x-large', fontweight='semibold')
axP.set_ylabel('y coordinates', fontsize='x-large', fontweight='semibold')


#outputIMG = f'{scenario.name}_{ID}.png'
#fig.savefig(outputIMG)

#%% Report Image: Position and Input Plots. 
# Init Graphs
fig = plt.figure(figsize= (10,14), layout="constrained")
gs = GridSpec(4, 2, figure= fig)
axP  = fig.add_subplot(gs[0:2, 0])
axS  = fig.add_subplot(gs[3,0:2])
axD = fig.add_subplot(gs[0,1])
axT  = fig.add_subplot(gs[2,0])
axSt = fig.add_subplot(gs[1,1])
axB  = fig.add_subplot(gs[2,1])

# Plot
plot_position(axP,'k-', labels=True)
plot_speed(axS, 'k-')
plot_headingVector(axD)
plot_throttle(axT,'g-')
plot_steering(axSt,'b-')
plot_brake(axB,'r-')

#%% Report Image: Plotting road edges, comparing as exported vs the truncated version. 
    # Initialise
roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']
left_colourAndtype = 'b-'
right_colourAndtype = 'm-'
fig, (ax, ax2) = plt.subplots(1,2, sharex='all', sharey='all', figsize=(10,8))
    # Plot using functions
[ x_min, x_max, y_min, y_max] = road_maxmin(roadlist, True)
[road_leftEdges, road_rightEdges, roadGeometries] = plot_road(ax2, roadlist, 'Truncated for Simple Loop', left_colourAndtype, right_colourAndtype, True)
plot_decalRoad(ax, roadlist, 'As exported')
#
    # Add -o to indicate points of road edges
addPoints = False
if addPoints:
    ax2.scatter(road_leftEdges[:,0], road_leftEdges[:,1])
    ax2.scatter(road_rightEdges[:,0], road_rightEdges[:,1], c='m')
    # Plot Formatting
fig.legend(['left edge', 'right edge'], loc='center', bbox_to_anchor=(0.84, 0.045))
fig.suptitle('Road Edges', fontsize='xx-large', fontweight='semibold')
fig.supxlabel('x coordinates', fontsize='x-large', fontweight='semibold')
fig.supylabel('y coordinates', fontsize='x-large', fontweight='semibold')



#%% Report Image: Plotting position of vehicle w.r.t road edges. 
lineFormat_position = 'k-'
lineFormat_leftEdge = 'b--'
lineFormat_rightEdge = 'm--'

fig2, ax3 = plt.subplots(figsize=(10, 12))
[road_leftEdges, road_rightEdges, roadGeometries] = plot_road(ax3, roadlist, 'Vehicle Position and Road Edges', lineFormat_leftEdge, lineFormat_rightEdge, True)
plot_position(ax3, lineFormat_position, False)
[left, right, pos] = my_legendHandler()

fig2.legend(handles=[left, right, pos], loc='center', bbox_to_anchor=(.78, 0.048))
fig2.supxlabel('x coordinates', fontsize='x-large', fontweight='semibold')
fig2.supylabel('y coordinates', fontsize='x-large', fontweight='semibold')