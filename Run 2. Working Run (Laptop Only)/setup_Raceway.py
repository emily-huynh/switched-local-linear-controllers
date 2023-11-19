import colorsys
import time

import matplotlib.pyplot as plt
import matplotlib.axes as pltaxs
from matplotlib.gridspec import GridSpec
import matplotlib.lines as mlines

import numpy as np
from beamngpy import BeamNGpy, MeshRoad, Road, Scenario, Vehicle, angle_to_quat
from beamngpy.sensors import (
    IMU, Camera, Damage, Electrics, State, Timer)
from shapely.geometry import Polygon, Point

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
beamng = BeamNGpy('localhost', 64256, home='C:/Users/Em/Documents/0. Honours/BeamNG.tech.v0.27.2.0')
beamng.open()

global t, t_sec
t_sec = 60
t = np.linspace(1,t_sec,2*t_sec)

# Road
w = 15 # road width
radius = 100
length = 300
road45 =  (1-np.cos(np.pi/4))*radius
road30 =  (1-np.cos(np.pi/6))*radius


loopOne = Road('track_editor_C_center', rid='loopOne', looped=True, smoothness=1)
loopOne.add_nodes(
    (0, length, 0, w),
    (radius, radius+length, 0, w),
    (radius*2, length, 0, w),
    (radius, length-radius, 0, w)
    )
straightFront = Road('track_editor_C_center', rid='straightFront')
straightFront.add_nodes(
    (0, 0, 0, w),
    (0, length, 0, w)
    )
straightBack = Road('track_editor_C_center', rid='straightBack')
straightBack.add_nodes(
    (radius*2, length, 0, w),
    (radius*2, 0, 0, w)
    )
loopTwo = Road('track_editor_C_center', rid='loopTwo', looped=True, smoothness=1)
loopTwo.add_nodes(
    (radius*2, 0, 0, w),
    (radius, -radius, 0, w),
    (0, 0, 0, w),
    (radius, radius, 0, w)
    )

loopFull = Road('track_editor_C_center', rid='loopFull', looped=True, smoothness=1)
loopFull.add_nodes(
    (0, 0, 0, w),
    (0, length, 0, w),
    (radius, radius+length, 0, w),
    (radius*2, length, 0, w),
    (radius*2, 0, 0, w),
    (radius, -radius, 0, w),
    (0, 0, 0, w)
    )

# Vehicle Settings
electrics = Electrics() ; timer = Timer()
    #
vehicle = Vehicle('VehicleID', model='vivace', license='Emily') # , option='arcade'
vehicle.sensors.attach('electrics', electrics)
vehicle.sensors.attach('timer', timer)

# Scenario 
scenario = Scenario('smallgrid', 'straightsAndcircles')
scenario.add_road(loopOne)
scenario.add_road(straightFront)
scenario.add_road(straightBack)
scenario.add_road(loopTwo)
scenario.add_vehicle(vehicle, pos=(0, -10, 0), rot_quat=angle_to_quat((0,0,180)))
scenario.make(beamng)


# Scenario Start
beamng.settings.set_deterministic(60) # sets such that 60 steps  = 1 second
beamng.scenario.load(scenario)
beamng.scenario.start()

# Vehicle Settings
# vehicle.connect(vehicle) # I don't think I need connect until I want to transmit
vehicle.set_shift_mode('arcade')
sensors = vehicle.sensors
#%% Reload and Restart
# beamng.scenario.load(scenario)
beamng.settings.set_deterministic(60) # sets such that 60 steps  = 1 second
beamng.scenario.restart()
#%% Gather States
beamng.scenario.restart()

    # Data: Position
positions = list() ; 
wheelSpeed = list() 
brakes = list()
steering = list()
throttle = list()
    # directions = list()   # brakes_input = list()  # steering_input = list()  # throttle_input = list()

t_sec = 60
pollFrequency_sec = 0.25
for _ in range(int(t_sec/pollFrequency_sec)): # range(n) 2 x n = duration of polling
    time.sleep(pollFrequency_sec) 
    vehicle.sensors.poll() # Polls the data of all sensors attached to the vehicle
    positions.append(vehicle.state['pos'])
    wheelSpeed.append(sensors['electrics']['wheelspeed'])
    brakes.append(sensors['electrics']['brake'])
    steering.append(sensors['electrics']['steering'])
    throttle.append(sensors['electrics']['throttle'])
    # The following were empirically deemed not useful.
    # brakes_input.append(sensors['electrics']['brake_input'])
    # steering_input.append(sensors['electrics']['steering_input'])            
    # throttle_input.append(sensors['electrics']['throttle_input'])   
    # directions.append(vehicle.state['dir'])
    
#%% Report Image: Position and Input Plots. 
# Init Graphs
fig = plt.figure(figsize= (10,12), layout="constrained")
gs = GridSpec(3, 2, figure= fig)
axP  = fig.add_subplot(gs[0:2, 0])
axS  = fig.add_subplot(gs[2, 0])
axT  = fig.add_subplot(gs[0,1])
axSt = fig.add_subplot(gs[1,1])
axB  = fig.add_subplot(gs[2,1])

# Plot
plot_position(axP,'k-', labels=True)
plot_speed(axS, 'k-')
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
[ x_min, x_max, y_min, y_max] = road_maxmin(roadlist)
[road_leftEdges, road_rightEdges] = plot_road(ax2, roadlist, 'Truncated for Simple Loop', left_colourAndtype, right_colourAndtype)
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
[road_leftEdges, road_rightEdges] = plot_road(ax3, roadlist, 'Vehicle Position and Road Edges', lineFormat_leftEdge, lineFormat_rightEdge)
plot_position(ax3, lineFormat_position, False)
[left, right, pos] = my_legendHandler()

fig2.legend(handles=[left, right, pos], loc='center', bbox_to_anchor=(.78, 0.048))
fig2.supxlabel('x coordinates', fontsize='x-large', fontweight='semibold')
fig2.supylabel('y coordinates', fontsize='x-large', fontweight='semibold')

#%% Combined Plots (6 Plots)
# Init Graphs
fig = plt.figure(figsize= (14,12), layout="constrained")
gs = GridSpec(3, 2, figure= fig)
axP  = fig.add_subplot(gs[0:2, 0])
axS  = fig.add_subplot(gs[2, 0])
axT  = fig.add_subplot(gs[0,1])
axSt = fig.add_subplot(gs[1,1])
axB  = fig.add_subplot(gs[2,1])

# Format
lineFormat_position = 'k-'
lineFormat_leftEdge = 'b--'
lineFormat_rightEdge = 'm--'

# Plot
[road_leftEdges, road_rightEdges] = plot_road(axP, roadlist, 'Vehicle Position and Road Edges', lineFormat_leftEdge, lineFormat_rightEdge)
plot_position(axP, lineFormat_position, False)
[left, right, pos] = my_legendHandler()
plot_speed(axS, 'k-')
plot_throttle(axT,'g-')
plot_steering(axSt,'b-')
plot_brake(axB,'r-')

axP.legend(handles=[left, right, pos], loc='center', bbox_to_anchor=(.78, 0.051))
axP.set_xlabel('x coordinates', fontsize='x-large', fontweight='semibold')
axP.set_ylabel('y coordinates', fontsize='x-large', fontweight='semibold')