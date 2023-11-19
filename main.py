import time
import matplotlib.pyplot as plt
import matplotlib.axes as pltaxs
from matplotlib.gridspec import GridSpec
import numpy as np
import pandas as pd
import pickle


# import emFunctions as 
from emFunctions import import_data, centreLineOfRoad, calc_XFromCentre, segment_LegendHandler
global timeBasedSegment, length, K1, K2

homePC = True
if homePC: # Enable True if on BeamNGPy is installed. 
    from beamngpy import BeamNGpy, MeshRoad, Road, Scenario, Vehicle, angle_to_quat
    from beamngpy.sensors import (
        IMU, Camera, Damage, Electrics, State, Timer)
    # from shapely.geometry import Polygon, Point
    electrics = Electrics() ; timer = Timer() ;

# Setting constants
global t, t_sec, pollFrequency_sec, roadList
t_sec = 60
pollFrequency_sec = 0.1
t = np.linspace(0,t_sec,int(t_sec/pollFrequency_sec))
# Run Functions
exec(open("emFunctions.py").read())

#% Start BeamNG
if homePC:   
    # Run a setup_XXXX.py 
    map_py_fileName = "setup_straightsAndCircles.py"
    exec(open(map_py_fileName).read())
else:
    global roadGeometries
    filePath = 'straightsAndcircles_roadGeometries.pkl'

    with open (filePath, 'rb') as fp:
        roadGeometries = pickle.load(fp)

    roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']

#%
# Instantiate BeamNGpy
if homePC:
    beamng = BeamNGpy('localhost', 64256, home= 'C:/Users/Em/Documents/0. Honours/BeamNG.tech.v0.27.2.0')
    beamng.open()
    scenario.make(beamng)
    
    # Start Scenario 
    beamng.settings.set_deterministic(60) # sets such that 60 steps  = 1 second
    beamng.scenario.load(scenario)
    beamng.scenario.start()
    
    # Vehicle Settings
    sensors = vehicle.sensors
#%% Gather States
if 'ID' not in globals():
    ID = 30
ID += 1

beamng.scenario.restart()
roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']

    # Data: Position
positions = list() 
directions = list()
velocity = list()
brakes = list()
steering = list()
throttle = list()

    # brakes_input = list()  # steering_input = list()  # throttle_input = list()
vehicle.set_shift_mode('arcade')
time.sleep(pollFrequency_sec)
vehicle.control(steering = None, throttle = None , brake = None, parkingbrake = 0, clutch = None, gear = 3) # gear = S1
time.sleep(pollFrequency_sec)

test_sec = 60

t = np.linspace(0,test_sec,int(test_sec/pollFrequency_sec))

for _ in range(int(test_sec/pollFrequency_sec)): 
    time.sleep(pollFrequency_sec) 
    vehicle.sensors.poll() # Polls the data of all sensors attached to the vehicle
    positions.append(vehicle.state['pos'])
    directions.append(vehicle.state['dir'])
    velocity.append(vehicle.state['vel'])
    brakes.append(sensors['electrics']['brake'])
    steering.append(sensors['electrics']['steering'])
    throttle.append(sensors['electrics']['throttle'])
    #
    
   # break this if the scenario is reset.      
        
#%% Live Data
if False:
    forHowlong = int(t_sec/pollFrequency_sec)
else:
    forHowLong = int(10000)
    
for _ in range(forHowLong):
    time.sleep(pollFrequency_sec*3)
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
        # print(f'x is {posx}, y is {posy} and z is {posz}')
        centreLine = centreLineOfRoad()
        [xFromCentre,_] = calc_XFromCentre(np.array([posx,posy]), centreLine)
        print(f' distance from centreline is {xFromCentre:0.3f}')
    
    if True:
        steering = sensors['electrics']['steering']
        print(f' steering is {steering:0.3f} \n')
        
      
#%% Programatically Driving
import emFunctions
exec(open("emFunctions.py").read())
global postProcess_steering, firstK, K_1, K_2
beamng.scenario.restart()

## Vehicle Set Up
if False:
    vehicle.teleport((0,0,0.20),None, True)
    time.sleep(pollFrequency_sec)
vehicle.control(steering = None, throttle = None , brake = None, parkingbrake = 0, clutch = None, gear = 3) # gear = S1
beamng.settings.set_deterministic(60)
time.sleep(pollFrequency_sec)
vehicle.set_shift_mode('arcade')

states = All_zeta[0,:]
plotData = True

if plotData:
    graph_positions = list() 
    graph_positionColour = list()
    graph_directions = list()
    graph_wheelSpeed = list() 
    graph_brakes = list()
    graph_steering = list()
    graph_throttle = list()

K1_test_sec = 30
K2_test_sec = 30
if timeBasedSegment:
    test_sec = K1_test_sec + K2_test_sec
else:
    test_sec = 50
    
for t in range(int(test_sec/pollFrequency_sec)):
    
    # Time Based Segment control
    if timeBasedSegment:
        if t <= (int(K1_test_sec/pollFrequency_sec)):
            K = Kt1
        else:
            K = Kt2
            
    # Location Based Segmenting
      # wait n seconds 
    segmentThres = length + 30  
    n_sec = 1
    global a, b, c, d
    if t > int(n_sec/pollFrequency_sec):
        posy = positions[1]
        posx = positions[0]        
        if posx <= 100 and (a*posx - b) <= posy <= 300:
            K = K1
            Kgain = "K1"
        elif posy >= 300  and posy >= a*posx + (300-b):
            K = K2 
            Kgain = "K2"
        elif posx >= 100 and c <= posy <= a*posx + (300-b):
            K = K3
            Kgain = "K3"
        elif posy <= c and posy <= (a*posx - b):
            K = K4
            Kgain = "K4"

    else: 
         K = K1   
         Kgain = "K1"
    
    # Calculate the predicted inputs using the given state
    inputs = np.matmul(K, states)
    
    throttle = inputs[0] # Throttle has a range: [0,  1]
    steering = inputs[1] # Steering has a range: [-1, 1]
    brake = inputs[2]    # Brake has a range of: [0,  1]
    
    # saturate inputs to [0, 1]
        # where 0 |-> -1 
        # use threshold e-3
    threshold_zero = 1E-4 
    
    for thisValue in [throttle, steering, brake]:
        if thisValue == inputs[0]:
            throttle = abs(throttle) 
            if throttle <= threshold_zero:
                throttle = -1
        elif thisValue == inputs[1]:
            if postProcess_steering == False:
                steering = -steering * (1/470)
            continue
        elif thisValue == inputs[2]:
            brake = abs(brake)
            if brake <= threshold_zero:
                brake = -1
        else:
            print("we have a problem")
    
        
    # Use predicted inputs to drive the vehicle and then poll for observed data
    vehicle.control(steering, throttle, -1)
    time.sleep(pollFrequency_sec)
    vehicle.sensors.poll()
    
    debug = True
    if debug:
        steeringPolled = sensors['electrics']['steering']
        throttlePolled = sensors['electrics']['throttle']
        brakePolled = sensors['electrics']['brake']
        
        steeringPrint = steering * 470
        print(f'Using {Kgain} \n' +
              f'  y pos = {posy:.1f} \n'
              f'  Throttle is {throttlePolled:.3f} with input {throttle:.3f} \n' +
              f'  Brake is {brakePolled:.3f} with input {brake:.3f} \n' +
              f'  Steering is {steeringPolled:.3f} with input {steeringPrint:.3f} ')
    
    # New states
    if xFromCentreMode and headingAngleMode == False:
        roadList = ['straightFront', 'loopOne', 'straightBack', 'loopTwo']
        # Organise the observed data and save into states array.
        positions = vehicle.state['pos']
        directions = vehicle.state['dir']
        vel = vehicle.state['vel']
        
        centreLine = centreLineOfRoad()
        [xFromCentre,_] = calc_XFromCentre(np.array([positions[0],positions[1]]), centreLine)
                            
        xheading = directions[0] 
        yheading = directions[1]
        
        velocity = math.sqrt(vel[0]**2+vel[1]**2)
        
        states = np.array([xFromCentre, velocity, xheading, yheading])
        
    elif xFromCentreMode and headingAngleMode:
        # Organise the observed data and save into states array.
        print('write this part lol')    
    elif headingAngleMode == False:
        # Organise the observed data and save into states array.
        print('write this part lol')
    elif headingAngleMode:
        # Organise the observed data and save into states array.
        positions = vehicle.state['pos']
        posx = positions[0] 
        posy = positions[1]
        velocity = sensors['electrics']['wheelspeed']
        bearing = calc_HeadingAngle(vehicle.state['dir'])
        
        states = np.array([posx, posy, velocity, bearing])
        
    # Add data into arrays to be plotted.
    if plotData:
        graph_positions.append(vehicle.state['pos'])
        graph_positionColour.append(Kgain)        
        graph_directions.append(vehicle.state['dir']) 
        graph_wheelSpeed.append(sensors['electrics']['wheelspeed']) 
        graph_brakes.append(sensors['electrics']['brake']) 
        graph_steering.append(sensors['electrics']['steering']) 
        graph_throttle.append(sensors['electrics']['throttle']) 
    
        
        #%% Quick Save
columnNames = ['positions', 'gain colours', 'directions', 'wheelSpeed', 'brakes', 'steering', 'throttle']
data = np.array(np.array(graph_positions), np.array(graph_positionColour), np.array(graph_directions), np.array(graph_wheelSpeed), np.array(graph_brakes), np.array(graph_steering), np.array(graph_throttle))
expoort = pd.DataFrame(np.transpose(data), columns = columnNames)

# outputFile = 'Saves/PC-Final-Run_17.csv'
expoort.to_csv(outputFile, index=False, lineterminator='\r\n')
        
#%% Plotting of Playback    
if False:
    # import saved results
    df = pd.read_csv('Saves/PC-Final-Run_17.csv')
        #
    graph_positions      = np.array(eval(df['positions'].iloc[0]), dtype=np.float64) #imported_data['positions'].to_numpy()
    graph_positionColour = np.array(eval(df['gain colours'].iloc[0]), dtype=np.float64) #imported_data['gain colours'].to_numpy()
    graph_directions     = np.array(eval(df['directions'].iloc[0]), dtype=np.float64) #imported_data[ 'directions'].to_numpy()
    graph_wheelSpeed     = np.array(df['wheelSpeed'], dtype=np.float64) #imported_data['wheelSpeed'].to_numpy() 
    graph_brakes         = np.array(df['brakes'], dtype=np.float64) #imported_data['brakes'].to_numpy()
    graph_steering       = np.array(df['steering'], dtype=np.float64) #imported_data['steering'].to_numpy()
    graph_throttle       = np.array(df['throttle'], dtype=np.float64) #imported_data['throttle'].to_numpy() 
        #
    test_sec = len(graph_wheelSpeed)
    t = np.linspace(0,test_sec*pollFrequency_sec,int(test_sec))
    
    
# Plot data    
if plotData:
    fig3 = plt.figure(figsize= (14,12), layout="constrained")
    fig3.suptitle(f'Simulation results using learned controller', fontsize='xx-large', fontweight='extra bold')  
    
    gs = GridSpec(3, 2, figure = fig3)
    axP  = fig3.add_subplot(gs[0:2, 0])
    axS  = fig3.add_subplot(gs[2, 0])
    axT  = fig3.add_subplot(gs[0,1])
    axSt = fig3.add_subplot(gs[1,1])
    axH  = fig3.add_subplot(gs[2,1])
    
    # Plot Road and Position
        #Road
    roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']
    [road_leftEdges, road_rightEdges, roadGeometries] = plot_road(axP, roadlist, f'Vehicle Position and Road Edges', 'k--', 'k-.', roadGeometries, True)
        # Position
    x = [p[0] for p in graph_positions]
    y = [p[1] for p in graph_positions]
        # plot position based on colour (gain)
    i0 = 0; current_colour = 'b-';
    for i in range(len(x)):
        if graph_positionColour[i] == "K1":
            colour = 'c'
        elif graph_positionColour[i] == "K2":
            colour = 'm'
        elif graph_positionColour[i] == "K3":
            colour = 'g'
        elif graph_positionColour[i] == "K4":
            colour = 'darkorange'
            
        if current_colour != colour:
            i0 = i
            current_colour = colour
        if i0 == i:
            axP.plot(x[i0-2:i], y[i0-2:i], color = colour, label="Position of Vehicle")
        else:
            axP.plot(x[i0:i], y[i0:i], colour, label="Position of Vehicle")
    if True: # Settings
        [k1, k2, k3, k4, road] = segment_LegendHandler()
        axP.legend(handles=[k1, k2, k3, k4, road], loc='lower left', bbox_to_anchor=(0.0125, 0.0125))
        axP.set_xlabel('x coordinates', fontsize='x-large', fontweight='semibold')
        axP.set_ylabel('y coordinates', fontsize='x-large', fontweight='semibold')
        [x_min, x_max, _, _] = road_maxmin(roadlist, roadGeometries)
        axP.set_xlim([x_min-20, x_max-20])
        axP.grid()
    # Plot Rest of Data
    t = np.linspace(0,test_sec,int(test_sec/pollFrequency_sec))
        # Speed
    plot_speed(axS, graph_wheelSpeed, 'k-')
    axS.set_ylim([0,40])
    axS.set_xlim([0,t[-1]])
        # Throttle
    plot_throttle(axT, graph_throttle, 'g-')
        # Steering
    plot_steering(axSt, graph_steering, 'b-')
    axSt.set_xlim([0,t[-1]])
        # Brake
    plot_brake(axT, graph_brakes, 'r-')
    axT.set_title(' Throttle and Brake input ', fontsize = 'x-large')
    axT.set_xlim([0,t[-1]])
    axT.legend()
        # Heading
    x_head = [d[0] for d in graph_directions]
    y_head = [d[1] for d in graph_directions]
    plot_headingVectors(axH, x_head, y_head)
    axH.set_xlim([0,t[-1]])
    axH.set_ylim([-1.05,1.05])
        # Segment Colouring
    if True:    
        plot_segments(a, b, c, axP)
        alphaNumber = 0.15
        plot_segmentsTimeSeries(t, graph_positionColour, 0, 40, axS, alphaNumber)
        plot_segmentsTimeSeries(t, graph_positionColour, 0, 2, axT, alphaNumber)
        plot_segmentsTimeSeries(t, graph_positionColour, -200, 200, axSt, alphaNumber)
        plot_segmentsTimeSeries(t, graph_positionColour, -1.2, 1.2, axH, alphaNumber)
    
#%% Export Data
columnNames = ['time', 'throttle', 'steering', 'brakes', 'x_pos', 'y_pos', 'velocity', 'directions']

x_pos = [p[0] for p in positions]
y_pos = [p[1] for p in positions]

speed = list()
for v in velocity:
    speed.append(math.sqrt(v[0]**2+v[1]**2))

t = np.linspace(0,test_sec,int(test_sec/pollFrequency_sec))
export_data = np.array([t, throttle, steering, brakes, x_pos, y_pos, speed, directions])
states = pd.DataFrame(np.transpose(export_data), columns = columnNames) 

outputFile = f'Saves/{scenario.name}_{ID}.csv'

states.to_csv(outputFile, index=False, lineterminator='\r\n')
print(f'saved run {ID}')
#%% Import Data and Plot
shouldWeSave = False

importFile = True
if importFile:
    inputFile = f'Saves/straightsAndcircles_{ID}.csv'
    [mu, eta, t, throttle, steering, brakes, x_pos, y_pos, velocity, headingAngle_degrees, positions, x_head, y_head] = import_data(inputFile, False)
else:
    x_head = [d[0] for d in directions]
    y_head = [d[1] for d in directions]
    
    speed = list()
    for v in velocity:
        speed.append(math.sqrt(v[0]**2+v[1]**2))
    velocity = speed
    
#% Import Road Dictionary
global roadGeometries
filePath = 'straightsAndcircles_roadGeometries.pkl'

with open (filePath, 'rb') as fp:
    roadGeometries = pickle.load(fp)


#% Combined Plots (6 Plots)
# t = np.linspace(0,t_sec,int(t_sec/pollFrequency_sec))

# Init Graphs
fig = plt.figure(figsize= (14,12), layout="constrained")
title = f'Data Collection using ID {ID}'#f'{scenario.name} {ID}'
fig.suptitle(title, fontsize='xx-large', fontweight='extra bold')  
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
plot_speed(axS, velocity, 'k-')
plot_throttle(axT, throttle, 'g-')
plot_steering(axSt,steering, 'b-')
plot_brake(axT, brakes, 'r-')
axT.set_title(' Throttle and Brake input ', fontsize = 'x-large')
plot_headingVectors(axH, x_head, y_head)
[ x_min, x_max, y_min, y_max] = road_maxmin(roadlist, roadGeometries)
[road_leftEdges, road_rightEdges, roadGeometries] = plot_road(axP, roadlist, 'Vehicle Position and Road Edges', lineFormat_leftEdge, lineFormat_rightEdge, roadGeometries, imported)
axP.set_xlim([x_min-20, x_max-20])
plot_position(axP, lineFormat_position, False)
[left, right, pos] = my_legendHandler()
axP.grid()
axS.set_xlim([0,t[-1]])
axSt.set_xlim([0,t[-1]])
axT.set_xlim([0,t[-1]])
axH.set_xlim([0,t[-1]])
axH.set_ylim([-1.05,1.05])
# plot_segments(a, b, c, axP)

axP.legend(handles=[left, right, pos], loc='center', bbox_to_anchor=(0.13, 0.08))
axP.set_xlabel('x coordinates', fontsize='x-large', fontweight='semibold')
axP.set_ylabel('y coordinates', fontsize='x-large', fontweight='semibold')

if shouldWeSave:
    outputIMG = f'Saves/{scenario.name}_{ID}.png'
    fig.savefig(outputIMG)

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
plot_speed(axS, wheelSpeed, 'k-')
plot_headingVectors(axH, x_head, y_head)
plot_throttle(axT, throttle, 'g-')
plot_steering(axSt, steering, 'b-')
plot_brake(axB, brakes, 'r-')

#%% Report Image: Plotting road edges, comparing as exported vs the truncated version. 
    # Initialise
roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']
left_colourAndtype = 'b-'
right_colourAndtype = 'm-'
fig, (ax, ax2) = plt.subplots(1,2, sharex='all', sharey='all', figsize=(10,8))
    # Plot using functions
[ x_min, x_max, y_min, y_max] = road_maxmin(roadlist, roadGeometries)
[road_leftEdges, road_rightEdges, roadGeometries] = plot_road(ax2, roadlist, 'Truncated', left_colourAndtype, right_colourAndtype, roadGeometries, True)
plot_decalRoad(ax, roadlist, roadGeometries, 'As exported')
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
fig2, ax3 = plt.subplots(figsize=(10, 12))
[road_leftEdges, road_rightEdges, roadGeometries] = plot_road(ax3, roadlist, 'Segments', 'b--', 'm--', roadGeometries, True)
plot_position(ax3, 'k--', False)

[left, right, pos] = my_legendHandler()
fig2.legend(handles=[left, right, pos], loc='center', bbox_to_anchor=(.78, 0.048))
fig2.supxlabel('x coordinates', fontsize='x-large', fontweight='semibold')
fig2.supylabel('y coordinates', fontsize='x-large', fontweight='semibold')

#%% Report Image: Segmenting
fig2, ax3 = plt.subplots(figsize=(10, 12))
axP = ax3
# Plot Road and Position
    #Road
roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']
plotTitle = 'Track Segments' #f'Vehicle Position and Road Edges'

[road_leftEdges, road_rightEdges, roadGeometries] = plot_road(axP, roadlist, plotTitle, 'k--', 'k-.', roadGeometries, True)
plotPositions = False
if plotPositions:
       # Position
    x = [p[0] for p in graph_positions]
    y = [p[1] for p in graph_positions]
        # plot position based on colour (gain)
    i0 = 0; current_colour = 'b-';
    for i in range(len(x)):
        if graph_positionColour[i] == "K1":
            colour = 'c'
        elif graph_positionColour[i] == "K2":
            colour = 'm'
        elif graph_positionColour[i] == "K3":
            colour = 'g'
        elif graph_positionColour[i] == "K4":
            colour = 'darkorange'
            
        if current_colour != colour:
            i0 = i
            current_colour = colour
        if i0 == i:
            axP.plot(x[i0-2:i], y[i0-2:i], color = colour, label="Position of Vehicle")
        else:
            axP.plot(x[i0:i], y[i0:i], colour, label="Position of Vehicle")
if True: # Settings
    [k1, k2, k3, k4, road] = segment_LegendHandler()
    axP.legend(handles=[k1, k2, k3, k4, road], loc='lower left', bbox_to_anchor=(0.0125, 0.0125))
    axP.set_xlabel('x coordinates', fontsize='x-large', fontweight='semibold')
    axP.set_ylabel('y coordinates', fontsize='x-large', fontweight='semibold')
    [x_min, x_max, _, _] = road_maxmin(roadlist, roadGeometries)
    axP.set_xlim([x_min-20, x_max-20])
    axP.grid()
    plot_segments(a, b, c, axP)

if False: # Video Only
    axP.set_xlim([x_min, x_max])
    
    # rewrite of plot_segments
    x = np.array([-100, 100, 255])
    # Define Regions 
    y1 = np.array([a*x[0]-b, c])
    y12 = np.array([300, 300])
    y2 = np.array([500, 500, 500])
    y22 = np.array([300, 300, x[2]*a+(300-b)])
    y3 = np.array([300, a*x[2]+(300-b)])
    y32 = np.array([c, c])
    y4 = np.array([a*x[0]-b, c, c])
    y42 = np.array([-200, -200, -200])
    # Fill regions 
    ax3.fill_between((x[0],x[1]), y1, y12, where=(y12 >= y1), color = 'deepskyblue', alpha = 0.15, interpolate = False)
    # ax3.fill_between(x, y2, y22, where=(y2 >= y22), color = 'fuchsia', alpha = 0.15*.7, interpolate = False)
    # ax3.fill_between((x[1],x[2]), y3, y32, where=(y3 >= y32), color = 'lime', alpha = 0.15, interpolate = False)
    # ax3.fill_between(x, y4, y42, where=(y4 >= y42), color = 'darkorange', alpha = 0.15, interpolate = False)
    # ax3.grid(which='both',  linestyle='--') 
    plt.savefig('K1.png', transparent=True)