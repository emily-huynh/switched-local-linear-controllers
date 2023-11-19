import colorsys
import time

import matplotlib.pyplot as plt
import matplotlib.axes as pltaxs
from matplotlib.gridspec import GridSpec
import matplotlib.lines as mlines

import numpy as np
import pandas as pd
import pickle


#% Import Road Dictionary
global roadGeometries
filePath = 'straightsAndcircles_roadGeometries.pkl'

with open (filePath, 'rb') as fp:
    roadGeometries = pickle.load(fp)

roadlist = ['straightFront', 'loopOne', 'straightBack', 'loopTwo']
#%


from beamngpy import BeamNGpy, MeshRoad, Road, Scenario, Vehicle, angle_to_quat 
from beamngpy.sensors import ( 
     IMU, Camera, Damage, Electrics, State, Timer) 
# from shapely.geometry import Polygon, Point

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
beamng = BeamNGpy('localhost', 64256, home='C:/Users/Em/Documents/0. Honours/BeamNG.tech.v0.27.2.0')
# beamng.open()

  
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

roadlist = ['straightFront', 'straightBack', 'loopOne', 'loopTwo']

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
#scenario.make(beamng)

    
#%% Export Road Dictionary
# with open(f'{scenario.name}_roadGeometries.pkl', 'wb') as fp:
#     pickle.dump(roadGeometries, fp)
#     print(f'{scenario.name} road dictionary saved to file')

