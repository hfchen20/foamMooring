#! /usr/bin/env python
# -*- coding: utf-8 -*-


#   Copyright (C) 2014 mdm                                     
#   map[dot]plus[dot]plus[dot]help[at]gmail                     
#                                                              
# Licensed to the Apache Software Foundation (ASF) under one   
# or more contributor license agreements.  See the NOTICE file 
# distributed with this work for additional information        
# regarding copyright ownership.  The ASF licenses this file   
# to you under the Apache License, Version 2.0 (the            
# "License"); you may not use this file except in compliance   
# with the License.  You may obtain a copy of the License at   
#                                                              
#   http://www.apache.org/licenses/LICENSE-2.0                 
#                                                              
# Unless required by applicable law or agreed to in writing,   
# software distributed under the License is distributed on an  
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY       
# KIND, either express or implied.  See the License for the    
# specific language governing permissions and limitations            
# under the License.                                             


#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Copyright (C) 2015 
map[dot]plus[dot]plus[dot]help[at]gmail                     
License: http://www.apache.org/licenses/LICENSE-2.0                 
'''  

from mapsys import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
np.set_printoptions(precision=2)
np.set_printoptions(suppress=True)


if __name__ == '__main__':      
    mooring_1 = Map()
    
    mooring_1.map_set_sea_depth(350)      # m
    mooring_1.map_set_gravity(9.81)       # m/s^2
    mooring_1.map_set_sea_density(1025.0) # kg/m^3

    # mooring_1.read_file("../test/baseline_1.map")                                       # 120 m depth    
    mooring_1.read_file("../test/baseline_2.map")                                         # 350 m depth    
    # mooring_1.read_file("../test/baseline_5.map")                                       # 80 m depth
    # mooring_1.read_file("../test/NRELOffshrBsline5MW_Platform_OC3Hywind.map")           # 320 m depth
    # mooring_1.read_file("../test/NRELOffshrBsline5MW_Platform_OC3Hywind_segmented.map") # 320 m depth
    # mooring_1.read_file("../test/NRELOffshrBsLine5MW_OC4.map")                          # 200 m depth
    # mooring_1.read_file("../test/NRELOffshrBsLine5MW_TLP.map")                          # 200 m depth
    
    mooring_1.summary_file('summary_file.txt')

    mooring_1.init( )
    
    epsilon = 1e-3 # finite difference epsilon
    K = mooring_1.linear(epsilon)    
    print("\nLinearized stiffness matrix with 0.0 vessel displacement:\n")
    print(np.array(K))
     
    surge = 5.0 # 5 meter surge displacements
    mooring_1.displace_vessel(surge,0,0,0,0,0)
    mooring_1.update_states(0.0,0)
     
    K = mooring_1.linear(epsilon)    
    print("\nLinearized stiffness matrix with %2.2f surge vessel displacement:\n"%(surge))
    print(np.array(K))
    
    # We need to call update states after linearization to find the equilibrium
    mooring_1.update_states(0.0,0)

    line_number = 0
    H,V = mooring_1.get_fairlead_force_2d(line_number)    
    print("Line %d: H = %2.2f [N]  V = %2.2f [N]"%(line_number, H, V))
      
    fx,fy,fz = mooring_1.get_fairlead_force_3d(line_number)    
    print("Line %d: Fx = %2.2f [N]  Fy = %2.2f [N]  Fz = %2.2f [N]\n"%(line_number, fx, fy, fz))

    print("These values come from the output buffer as defined in the 'LINE PROPERTIES' portion of the input file")
    print("Labels : ", mooring_1.get_output_labels()[0:6])
    print("Units  : ", mooring_1.get_output_units()[0:6])
    v = mooring_1.get_output_buffer()[0:6]
    print("Values : ", ["{0:0.2f}".format(i) for i in v])
    
    fig = plt.figure()
    ax = Axes3D(fig)
    num_points = 20
    for i in range(0,mooring_1.size_lines()):
        x = mooring_1.plot_x(i, num_points)
        y = mooring_1.plot_y(i, num_points)
        z = mooring_1.plot_z(i, num_points)        
        ax.plot(x,y,z,'b-')
     
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')        
     
    plt.show()
    
    mooring_1.end( )
