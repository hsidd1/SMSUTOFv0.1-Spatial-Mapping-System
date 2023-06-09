#
#   Modified by Hamza Siddiqui - 400407170 - siddih38

#   Purpose: This example Python program simulates data received from a
#   sensor by writing the expected format of example data to a file. The
#   visualization will be demonstrated using a Python module called Open3D.
#
#   Special notes:
#       1. Open3D only works with Pythons 3.6-3.9.  It does not work with 3.10
#       2. For this eample you should run it in IDLE.  Anaconda/Conda/Jupyter
#       require different Open3D graphing methods (these methods are poorly documented)
#       3. Under Windows 10 you may need to install the MS Visual C++ Redistributable bundle
#           https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170
#       4. VirtualBox does not support OpenGL. If you're running Windows under Virtualbox or
#       your system doesn't support OpenGL (very rare), then you can install an OpenGL emulator dll
#           https://fdossena.com/?p=mesa/index.frag (unzip and copy opengl32.dll into Python dir)
#
#   T. Doyle
#   March 18, 2022 (Updated 2020 example)
#

'''
Run this program after acquiring measurement data into file from measurement_data.py. This program reads data from the file
and  outputs a 3D spatial mapping from the measurements obtained

'''
# Importing necessary libraries
import numpy as np
import open3d as o3d

if __name__ == "__main__":
    num_scans = int(input("Enter how many scans taken:"))
    # Set number of rotations
    ROTATIONS = 16

    #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("pointdata.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))
    #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])
    
    # append each vertex from data
    yz_slice_vertex = [i for i in range(0,num_scans*ROTATIONS)]
  

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,num_scans*ROTATIONS,ROTATIONS):
        for i in range(ROTATIONS):
            if i==ROTATIONS-1:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x]])
            else:
                lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+1]])
    
    #Define coordinates to connect lines between current and next yz slice        
    for x in range(0,num_scans*ROTATIONS-ROTATIONS-1,ROTATIONS):
        for i in range(ROTATIONS):
            lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+i+ROTATIONS]])

    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set]) # Opens GUI of visualization
                                    
    
 
