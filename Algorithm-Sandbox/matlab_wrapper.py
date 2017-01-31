#
# matlab_wrapper.py
# Algorithm Sandbox
# 
# Created by Blake Nazario-Casey on 01/02/2017
# Last updated by Blake Nazario-Casey on 01/02/2017
#
# DESCRIPTION:
#	Wrapper for AI-Module python scripts that allows for execution of MATLAB 
#	scripts.
#
# USAGE:
#	To execute a MATLAB script/function, import the functions of this script
#   like so:
#		from matlab_wrapper import execute
#
#	Then, executing the wrapper function will work simply by calling that 
#   function from whatever file you are importing the wrapper to. 
#
#   Wrapper functions will be added as necessary. This wrapper is intended 
#   to be used as an API between the AI-Module scripts and the actual 
#   AI work being done by MATLAB. 


import matlab.engine
import os
import numpy as np

################################################################################### 
### Wrapper + Engine Initialization

# Set working directory to scripts parent directory
os.chdir("/Users/BlakeNaz/tensorflow/rmc_2017_robot/Algorithm-Sandbox")	

# Initialize Engine
eng = matlab.engine.start_matlab()

### End Initializtion
################################################################################### 



################################################################################### 
### MATLAB Script execution

def execute(start, goal, grid):
	print("Executing...")

	'''
	
	From here, call the parent script for autonomy 
	
	'''

	w, h = 12, 24
	# Matrix = [[0 for x in range(w)] for y in range(h)] 
	# print(Matrix)
	grid = matlab.double(grid)

	start = matlab.double(start)
	goal = matlab.double(goal)

	print("Start: " + str(start))
	print("Goal: " + str(goal))

	path = eng.plan_path(start, goal,grid);

	print("Planned path received\n\n");
	print(path);

	

### End MATLAB Script execution
################################################################################### 



################################################################################### 
### Quit MATLAB Engine

def quitEngine():
	# Quit Engine
	eng.quit()

### End Quit Engine
###################################################################################


## Execute code
#execute();
