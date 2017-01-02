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

################################################################################### 
### Wrapper + Engine Initialization

# Set working directory to scripts parent directory
os.chdir("/Users/BlakeNaz/tensorflow/rmc_2017_robot/Algorithm-Sandbox/MATLAB")	

# Initialize Engine
eng = matlab.engine.start_matlab()

### End Initializtion
################################################################################### 



################################################################################### 
### MATLAB Script execution

def execute():
	print("Executing")

	'''
	
	From here, call the parent script for autonomy by 
	
	'''

### End MATLAB Script execution
################################################################################### 



################################################################################### 
### Quit MATLAB Engine

def quitEngine():
	# Quit Engine
	eng.quit()

### End Quit Engine
###################################################################################

