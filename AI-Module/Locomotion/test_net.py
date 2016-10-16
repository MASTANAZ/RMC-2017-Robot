# Test Network


'''
This is a test approach to the ML algorithm that will exist in the robot.
'''
import tensorflow as tf 
import ast
import numpy as np 
import random
import pickle
import matplotlib.pyplot as plt

hm_lines_controller = 24 # The number of lines for each controller set from the xcode algorithm
hm_lines_file = 24000


'''
Create_Data_Set

Creates tensor-flow-acceptable data sets

@positive The file path to positive-producing value
@negative The file path to negative-producing values
'''

def create_data_set(positive, negative):

	lexicon = []
	totalControllerSet = []

	with open(positive,'r') as f:
        
		contents = f.readlines()
        
		controllerWithPoints = []
		pointsList = []

		idx = 0
		for i in range(0,hm_lines_file):
			pointsList.append(float(contents[i]))
			idx = idx + 1

			if idx == 4:
				totalControllerSet.append(pointsList)
				pointsList = []
				idx = 0
			
	with open(negative,'r') as f:
		contents = f.readlines()
        
		controllerWithPoints = []
		pointsList = []

		idx = 0
		for i in range(0,hm_lines_file):
			pointsList.append(float(contents[i]))
			idx = idx + 1

			if idx == 4:
				totalControllerSet.append(pointsList)
				pointsList = []
				idx = 0

	return totalControllerSet


def sample_handling(sample,lexicon,classification):

	contents = lexicon;

	featureset = []

	for i in range(0,hm_lines_file/2):

		features = contents[i]
		featureset.append([features, classification])

	return featureset


def create_feature_sets_and_labels(pos,neg,test_size = 0.1):
	lexicon = create_data_set(pos,neg)

	'''
	NOTE: For the features, feel free to add more classifiers (such as neutral)

	For every classifier, add another:
		features += sample_handling('file_name',lexicon,[a,b,c])

	The [a,b,c] array will contain a number of elements equal to the number of classifiers.
		e.g. positive and negative makes 2 classifiers, so [a,b]
		     positive, negative, neutral makes 3 classifiers, so [a,b,c]

    NOTE: The features will have to be modified to include a positive-negative classifier
          for EVERY individual data point. The creates a classifier array for each controller
          like so: 

          FeatureSet = [
                       [[a, b, c] , [[1,0], [1,0], [1,0]],
                       ...
                       ]
	'''
	features = []
	features += sample_handling('positive',lexicon,[1,0]) #[1,0] corresponds to positive classification
	features += sample_handling('negative',lexicon,[0,1]) #[0,1] corresponds to negative classification

	random.shuffle(features)
	features = np.array(features)

	testing_size = int(test_size*len(features))

	train_x = list(features[:,0][:-testing_size])
	train_y = list(features[:,1][:-testing_size])
	test_x = list(features[:,0][-testing_size:])
	test_y = list(features[:,1][-testing_size:])


	return train_x,train_y,test_x,test_y


if __name__ == '__main__':
	train_x,train_y,test_x,test_y = create_feature_sets_and_labels('positive','negative')








