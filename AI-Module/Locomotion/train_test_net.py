#TRAIN TEST NET

from test_net import create_feature_sets_and_labels
import tensorflow as tf
import pickle
import numpy as np

checkpoint_dir = "/Users/BlakeNaz/tensorflow/rmc_2017_robot/AI-Module/Locomotion/Model/"

train_x,train_y,test_x,test_y = create_feature_sets_and_labels('TEST_DATA/formatted_positive','TEST_DATA/formatted_negative')

n_nodes_hl1 = 10 #15 #1500
n_nodes_hl2 = 10 #15 #1500
n_nodes_hl3 = 10 #15 #1500

n_classes = 2
batch_size = 100 #100
hm_epochs = 10

x = tf.placeholder('float')
y = tf.placeholder('float')

'''
hidden_1_layer = {'f_fum':n_nodes_hl1,
                  'weight':tf.Variable(tf.random_normal([len(train_x[0]), n_nodes_hl1])),
                  'bias':tf.Variable(tf.random_normal([n_nodes_hl1]))}

hidden_2_layer = {'f_fum':n_nodes_hl2,
                  'weight':tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])),
                  'bias':tf.Variable(tf.random_normal([n_nodes_hl2]))}

hidden_3_layer = {'f_fum':n_nodes_hl3,
                  'weight':tf.Variable(tf.random_normal([n_nodes_hl2, n_nodes_hl3])),
                  'bias':tf.Variable(tf.random_normal([n_nodes_hl3]))}

output_layer = {'f_fum':None,
                'weight':tf.Variable(tf.random_normal([n_nodes_hl3, n_classes])),
                'bias':tf.Variable(tf.random_normal([n_classes])),}
'''


hidden_1_layer = {'f_fum':n_nodes_hl1, 
				  'weight':tf.Variable(tf.random_normal([len(train_x[0]), n_nodes_hl1])),
				  'bias':tf.Variable(tf.random_normal([n_nodes_hl1]))}

hidden_2_layer = {'f_fum':n_nodes_hl2,
                  'weight':tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])),
                  'bias':tf.Variable(tf.random_normal([n_nodes_hl2]))}

output_layer = {'f_fum':None,
                'weight':tf.Variable(tf.random_normal([n_nodes_hl2, n_classes])),
                'bias':tf.Variable(tf.random_normal([n_classes])),}


def neural_network_model(data):
	'''
    l1 = tf.add(tf.matmul(data,hidden_1_layer['weight']), hidden_1_layer['bias'])
    l1 = tf.nn.relu(l1)

    l2 = tf.add(tf.matmul(l1,hidden_2_layer['weight']), hidden_2_layer['bias'])
    l2 = tf.nn.relu(l2)

    l3 = tf.add(tf.matmul(l2,hidden_3_layer['weight']), hidden_3_layer['bias'])
    l3 = tf.nn.relu(l3)

    output = tf.matmul(l3,output_layer['weight']) + output_layer['bias']
	'''
	sess = tf.Session()
	sess.run(tf.initialize_all_variables())

	l1 = tf.add(tf.matmul(data,hidden_1_layer['weight']), hidden_1_layer['bias'])
	print("LAYER 1: "  + str(l1))
	print("LAYER 1 WEIGHT: " + str(sess.run(hidden_1_layer['weight'])))

	l1 = tf.nn.relu(l1)

	l2 = tf.add(tf.matmul(l1,hidden_2_layer['weight']), hidden_2_layer['bias'])
	l2 = tf.nn.relu(l2)
	print("LAYER 2: "  + str(l2))

	# 'Output' is the y-value
	output = tf.matmul(l2,output_layer['weight']) + output_layer['bias']

	print("OUTPUT: " + str(output))

	return output

saver = tf.train.Saver()
tf_log = 'tf.log'

# '''
def train_neural_network(x):
	prediction = neural_network_model(x)
	cost = tf.reduce_mean( tf.nn.softmax_cross_entropy_with_logits(prediction,y) )
	optimizer = tf.train.AdamOptimizer(learning_rate=0.001).minimize(cost)

	with tf.Session() as sess:
		sess.run(tf.initialize_all_variables())
	    
		for epoch in range(hm_epochs):
			epoch_loss = 0
			i=0
			while i < len(train_x):
				start = i
				end = i+batch_size
				batch_x = np.array(train_x[start:end])
				batch_y = np.array(train_y[start:end])

				_, c = sess.run([optimizer, cost], feed_dict={x: batch_x,
				                                              y: batch_y})
				epoch_loss += c
				i+=batch_size

			saver.save(sess, checkpoint_dir + 'model.ckpt')

				
			print('Epoch', epoch+1, 'completed out of',hm_epochs,'loss:',epoch_loss)
		correct = tf.equal(tf.argmax(prediction, 1), tf.argmax(y, 1))
		accuracy = tf.reduce_mean(tf.cast(correct, 'float'))

		print('Accuracy:',accuracy.eval({x:test_x, y:test_y}))

train_neural_network(x)
# '''


'''
def use_neural_network(input_data):
	prediction = neural_network_model(x)

	sess = tf.Session()
	sess.run(tf.initialize_all_variables())
	saver.restore(sess, checkpoint_dir + "model.ckpt")

	ckpt = tf.train.get_checkpoint_state(checkpoint_dir)
	if ckpt:
   		print("CKPT obtained.")
    	#saver.restore(sess, ckpt.model_checkpoint_path)
	else:
		raise ValueError('Checkpoint not found in directory: ' + checkpoint_dir)


	print("Data to test against: " + str(input_data))


	#
	#NOTE: This form of use outputs an argmax value relating to the binary classification
	#	  of the provided tensor. This method must be changed when the neural network model is 
	#	  updated to have classifiers for each data point.
	#

	# pos: [1,0] , argmax: 0
	# neg: [0,1] , argmax: 1
	predictions = sess.run(tf.argmax(prediction.eval(session=sess,feed_dict={x:np.array([input_data])}), 1))
	
	print("FEED DICT: " + str({x:np.array(input_data)}))	

	probability = prediction
	print('Classifications: ' + str(probability.eval(feed_dict = {x:np.array([input_data])}, session = sess)))

	#predictions = sess.run(prediction,feed_dict={x:np.array([input_data])})
	print("Predictions: " + str(predictions))
	if predictions[0] == 0: 
		print("Positive " + str(predictions) + "; Data: " + str(input_data))
	

	if predictions[0] == 1:
		print("Negative " + str(predictions) + "; Data: " + str(input_data))
	


	sess.close()

use_neural_network([1.098, 0.045, 1.987, 0.314])
'''
