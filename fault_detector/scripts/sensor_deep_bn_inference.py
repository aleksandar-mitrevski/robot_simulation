from deep_bn_base import DeepBNBase
import numpy
import math

class SensorDeepBNInferenceEngine(DeepBNBase):
    def __init__(self, number_visible_units, number_hidden_units, fault_threshold, layers=1):
        """Creates an architecture for a deep belief network.

        Keyword arguments:
        number_visible_units -- An integer denoting the number of visible units.
        number_hidden_units -- A list containing the number of hidden units at each level of the network.
        layers -- The number of layers in the network (default 1, resulting in a single restricted Boltzmann machine).

        """
        self.number_visible_units = number_visible_units
        self.number_hidden_units = list(number_hidden_units)
        self.fault_threshold = fault_threshold
        self.layers = layers

        self.neuron_values = list()
        self.biases = list()
        self.connection_weights = list()

        self.neuron_values.append(numpy.zeros(number_visible_units))
        self.neuron_values.append(numpy.zeros(number_hidden_units[0]))
        self.biases.append(numpy.random.rand(number_visible_units))
        self.biases.append(numpy.random.rand(number_hidden_units[0]))

        #we initialise the connection weights randomly and scale them to the range (0,0.05)
        self.connection_weights.append(numpy.random.rand(number_visible_units,number_hidden_units[0]) * 0.05)

        for i in xrange(1,layers):
            self.neuron_values.append(numpy.zeros(number_hidden_units[i]))
            self.biases.append(numpy.random.rand(number_hidden_units[i]))

            weights = numpy.random.rand(number_hidden_units[i-1],number_hidden_units[i]) * 0.05
            self.connection_weights.append(weights)

    def train(self, data, epochs=100, learning_rate=0.1):
        """Trains the belief network with a given set of training vectors.

        Keyword arguments:
        data -- A 'numpy.array' containing data for training the RBM. Each row of the array should be a training vector of dimension 'number_visible_units'.
        epochs -- The number of iterations of the learning algorithm (default 100).
        learning_rate -- The algorithm's learning rate (default).

        """
        number_training_vectors = data.shape[0]
        for current_layer in xrange(self.layers):
            for _ in xrange(epochs):
                for vector in xrange(number_training_vectors):
                    numpy.copyto(self.neuron_values[0],data[vector,:])

                    #we assign values to all neurons up to the current layer;
                    #if we are training the first layer, no hidden units will be assigned;
                    #if we are training an upper layer, then hidden neurons below it will be assigned values
                    for layer in xrange(current_layer):
                        for neuron in xrange(self.number_hidden_units[layer]):
                            prob = self._logistic(numpy.sum(self.connection_weights[layer][:,neuron] * self.neuron_values[layer]) + self.biases[layer+1][neuron])
                            threshold = numpy.random.rand()
                            if prob > threshold:
                                self.neuron_values[layer+1][neuron] = 1.
                            else:
                                self.neuron_values[layer+1][neuron] = 0.

                    #we sample from the current layer of the network
                    layer_sample = self._sample_layer(current_layer, 1)

                    #we update the connection weights between the visible and hidden units of the current layer
                    for i in xrange(self.connection_weights[current_layer].shape[0]):
                        #we update the bias values of the visible units in the current visible layer
                        visible_bias_delta = learning_rate * (self.neuron_values[current_layer][i] - layer_sample[i])
                        self.biases[current_layer][i] = self.biases[current_layer][i] + visible_bias_delta

                        for j in xrange(self.connection_weights[current_layer].shape[1]):
                            data_expectation = self._logistic(numpy.sum(self.connection_weights[current_layer][:,j] * self.neuron_values[current_layer]) + self.biases[current_layer+1][j])
                            sample_expectation = self._logistic(numpy.sum(self.connection_weights[current_layer][:,j] * layer_sample) + self.biases[current_layer+1][j])

                            #we update the connection weight between the i-th visible unit and the j-th hidden unit
                            weight_change_delta = learning_rate * (data_expectation * self.neuron_values[current_layer][i] - sample_expectation * layer_sample[i])
                            self.connection_weights[current_layer][i,j] = self.connection_weights[current_layer][i,j] + weight_change_delta

                            #we update the bias values of the hidden units in the current visible layer
                            hidden_bias_delta = learning_rate * (data_expectation - sample_expectation)
                            self.biases[current_layer+1][j] = self.biases[current_layer+1][j] + hidden_bias_delta

    def detect_fault(self, vector):
        sample = self.sample_network(vector)
        sample_distance = self._distance(vector, sample)
        return sample_distance > self.fault_threshold

    def sample_network(self, vector=None):
        """Samples a vector from the network.

        Keyword arguments:
        vector -- An input vector for the network's visible layer (default None, in which case a random visible vector is used as input).

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        if vector != None:
            self.neuron_values[0] = numpy.array(vector)
        else:
            for i in xrange(self.number_visible_units):
                self.neuron_values[0][i] = numpy.random.randint(0,2)

        for layer in xrange(self.layers):
            number_hidden_units = len(self.neuron_values[layer+1])
            for neuron in xrange(number_hidden_units):
                prob = self._logistic(numpy.sum(self.connection_weights[layer][:,neuron] * self.neuron_values[layer]) + self.biases[layer+1][neuron])
                threshold = numpy.random.rand()

                if prob > threshold:
                    self.neuron_values[layer+1][neuron] = 1.
                else:
                    self.neuron_values[layer+1][neuron] = 0.

        for layer in xrange(self.layers-1,-1,-1):
            number_visible_units = len(self.neuron_values[layer])
            for neuron in xrange(number_visible_units):
                prob = self._logistic(numpy.sum(self.connection_weights[layer][neuron,:] * self.neuron_values[layer+1]) + self.biases[layer][neuron])
                threshold = numpy.random.rand()

                if prob > threshold:
                    self.neuron_values[layer][neuron] = 1.
                else:
                    self.neuron_values[layer][neuron] = 0.

        return numpy.array(self.neuron_values[0])

    def _sample_layer(self, layer, k):
        """Samples a visible vector at the layer-th layer of the network.
        Uses Contrastive Divergence for sampling the values.

        Keyword arguments:
        layer -- The layer at which we want to sample.
        k -- The number of samples created by Contrastive Divergence before a sample is accepted.

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        visible_units = numpy.array(self.neuron_values[layer])
        hidden_units = numpy.array(self.neuron_values[layer+1])

        visible_biases = numpy.array(self.biases[layer])
        hidden_biases = numpy.array(self.biases[layer+1])

        number_visible_units = len(visible_units)
        number_hidden_units = len(hidden_units)

        for sample in xrange(k):
            for neuron in xrange(number_hidden_units):
                prob = self._logistic(numpy.sum(self.connection_weights[layer][:,neuron] * visible_units) + hidden_biases[neuron])
                threshold = numpy.random.rand()

                if prob > threshold:
                    hidden_units[neuron] = 1.
                else:
                    hidden_units[neuron] = 0.

            for neuron in xrange(number_visible_units):
                prob = self._logistic(numpy.sum(self.connection_weights[layer][neuron,:] * hidden_units) + visible_biases[neuron])
                threshold = numpy.random.rand()

                if prob > threshold:
                    visible_units[neuron] = 1.
                else:
                    visible_units[neuron] = 0.

        return visible_units

    def _distance(self, vec1, vec2):
        dimension = len(vec1)
        dimension_distance_sum = 0.
        for i in xrange(dimension):
            dimension_distance_sum = dimension_distance_sum + (vec1[i] - vec2[i]) * (vec1[i] - vec2[i])
        return math.sqrt(dimension_distance_sum)
