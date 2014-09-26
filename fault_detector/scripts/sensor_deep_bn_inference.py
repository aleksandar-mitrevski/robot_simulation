from deep_bn_base import DeepBNBase
import numpy
import math

class SensorDeepBNInferenceEngine(DeepBNBase):
    def __init__(self, number_visible_units, number_hidden_units, weights, fault_threshold, layers=1):
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
        self.biases.append(weights['biases'][0])
        self.biases.append(weights['biases'][1])

        #we initialise the connection weights randomly and scale them to the range (0,0.05)
        self.connection_weights.append(weights['connection_weights'][0])

        for i in xrange(1,layers):
            self.neuron_values.append(numpy.zeros(number_hidden_units[i]))
            self.biases.append(weights['biases'][i+1])
            self.connection_weights.append(weights['connection_weights'][i])

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

    def _distance(self, vec1, vec2):
        dimension = len(vec1)
        dimension_distance_sum = 0.
        for i in xrange(dimension):
            dimension_distance_sum = dimension_distance_sum + (vec1[i] - vec2[i]) * (vec1[i] - vec2[i])
        return math.sqrt(dimension_distance_sum)
