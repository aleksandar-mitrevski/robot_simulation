from trbm_base import TRBMBase
import numpy
import math

class SensorTRBMInferenceEngine(TRBMBase):
    def __init__(self, number_visible_units, number_hidden_units, weights, fault_threshold, order=1):
        self.number_visible_units = number_visible_units
        self.number_hidden_units = number_hidden_units
        self.fault_threshold = fault_threshold
        self.order = order

        self.visible_values = list()
        self.hidden_values = list()
        for _ in xrange(order+1):
            self.visible_values.append(numpy.zeros((number_visible_units,1)))
            self.hidden_values.append(numpy.zeros((number_hidden_units,1)))

        #visible to hidden connections at time t
        self.connection_weights = weights['connection_weights']

        #bias at time t
        self.visible_bias = weights['visible_bias']
        self.hidden_bias = weights['hidden_bias']

        #bias propagated from previous time steps
        self.visible_to_visible_bias = list()
        self.visible_to_hidden_bias = list()
        self.hidden_to_hidden_bias = list()
        for _ in xrange(order):
            visible_to_visible = numpy.random.rand(number_visible_units, number_visible_units) * 0.05
            visible_to_hidden = numpy.random.rand(number_visible_units, number_hidden_units) * 0.05
            hidden_to_hidden = numpy.random.rand(number_hidden_units, number_hidden_units) * 0.05

            self.visible_to_visible_bias.append(weights['visible_to_visible_bias'][order])
            self.visible_to_hidden_bias.append(weights['visible_to_hidden_bias'][order])
            self.hidden_to_hidden_bias.append(weights['hidden_to_hidden_bias'][order])

    def detect_fault(self, current_vector, initial_data=None):
        sample = self.sample_network(current_vector, initial_data)
        sample_distance = self._distance(current_vector, sample)
        return sample_distance > self.fault_threshold

    def sample_network(self, current_vector, initial_data=None):
        """Samples a visible vector from the network.

        Keyword arguments:
        current_vector -- Data vector at time t given as a 'numpy.array' of dimension (number_visible_units).
        initial_data -- Data used for initialising the network, given as a 'numpy.array' of dimension (number_visible_units,order) (default None, meaning that the network has already been initialised).

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        if initial_data != None:
            for t in xrange(self.order):
                self.visible_values[t] = self._copy_array(initial_data[t,:], self.visible_values[t].shape)
                self.hidden_values[t] = self._sample_initial(t,1)

        self.visible_values[self.order] = self._copy_array(current_vector, self.visible_values[self.order].shape)
        visible_units = numpy.array(self.visible_values[self.order])
        hidden_units = numpy.array(self.hidden_values[self.order])

        current_time_visible_bias_values = self.connection_weights.dot(self.hidden_values[self.order])
        visible_bias = self._bias_function_visible()

        current_time_hidden_bias_values = self.connection_weights.T.dot(self.visible_values[self.order])
        hidden_bias = self._bias_function_hidden()

        for neuron in xrange(self.number_hidden_units):
            prob = self._sigmoid(current_time_hidden_bias_values[neuron] + hidden_bias[neuron])
            hidden_units[neuron] = prob

        for neuron in xrange(self.number_visible_units):
            prob = self._sigmoid(current_time_visible_bias_values[neuron] + visible_bias[neuron])
            threshold = numpy.random.rand()

            if prob > threshold:
                visible_units[neuron] = 1.
            else:
                visible_units[neuron] = 0.

        self._shift_visible_vectors_back()
        return visible_units

    def _sample_initial(self, t, k):
        """Samples a hidden layer given only on the visible vector at the current time step.
        Uses Contrastive Divergence for sampling the values.

        Keyword arguments:
        t -- Current time step.
        k -- The number of samples created by Contrastive Divergence before a sample is accepted.

        Returns:
        hidden_units -- A 'numpy.array' containing the sampled hidden values.

        """
        visible_units = numpy.array(self.visible_values[t])
        hidden_units = numpy.array(self.hidden_values[t])

        for sample in xrange(k):
            for neuron in xrange(self.number_hidden_units):
                prob = self._sigmoid(numpy.sum(self.connection_weights[:,neuron] * visible_units) + self.hidden_bias[neuron])
                hidden_units[neuron] = prob

            for neuron in xrange(self.number_visible_units):
                prob = self._sigmoid(numpy.sum(self.connection_weights[neuron,:] * hidden_units) + self.visible_bias[neuron])
                threshold = numpy.random.rand()

                if prob > threshold:
                    visible_units[neuron] = 1.
                else:
                    visible_units[neuron] = 0.

        return hidden_units

    def _distance(self, vec1, vec2):
        dimension = len(vec1)
        dimension_distance_sum = 0.
        for i in xrange(dimension):
            dimension_distance_sum = dimension_distance_sum + (vec1[i] - vec2[i]) * (vec1[i] - vec2[i])
        return math.sqrt(dimension_distance_sum)


class SensorTRBMContinuousInferenceEngine(TRBMBase):
    def __init__(self, number_visible_units, number_hidden_units, weights, fault_threshold, order=1):
        self.number_visible_units = number_visible_units
        self.number_hidden_units = number_hidden_units
        self.fault_threshold = fault_threshold
        self.order = order

        self.visible_values = list()
        self.hidden_values = list()
        for _ in xrange(order+1):
            self.visible_values.append(numpy.zeros((number_visible_units,1)))
            self.hidden_values.append(numpy.zeros((number_hidden_units,1)))

        #visible to hidden connections at time t
        self.connection_weights = weights['connection_weights']

        #bias at time t
        self.visible_bias = weights['visible_bias']
        self.hidden_bias = weights['hidden_bias']

        #bias propagated from previous time steps
        self.visible_to_visible_bias = list()
        self.visible_to_hidden_bias = list()
        self.hidden_to_hidden_bias = list()
        for _ in xrange(order):
            visible_to_visible = numpy.random.rand(number_visible_units, number_visible_units) * 0.05
            visible_to_hidden = numpy.random.rand(number_visible_units, number_hidden_units) * 0.05
            hidden_to_hidden = numpy.random.rand(number_hidden_units, number_hidden_units) * 0.05

            self.visible_to_visible_bias.append(weights['visible_to_visible_bias'][order])
            self.visible_to_hidden_bias.append(weights['visible_to_hidden_bias'][order])
            self.hidden_to_hidden_bias.append(weights['hidden_to_hidden_bias'][order])

    def detect_fault(self, current_vector, initial_data=None):
        sample = self.sample_network(current_vector, initial_data)
        sample_distance = self._distance(current_vector, sample)
        return sample_distance > self.fault_threshold

    def sample_network(self, current_vector, initial_data=None):
        """Samples a visible vector from the network.

        Keyword arguments:
        current_vector -- Data vector at time t given as a 'numpy.array' of dimension (number_visible_units).
        initial_data -- Data used for initialising the network, given as a 'numpy.array' of dimension (number_visible_units,order) (default None, meaning that the network has already been initialised).

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        if initial_data != None:
            for t in xrange(self.order):
                self.visible_values[t] = self._copy_array(initial_data[t,:], self.visible_values[t].shape)
                self.hidden_values[t] = self._sample_initial(t,1)

        self.visible_values[self.order] = self._copy_array(current_vector, self.visible_values[self.order].shape)
        visible_units = numpy.array(self.visible_values[self.order])
        hidden_units = numpy.array(self.hidden_values[self.order])

        current_time_visible_bias_values = self.connection_weights.dot(self.hidden_values[self.order])
        visible_bias = self._bias_function_visible()

        current_time_hidden_bias_values = self.connection_weights.T.dot(self.visible_values[self.order])
        hidden_bias = self._bias_function_hidden()

        for neuron in xrange(self.number_hidden_units):
            prob = self._sigmoid(current_time_hidden_bias_values[neuron] + hidden_bias[neuron])
            hidden_units[neuron] = prob

        for neuron in xrange(self.number_visible_units):
            prob = self._sigmoid(current_time_visible_bias_values[neuron] + visible_bias[neuron])
            visible_units[neuron] = prob

        self._shift_visible_vectors_back()
        return visible_units

    def _sample_initial(self, t, k):
        """Samples a hidden layer given only on the visible vector at the current time step.
        Uses Contrastive Divergence for sampling the values.

        Keyword arguments:
        t -- Current time step.
        k -- The number of samples created by Contrastive Divergence before a sample is accepted.

        Returns:
        hidden_units -- A 'numpy.array' containing the sampled hidden values.

        """
        visible_units = numpy.array(self.visible_values[t])
        hidden_units = numpy.array(self.hidden_values[t])

        for sample in xrange(k):
            for neuron in xrange(self.number_hidden_units):
                prob = self._sigmoid(numpy.sum(self.connection_weights[:,neuron] * visible_units) + self.hidden_bias[neuron])
                hidden_units[neuron] = prob

            for neuron in xrange(self.number_visible_units):
                prob = self._sigmoid(numpy.sum(self.connection_weights[neuron,:] * hidden_units) + self.visible_bias[neuron])
                visible_units[neuron] = prob

        return hidden_units

    def _distance(self, vec1, vec2):
        dimension = len(vec1)
        dimension_distance_sum = 0.
        for i in xrange(dimension):
            dimension_distance_sum = dimension_distance_sum + (vec1[i] - vec2[i]) * (vec1[i] - vec2[i])
        return math.sqrt(dimension_distance_sum)
