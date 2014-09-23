from trbm_base import TRBMBase
import numpy
import math

class SensorTRBMInferenceEngine(TRBMBase):
    def __init__(self, number_visible_units, number_hidden_units, fault_threshold, order=1):
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
        self.connection_weights = numpy.random.rand(number_visible_units, number_hidden_units) * 0.05

        #bias at time t
        self.visible_bias = numpy.random.rand(number_visible_units,1)
        self.hidden_bias = numpy.random.rand(number_hidden_units,1)

        #bias propagated from previous time steps
        self.visible_to_visible_bias = list()
        self.visible_to_hidden_bias = list()
        self.hidden_to_hidden_bias = list()
        for _ in xrange(order):
            visible_to_visible = numpy.random.rand(number_visible_units, number_visible_units) * 0.05
            visible_to_hidden = numpy.random.rand(number_visible_units, number_hidden_units) * 0.05
            hidden_to_hidden = numpy.random.rand(number_hidden_units, number_hidden_units) * 0.05

            self.visible_to_visible_bias.append(visible_to_visible)
            self.visible_to_hidden_bias.append(visible_to_hidden)
            self.hidden_to_hidden_bias.append(hidden_to_hidden)

    def train(self, data, epochs=100, learning_rate=0.1):
        """Trains the Boltzmann machine with a given set of training vectors.

        Keyword arguments:
        data -- A 'numpy.array' containing data for training the RBM. Each row of the array should be a training vector of dimension 'number_visible_units'.
        epochs -- The number of iterations of the learning algorithm (default 100).
        learning_rate -- The algorithm's learning rate (default).

        """
        for t in xrange(self.order):
            self.visible_values[t] = self._copy_array(data[t,:], self.visible_values[t].shape)
            self.hidden_values[t] = self._sample_initial(t,1)

        number_training_vectors = data.shape[0]
        for t in xrange(self.order,number_training_vectors):
            self.visible_values[self.order] = self._copy_array(data[t,:], self.visible_values[self.order].shape)

            #we sample the current hidden layer given the visible layer up to time t and the hidden layers up to time t-1
            current_time_hidden_bias_values = self.connection_weights.T.dot(self.visible_values[self.order])
            hidden_bias = self._bias_function_hidden()
            for neuron in xrange(self.number_hidden_units):
                prob = self._sigmoid(current_time_hidden_bias_values[neuron] + hidden_bias[neuron])
                self.hidden_values[self.order][neuron] = prob

            #we sample from the network
            sample,_ = self._sample(1)

            #we update the connection weights between the visible and hidden units of the current time step
            for i in xrange(self.number_visible_units):
                #we update the bias values of the visible units
                visible_bias_delta = learning_rate * (self.visible_values[self.order][i] - sample[i])
                self.visible_bias[i] = self.visible_bias[i] + visible_bias_delta

                for j in xrange(self.number_hidden_units):
                    data_expectation = self._sigmoid(numpy.sum(self.connection_weights[:,j] * self.visible_values[self.order]) + self.hidden_bias[j])
                    sample_expectation = self._sigmoid(numpy.sum(self.connection_weights[:,j] * sample) + self.hidden_bias[j])

                    #we update the connection weight between the i-th visible unit and the j-th hidden unit
                    weight_change_delta = learning_rate * (data_expectation * self.visible_values[self.order][i] - sample_expectation * sample[i])
                    self.connection_weights[i,j] = self.connection_weights[i,j] + weight_change_delta

                    #we update the bias values of the hidden units
                    hidden_bias_delta = learning_rate * (data_expectation - sample_expectation)
                    self.hidden_bias[j] = self.hidden_bias[j] + hidden_bias_delta

            #we update the visible to hidden connection weights between the current time step and the previous time steps
            for n in xrange(self.order):
                value_index = self.order - n - 1
                sample,_ = self._sample(1,self.visible_values[value_index])
                for i in xrange(self.number_visible_units):
                    for j in xrange(self.number_hidden_units):
                        data_expectation = self._sigmoid(numpy.sum(self.visible_to_hidden_bias[n][:,j] * self.visible_values[value_index]) + self.hidden_bias[j])
                        sample_expectation = self._sigmoid(numpy.sum(self.visible_to_hidden_bias[n][:,j] * sample) + self.hidden_bias[j])

                        #we update the connection weight between the i-th visible unit and the j-th hidden unit
                        weight_change_delta = learning_rate * (data_expectation * self.visible_values[value_index][i] - sample_expectation * sample[i])
                        self.visible_to_hidden_bias[n][i,j] = self.visible_to_hidden_bias[n][i,j] + weight_change_delta

            #we update the visible to visible connection weights between the current time step and the previous time steps
            for n in xrange(self.order):
                value_index = self.order - n - 1
                sample,_ = self._sample(1,self.visible_values[value_index])
                for i in xrange(self.number_visible_units):
                    for j in xrange(self.number_visible_units):
                        data_expectation = self._sigmoid(numpy.sum(self.visible_to_visible_bias[n][:,j] * self.visible_values[value_index]) + self.visible_bias[j])
                        sample_expectation = self._sigmoid(numpy.sum(self.visible_to_visible_bias[n][:,j] * sample) + self.visible_bias[j])

                        #we update the connection weight between the i-th and the j-th visible unit
                        weight_change_delta = learning_rate * (data_expectation * self.visible_values[value_index][i] - sample_expectation * sample[i])
                        self.visible_to_visible_bias[n][i,j] = self.visible_to_visible_bias[n][i,j] + weight_change_delta

            #we update the hidden to hidden connection weights between the current time step and the previous time steps
            for n in xrange(self.order):
                value_index = self.order - n - 1
                _,sample = self._sample(1,self.visible_values[value_index])
                for i in xrange(self.number_hidden_units):
                    for j in xrange(self.number_hidden_units):
                        data_expectation = self._sigmoid(numpy.sum(self.hidden_to_hidden_bias[n][:,j] * self.hidden_values[value_index]) + self.hidden_bias[j])
                        sample_expectation = self._sigmoid(numpy.sum(self.hidden_to_hidden_bias[n][:,j] * sample) + self.hidden_bias[j])

                        #we update the connection weight between the i-th and the j-th hidden unit
                        weight_change_delta = learning_rate * (data_expectation * self.hidden_values[value_index][i] - sample_expectation * sample[i])
                        self.hidden_to_hidden_bias[n][i,j] = self.hidden_to_hidden_bias[n][i,j] + weight_change_delta

            #we move the visible vectors one time step back
            self._shift_visible_vectors_back()

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

    def _sample(self, k, training_vector=None):
        """Samples a visible vector and a hidden vector given a training vector.
        Uses Contrastive Divergence for sampling the values.

        Keyword arguments:
        k -- The number of samples created by Contrastive Divergence before a sample is accepted.
        training_vector -- A vector that should be used at the t-th time step. (default None, resulting in a vector already stored in self.visible_values[self.order]).

        Returns:
        visible_units -- A 'numpy.array' containing the sampled visible values.
        hidden_units -- A 'numpy.array' containing the sampled hidden values.

        """
        visible_units = None
        if training_vector == None:
            visible_units = numpy.array(self.visible_values[self.order])
        else:
            visible_units = numpy.array(training_vector)
        hidden_units = numpy.array(self.hidden_values[self.order])

        current_time_visible_bias_values = self.connection_weights.dot(self.hidden_values[self.order])
        visible_bias = self._bias_function_visible()

        current_time_hidden_bias_values = self.connection_weights.T.dot(self.visible_values[self.order])
        hidden_bias = self._bias_function_hidden()

        for sample in xrange(k):
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

        return visible_units, hidden_units

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
