from abc import ABCMeta, abstractmethod
import numpy
import math

class TRBMBase(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def sample_network(self, current_vector, initial_data=None):
        """Samples a visible vector from the network.

        Keyword arguments:
        current_vector -- Data vector at time t given as a 'numpy.array' of dimension (number_visible_units).
        initial_data -- Data used for initialising the network, given as a 'numpy.array' of dimension (number_visible_units,order) (default None, meaning that the network has already been initialised).

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        pass

    @abstractmethod
    def _sample_initial(self, t, k):
        """Samples a hidden layer given only on the visible vector at the current time step.
        Uses Contrastive Divergence for sampling the values.

        Keyword arguments:
        t -- Current time step.
        k -- The number of samples created by Contrastive Divergence before a sample is accepted.

        Returns:
        hidden_units -- A 'numpy.array' containing the sampled hidden values.

        """
        pass

    def _bias_function_visible(self):
        bias = numpy.zeros((self.number_visible_units,1))
        for i in xrange(self.order):
            value_index = self.order - i - 1
            bias = bias + self.visible_to_visible_bias[i].T.dot(self.visible_values[value_index])
        bias = bias + self.visible_bias
        return bias

    def _bias_function_hidden(self):
        bias = numpy.zeros((self.number_hidden_units,1))
        for i in xrange(self.order):
            value_index = self.order - i - 1
            bias = bias + self.hidden_to_hidden_bias[i].T.dot(self.hidden_values[value_index]) + self.visible_to_hidden_bias[i].T.dot(self.visible_values[value_index])
        bias = bias + self.hidden_bias
        return bias

    def _sigmoid(self, x):
        return 1. / (1. + math.exp(-x))

    def _copy_array(self, src, shape):
        dst = numpy.zeros(shape)
        for i in xrange(len(src)):
            dst[i] = src[i]
        return dst

    def _shift_visible_vectors_back(self):
        for n in xrange(self.order):
            self.visible_values[n] = self._copy_array(self.visible_values[n+1],self.visible_values[n].shape)
