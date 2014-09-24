from abc import ABCMeta, abstractmethod
import math

class DeepBNBase(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def train(self, data, epochs=100, learning_rate=0.1):
        """Trains the belief network with a given set of training vectors.

        Keyword arguments:
        data -- A 'numpy.array' containing data for training the RBM. Each row of the array should be a training vector of dimension 'number_visible_units'.
        epochs -- The number of iterations of the learning algorithm (default 100).
        learning_rate -- The algorithm's learning rate (default).

        """
        pass

    @abstractmethod
    def sample_network(self, vector=None):
        """Samples a vector from the network.

        Keyword arguments:
        vector -- An input vector for the network's visible layer.

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        pass

    @abstractmethod
    def _sample_layer(self, layer, k):
        """Samples a visible vector at the layer-th layer of the network.
        Uses Contrastive Divergence for sampling the values.

        Keyword arguments:
        layer -- The layer at which we want to sample.
        k -- The number of samples created by Contrastive Divergence before a sample is accepted.

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        pass

    def _logistic(self, x):
        return 1. / (1. + math.exp(-x))
