from abc import ABCMeta, abstractmethod
import math

class DeepBNBase(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def sample_network(self, vector=None):
        """Samples a vector from the network.

        Keyword arguments:
        vector -- An input vector for the network's visible layer.

        Returns:
        visible_units -- A 'numpy.array' containing the sampled values.

        """
        pass

    def _logistic(self, x):
        return 1. / (1. + math.exp(-x))
