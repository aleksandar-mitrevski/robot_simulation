class SensorDbnInferenceEngine(object):
    def __init__(self, network):
        """Defines an engine for performing inference in a dynamic Bayesian network representing sensor states and measurements.

        Keyword arguments:
        network -- A 'libpgm.dyndiscbayesiannetwork.DynDiscBayesianNetwork' object representing a dynamic Bayesian network.

        """
        self.network = network
        self.network.toporder()

    def get_current_belief(self):
        belief = dict()
        for i,state in enumerate(self.network.initial_Vdata['state']['vals']):
            belief[state] = self.network.initial_Vdata['state']['cprob'][i]
        return belief

    def filter(self, measurement):
        """Performs a filtering update given a measurement.

        Keyword arguments:
        measurement -- Value describing an observed measurement.

        """
        state_values = list(self.network.initial_Vdata['state']['vals'])
        measurement_index = self.network.initial_Vdata['measurement']['vals'].index(str(measurement))

        #will store the updated belief after filtering
        updated_belief = list(self.network.initial_Vdata['state']['cprob'])

        for i,state in enumerate(state_values):
            #we take the measurement probability given the current state
            probability = self.network.twotbn_Vdata['measurement']['cprob']["['" + str(state) + "']"][measurement_index]

            prediction_probability = 0.
            for j,previous_state in enumerate(state_values):
                transition_probability = self.network.twotbn_Vdata['state']['cprob']["['" + str(previous_state) + "']"][i]
                previous_belief = self.network.initial_Vdata['state']['cprob'][j]
                prediction_probability = prediction_probability + (transition_probability * previous_belief)

            probability = probability * prediction_probability
            updated_belief[i] = probability

        normaliser = sum(updated_belief)
        for i in xrange(len(updated_belief)):
            updated_belief[i] = updated_belief[i] / normaliser
        self.network.initial_Vdata['state']['cprob'] = updated_belief
