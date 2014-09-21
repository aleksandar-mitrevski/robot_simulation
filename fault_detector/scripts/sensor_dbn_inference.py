class SensorDbnInferenceEngine(object):
    def __init__(self, network):
        '''Defines an engine for performing inference in a dynamic Bayesian network representing sensor states and measurements.

        Keyword arguments:
        network -- A 'libpgm.dyndiscbayesiannetwork.DynDiscBayesianNetwork' object representing a dynamic Bayesian network.

        '''
        self.network = network
        self.network.toporder()
        self.sensor_dict = dict()

    def add_sensor(self, sensor_dict):
        '''
        Keyword arguments:
        sensor_dict -- A dictionary that uniquely identifies the sensor measurements that are to be monitored;
                       the values can be left null during initialisation, as they will later on represent the beliefs for each individual sensor.
        '''
        for key in sensor_dict.keys():
            self.sensor_dict[key] = dict()
            for i,state in enumerate(self.network.initial_Vdata['state']['vals']):
                self.sensor_dict[key][state] = self.network.initial_Vdata['state']['cprob'][i]

    def get_current_belief(self, sensor_key):
        belief = dict()
        for _,state in enumerate(self.network.initial_Vdata['state']['vals']):
            belief[state] = self.sensor_dict[sensor_key][state]
        return belief

    def filter(self, sensor_key, measurement):
        '''Performs a filtering update given a measurement.

        Keyword arguments:
        sensor_key -- The key of the sensor whose belief we want to update.
        measurement -- Value describing an observed measurement.

        '''
        state_values = list(self.network.initial_Vdata['state']['vals'])
        measurement_index = self.network.initial_Vdata['measurement']['vals'].index(str(measurement))

        #will store the updated belief after filtering
        updated_belief = dict(self.sensor_dict[sensor_key])

        for i,state in enumerate(state_values):
            #we take the measurement probability given the current state
            probability = self.network.twotbn_Vdata['measurement']['cprob']["['" + str(state) + "']"][measurement_index]

            prediction_probability = 0.
            for _,previous_state in enumerate(state_values):
                transition_probability = self.network.twotbn_Vdata['state']['cprob']["['" + str(previous_state) + "']"][i]
                previous_belief = self.sensor_dict[sensor_key][previous_state]
                prediction_probability = prediction_probability + (transition_probability * previous_belief)

            probability = probability * prediction_probability
            updated_belief[state] = probability

        normaliser = 0.
        for key in updated_belief.keys():
            normaliser = normaliser + updated_belief[key]

        for key in updated_belief.keys():
            self.sensor_dict[sensor_key][key] = updated_belief[key] / normaliser
