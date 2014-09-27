import json

from libpgm.nodedata import NodeData
from libpgm.graphskeleton import GraphSkeleton
from libpgm.dyndiscbayesiannetwork import DynDiscBayesianNetwork

from sensor_dbn_inference import SensorDbnInferenceEngine
from sensor_trbm_inference import SensorTRBMInferenceEngine, SensorTRBMContinuousInferenceEngine
from sensor_deep_bn_inference import SensorDeepBNInferenceEngine, SensorDeepBNContinuousInferenceEngine

class DBNFaultDetector(object):
    def __init__(self, dbn_file_name):
        self.dbn_file_name = dbn_file_name
        self.inference_engines = dict()

    def add_sensor(self, sensor_keys):
        for key in sensor_keys:
            network_file = open(self.dbn_file_name, 'r')
            network_file_data = eval(network_file.read())

            network_skeleton = GraphSkeleton()
            network_skeleton.V = network_file_data["V"]
            network_skeleton.E = network_file_data["E"]

            self.network = DynDiscBayesianNetwork()
            self.network.V = network_skeleton.V
            self.network.E = network_skeleton.E
            self.network.initial_Vdata = network_file_data["initial_Vdata"]
            self.network.twotbn_Vdata = network_file_data["twotbn_Vdata"]

            self.inference_engines[key] = SensorDbnInferenceEngine(self.network)

    def get_current_belief(self, sensor_key):
        return self.inference_engines[sensor_key].get_current_belief()

    def update_belief(self, sensor_key, measurement):
        mapped_measurement = self.map_measurement(measurement)
        self.inference_engines[sensor_key].filter(mapped_measurement)

    def map_measurement(self, measurement):
        if measurement > 1e-4:
            return '1'
        return '0'


class TRBMFaultDetector(object):
    def __init__(self, weights_file_name_base):
        self.weights_file_name_base = weights_file_name_base
        self.inference_engines = dict()

    def add_sensor(self, sensor_keys):
        for key in sensor_keys:
            ########################################################
            # TODO: define a file format for the connection weights 
            # and read the necessary file(s) here
            ########################################################
            #self.inference_engines[key] = SensorTRBMInferenceEngine(self.network)

    def check_measurement(self, sensor_key, measurement):
        mapped_measurement = self.map_measurement(measurement)
        return self.inference_engines[sensor_key].detect_fault(mapped_measurement)

    def map_measurement(self, measurement):
        for i in xrange(len(measurement)):
            if measurement[i] > 1e-4:
                measurement[i] = 1
            else:
                measurement[i] = 0
        return measurement


class TRBMContinuousFaultDetector(object):
    def __init__(self, weights_file_name_base):
        self.weights_file_name_base = weights_file_name_base
        self.inference_engines = dict()

    def add_sensor(self, sensor_keys):
        for key in sensor_keys:
            ########################################################
            # TODO: define a file format for the connection weights 
            # and read the necessary file(s) here
            ########################################################
            #self.inference_engines[key] = SensorTRBMContinuousInferenceEngine(self.network)

    def check_measurement(self, sensor_key, measurement):
        mapped_measurement = self.map_measurement(measurement)
        return self.inference_engines[sensor_key].detect_fault(mapped_measurement)

    def map_measurement(self, measurement):
        for i in xrange(len(measurement)):
            if measurement[i] > 1e-4:
                measurement[i] = 1
            else:
                measurement[i] = 0
        return measurement


class DeepBNFaultDetector(object):
    def __init__(self, weights_file_name_base):
        self.weights_file_name_base = weights_file_name_base
        self.inference_engines = dict()

    def add_sensor(self, sensor_keys):
        for key in sensor_keys:
            ########################################################
            # TODO: define a file format for the connection weights 
            # and read the necessary file(s) here
            ########################################################
            #self.inference_engines[key] = SensorDeepBNInferenceEngine(self.network)

    def check_measurement(self, sensor_key, measurement):
        mapped_measurement = self.map_measurement(measurement)
        return self.inference_engines[sensor_key].detect_fault(mapped_measurement)

    def map_measurement(self, measurement):
        for i in xrange(len(measurement)):
            if measurement[i] > 1e-4:
                measurement[i] = 1
            else:
                measurement[i] = 0
        return measurement


class DeepBNContinuousFaultDetector(object):
    def __init__(self, weights_file_name_base):
        self.weights_file_name_base = weights_file_name_base
        self.inference_engines = dict()

    def add_sensor(self, sensor_keys):
        for key in sensor_keys:
            ########################################################
            # TODO: define a file format for the connection weights 
            # and read the necessary file(s) here
            ########################################################
            #self.inference_engines[key] = SensorDeepBNContinuousInferenceEngine(self.network)

    def check_measurement(self, sensor_key, measurement):
        mapped_measurement = self.map_measurement(measurement)
        return self.inference_engines[sensor_key].detect_fault(mapped_measurement)

    def map_measurement(self, measurement):
        for i in xrange(len(measurement)):
            if measurement[i] > 1e-4:
                measurement[i] = 1
            else:
                measurement[i] = 0
        return measurement
