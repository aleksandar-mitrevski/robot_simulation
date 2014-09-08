class SensorMeasurements(object):
    def __init__(self, front=0., left_diagonal=0., left_back_diagonal=0., left=0., right_diagonal=0., right_back_diagonal=0., right=0.):
        self.front = front
        self.left_diagonal = left_diagonal
        self.left_back_diagonal = left_back_diagonal
        self.left = left
        self.right_diagonal = right_diagonal
        self.right_back_diagonal = right_back_diagonal
        self.right = right

    def less_than(self, threshold):
        return self.front < threshold or self.left_diagonal < threshold or self.left < threshold or self.right_diagonal < threshold or self.right < threshold
