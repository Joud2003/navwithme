class PIDController:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point
        self.previous_error = 0

    def update(self, current_value):
        # Calculate the new error value = desired - real
        error = self.set_point - current_value
        # choosing P value = 1
        P_term = self.Kp * error
        # Choosing D value to be between 0 and 1
        D_term = self.Kd * (error - self.previous_error)
        # Updating error
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D
