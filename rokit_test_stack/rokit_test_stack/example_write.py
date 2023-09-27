from database.database import create_test_results
from database.model import TestResults

class Example:
    def __init__(self):
        self.test_name = "MAX_VELOCITY_SLOPE"
        self.trial_number = 1
        self.robot_name = "MiR"
        self.tracking_object = "rokit_1"
        self.temperature = 26.9
        self.humidity = 78.99
        self.inclination = 0.0
        self.floor_type="wood"
        self.notes="nothing to note down"
        self.velocity=0.2366
        test_results = TestResults(
            test_name=str(self.test_name),
            trial_number=int(self.trial_number),
            robot_name=str(self.robot_name),
            tracking_object=str(self.tracking_object),
            temperature=float(self.temperature),
            humidity=float(self.humidity),
            inclination=float(self.inclination),
            floor_type=str(self.floor_type),
            notes=str(self.notes),
            velocity=float(self.velocity)
        )
        response = create_test_results(test_results)
        return response
    

test = Example()

