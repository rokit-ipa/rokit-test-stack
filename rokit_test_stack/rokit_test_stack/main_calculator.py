import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position  # Make sure to replace 'your_package_name' with your actual package name
from rokit_test_stack.calculator import velocity_calculator
#from rokit_test_stack.database import write_velocity
from rokit_test_stack.database import database 
from rokit_test_stack.database import model
import time
START_POS = (0,0,0)
END_POS = (0,0,0)
START_FRAME = 0
class ViconSubscriber(Node):
    """Node that subscribes to the Vicon tacker messages and then estimate the velocity in meter per second.

    Args:
        Node (_type_): rclpy Class
    """
    def __init__(self):
        super().__init__('vicon_subscriber')
        self.declare_parameter('robot_name', 'MiR')
        self.declare_parameter('test_name', "")
        self.declare_parameter('tracking_object', "tracker_1")
        self.declare_parameter('trial_number', 1)        
        self.declare_parameter('temperature', 23.0)
        self.declare_parameter('humidity', 9.0)        
        self.declare_parameter('notes', 'none')
        self.declare_parameter('inclination', 0.0)
        self.declare_parameter('floor_type', 'wood')
        self.test_name = self.get_parameter('test_name').value
        self.robot_name = self.get_parameter('robot_name').value
        self.tracking_object = self.get_parameter('tracking_object').value
        self.trial_number = self.get_parameter('trial_number').value       
        self.temperature = self.get_parameter('temperature').value
        self.humidity = self.get_parameter('humidity').value        
        self.notes = self.get_parameter('notes').value
        self.inclination = self.get_parameter('inclination').value        
        self.floor_type = self.get_parameter('floor_type').value
        self.get_logger().info('This is the testrun {} for test_name {}'.format(self.trial_number, self.test_name))
        self.get_logger().info("Vicon subscriber initialized")
        self.calculation_start_position = (0,0,0)
        self.calculation_end_position = (0,0,0)
        self.calculation_start_frame = 0
        self.calculation_end_frame = 0
        self.meters_per_second = 0
        self.get_logger().info(f'/vicon/{self.tracking_object}/{self.tracking_object}')
        self.subscription = self.create_subscription(
            Position,
            f'/vicon/{self.tracking_object}/{self.tracking_object}',
            self.vicon_callback,
            10
        )
        
    def vicon_callback(self, msg):
        """callback function

        Args:
            msg (_type_): _description_
        """
        translation = (msg.x_trans, msg.y_trans, msg.z_trans)
        orientation = (msg.x_rot,msg.y_rot,msg.z_rot, msg.w)
        frame_number = msg.frame_number
        if (self.test_name == 'MAX_VELOCITY'):
            self.get_logger().info("I am calculating max_velocity")
            self.check_velocity(translation, frame_number)   
        if (self.test_name =='MAX_VELOCITY_SLOPE'):
            self.get_logger().info("I am calculating max_velocity_on_slope")
            self.check_velocity_on_slope(translation,frame_number)
        # Process the extracted data as needed
        # self.get_logger().info(
        #     f"Received Vicon data:\n"
        #     f"Tranlsation: ({translation})\n"
        #     f"Frame Number: {frame_number}\n"
        # )
        
           
    def check_velocity_on_slope (self, translation, frame_number):
        """estimate the velocity of the tracker in the slope

        Args:
            translation (_type_): _description_
            frame_number (_type_): _description_
        """
        if (translation[0]>1100 and translation[0]<1300):
            self.calculation_start_position = translation
            self.calculation_start_frame = frame_number
            print(f"here is start_pose {self.calculation_start_position}")
        if (translation[0]<100 and translation[0]>1 and self.calculation_start_position != (0,0,0)):
            self.calculation_end_position = translation
            self.calculation_end_frame = frame_number
            print("Calculating Velocity on the slope now")
            self.meters_per_second = velocity_calculator.calculate(self.calculation_start_position, self.calculation_start_frame, self.calculation_end_position, self.calculation_end_frame);
            #data to be written to the database should follow the data model defined in the models
            test_results = model.TestResults(
                test_name=str(self.test_name),
                trial_number=int(self.trial_number),
                robot_name=str(self.robot_name),
                tracking_object=str(self.tracking_object),
                temperature=float(self.temperature),
                humidity=float(self.humidity),
                inclination=float(self.inclination),
                floor_type=str(self.floor_type),
                notes=str(self.notes),
                velocity=float(self.meters_per_second)
            )
            response = database.create_test_results(test_results)
            self.get_logger().info(
                "Successfully wrote Data to DB"
            )
            self.destroy_node()
            rclpy.shutdown()
            return
        
    def check_velocity (self, translation, frame_number):
        """estimate the velocity of the tracker

        Args:
            translation (_type_): _description_
            frame_number (_type_): _description_
        """
        if (translation[0]>0 and translation[0]<100):
            self.calculation_start_position = translation
            self.calculation_start_frame = frame_number
            print(f"here is start_pose {self.calculation_start_position}")
        if (translation[0]>1100 and self.calculation_start_position != (0,0,0)):
            self.calculation_end_position = translation
            self.calculation_end_frame = frame_number
            print("Calculating Velocity now")
            self.meters_per_second = velocity_calculator.calculate(self.calculation_start_position, self.calculation_start_frame, self.calculation_end_position, self.calculation_end_frame);
            print(f'Meters per second calculated {self.meters_per_second}')
            
            test_results = model.TestResults(
                test_name=str(self.test_name),
                trial_number=int(self.trial_number),
                robot_name=str(self.robot_name),
                tracking_object=str(self.tracking_object),
                temperature=float(self.temperature),
                humidity=float(self.humidity),
                inclination=float(self.inclination),
                floor_type=str(self.floor_type),
                notes=str(self.notes),
                velocity=float(self.meters_per_second)
            )
            response = database.create_test_results(test_results)
            self.get_logger().info(
                "Successfully wrote Data to DB"
            )
            self.destroy_node()
            rclpy.shutdown()
            return

def main(args=None):
    rclpy.init(args=args)
    vicon_subscriber = ViconSubscriber()
    rclpy.spin(vicon_subscriber)
    vicon_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
