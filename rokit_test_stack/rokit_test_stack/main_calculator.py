import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position  # Make sure to replace 'your_package_name' with your actual package name
from rokit_test_stack.calculator import velocity_calculator
from rokit_test_stack.database import write_velocity
import time
START_POS = (0,0,0)
END_POS = (0,0,0)
START_FRAME = 0
class ViconSubscriber(Node):
    def __init__(self):
        super().__init__('vicon_subscriber')
        self.subscription = self.create_subscription(
            Position,
            '/vicon/rokit_1/rokit_1',
            self.vicon_callback,
            10
        )
        self.declare_parameter('testtype', "MAX_VELOCITY")
        self.declare_parameter('robottype', 'MiR')
        self.declare_parameter('tracking_object', "tracker_1")
        self.declare_parameter('trial_number', 1)        
        self.declare_parameter('temperature', 23.0)
        self.declare_parameter('humidity', 9.0)        
        self.declare_parameter('notes', 'none')
        self.declare_parameter('inclination', 0.0)
        self.declare_parameter('floor_type', 'wood')
        self.testtype = self.get_parameter('testtype').value
        self.robottype = self.get_parameter('robottype').value
        self.tracking_object = self.get_parameter('tracking_object').value
        self.trial_number = self.get_parameter('trial_number').value       
        self.temperature = self.get_parameter('temperature').value
        self.humidity = self.get_parameter('humidity').value        
        self.notes = self.get_parameter('notes').value
        self.inclination = self.get_parameter('inclination').value        
        self.floor_type = self.get_parameter('floor_type').value
        self.get_logger().info('This is the testrun {} for testtype {}'.format(self.trial_number, self.testtype))
        self.get_logger().info("Vicon subscriber initialized")
        self.calculation_start_position = (0,0,0)
        self.calculation_end_position = (0,0,0)
        self.calculation_start_frame = 0
        self.calculation_end_frame = 0
        self.meters_per_second = 0
    
    def vicon_callback(self, msg):
        translation = (msg.x_trans, msg.y_trans, msg.z_trans)
        orientation = (msg.x_rot,msg.y_rot,msg.z_rot, msg.w)
        frame_number = msg.frame_number
        if (self.testtype == 'MAX_VELOCITY'):
            self.check_velocity(translation, frame_number)   
        if (self.testtype =='MAX_VELOCITY_SLOPE'):
            self.check_velocity_on_slope(translation,frame_number)
        # Process the extracted data as needed
        self.get_logger().info(
            f"Received Vicon data:\n"
            f"Tranlsation: ({translation})\n"
            f"Frame Number: {frame_number}\n"
        )
        
        
    def check_velocity_on_slope (self, translation, frame_number):
        if (translation[0]>1100 and translation[0]<1300):
            self.calculation_start_position = translation
            self.calculation_start_frame = frame_number
            print(f"here is start_pose {self.calculation_start_position}")
        if (translation[0]<100 and translation[0]>1 and self.calculation_start_position != (0,0,0)):
            self.calculation_end_position = translation
            self.calculation_end_frame = frame_number
            print("Calculating Velocity on the slope now")
            self.meters_per_second = velocity_calculator.calculate(self.calculation_start_position, self.calculation_start_frame, self.calculation_end_position, self.calculation_end_frame);
            print(f'Meters per second calculated {self.meters_per_second}')
            write_velocity.write_content(self.testrun, self.testtype, self.meters_per_second )
            self.destroy_node()
            rclpy.shutdown()
            return
        
        
    def check_velocity_on_slope (self, translation, frame_number):
        if (translation[0]>1100 and translation[0]<1300):
            self.calculation_start_position = translation
            self.calculation_start_frame = frame_number
            print(f"here is start_pose {self.calculation_start_position}")
        if (translation[0]<100 and translation[0]>1 and self.calculation_start_position != (0,0,0)):
            self.calculation_end_position = translation
            self.calculation_end_frame = frame_number
            print("Calculating Velocity on the slope now")
            self.meters_per_second = velocity_calculator.calculate(self.calculation_start_position, self.calculation_start_frame, self.calculation_end_position, self.calculation_end_frame);
            print(f'Meters per second calculated {self.meters_per_second}')
            write_velocity.write_content(self)
            self.destroy_node()
            rclpy.shutdown()
            return
        
    def check_velocity (self, translation, frame_number):
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
            write_velocity.write_content(self)
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
