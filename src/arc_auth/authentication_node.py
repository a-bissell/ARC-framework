import rclpy
from rclpy.node import Node
from arc_auth_interfaces.msg import AuthenticationRequest, AuthenticationResponse
from arc_auth_interfaces.srv import EnrollUser
from .authentication_core import AuthenticationCore, AuthenticationFactor, AccessLevel
from .sensors.sensor_manager import SensorManager
from .sensors.facial_sensor import FacialRecognitionSensor
import time

class AuthenticationNode(Node):
    def __init__(self):
        super().__init__('authentication_node')
        
        self.auth_core = AuthenticationCore()
        self.sensor_manager = SensorManager()
        
        # Initialize sensors
        self.setup_sensors()
        
        # Create authentication service
        self.auth_service = self.create_service(
            AuthenticationRequest,
            'authenticate_user',
            self.handle_authentication
        )
        
        # Create enrollment service
        self.enroll_service = self.create_service(
            EnrollUser,
            'enroll_user',
            self.handle_enrollment
        )
        
        self.get_logger().info('Authentication node started')

    async def setup_sensors(self):
        # Register facial recognition sensor
        facial_sensor = FacialRecognitionSensor()
        await self.sensor_manager.register_sensor(facial_sensor, {
            'camera_id': 0,
            'min_confidence': 0.6
        })
        
        # Add more sensors here as needed
        
    async def handle_authentication(self, request, response):
        # Get readings from all sensors
        sensor_readings = await self.sensor_manager.verify_user(request.user_id)
        
        # Convert to authentication factors
        factors = [
            AuthenticationFactor(
                factor_id=sensor_id,
                confidence=confidence,
                timestamp=time.time(),
                metadata={}
            )
            for sensor_id, confidence in sensor_readings.items()
        ]
        
        # Get access level
        access_level = self.auth_core.determine_access_level(
            request.user_id,
            factors
        )
        
        # Prepare response
        response.access_level = access_level.value
        response.timestamp = self.get_clock().now().to_msg()
        
        return response

    def handle_enrollment(self, request, response):
        # TODO: Implement enrollment logic
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AuthenticationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 