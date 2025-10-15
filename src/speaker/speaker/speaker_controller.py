import rclpy
from rclpy.node import Node
import playsound
from ament_index_python.packages import get_package_share_directory
import os

class SpeakerController(Node):
    def __init__(self):
        super().__init__('speaker_controller')
        self.get_logger().info('Speaker Controller Node has been started.')
        self.sound_dir = get_package_share_directory('speaker')
        # self.play_sound('sound/hello.mp3')
        

    def play_sound(self, sound_file):
        try:
            sound_file = os.path.join(self.sound_dir, 'sound', sound_file)
            playsound.playsound(sound_file)
            self.get_logger().info(f'Playing sound: {sound_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to play sound: {e}')
            
    
    
def main(args=None):
    rclpy.init(args=args)
    speaker_controller = SpeakerController()
    speaker_controller.play_sound('comming.mp3')
    rclpy.spin(speaker_controller)
    speaker_controller.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
