import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(

        get_package_share_directory('process_signal'),
        'config',
        'params.yaml'
    )

    generation_node = Node(
        package = 'process_signal',
        executable = 'signal_generator',
        output = 'screen',
        parameters = [config]
    )

    reconstruction_node = Node(

        package = 'process_signal',
        executable = 'signal_reconstruction',
        output = 'screen'
    )

    rqt_plot_node = Node(

        package = 'rqt_plot',
        executable = 'rqt_plot',
        parameters=[{'args': '/signal/data /signal_reconstructed/data'}],

    )

    return LaunchDescription([generation_node, reconstruction_node, rqt_plot_node])

