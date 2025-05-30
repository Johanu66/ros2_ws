import rclpy
from rclpy.node import Node
import subprocess
import tempfile
import os
from urdf_parser_py.urdf import URDF

def extract_robot_info(xacro_path):
    """
    Convertit un fichier Xacro en URDF, parse le modèle avec urdf_parser_py,
    et retourne les infos utiles sous forme de dictionnaire.
    """

    # 1. Conversion Xacro ➜ URDF XML
    urdf_xml = subprocess.check_output(['xacro', xacro_path]).decode("utf-8")

    # 2. Parsing URDF (depuis string)
    robot = URDF.from_xml_string(urdf_xml)

    # 3. Extraction des informations
    data = {}

    # 3.1 Joints
    joints = []
    for j in robot.joints:
        joint_info = {
            "name": j.name,
            "type": j.type,
            "parent": j.parent,
            "child": j.child,
            "limit": vars(j.limit) if j.limit else None,
            "axis": j.axis,
            "origin": {
                "xyz": j.origin.xyz if j.origin else None,
                "rpy": getattr(j.origin, 'rpy', None) if j.origin else None,
            },
        }
        joints.append(joint_info)

    # 3.2 Links
    links = []
    for l in robot.links:
        link_info = {
            "name": l.name,
            "collisions": len(l.collisions or []),
            "visuals": len(l.visuals or []),
            "inertial": vars(l.inertial) if l.inertial else None,
        }
        links.append(link_info)

    # 3.3 End-effectors candidates (children with no further child)
    child_links = set(j['child'] for j in joints)
    parent_links = set(j['parent'] for j in joints)
    terminal_links = list(child_links - parent_links)

    # 4. Assemble final data
    data['robot_name'] = robot.name
    data['joints'] = joints
    data['links'] = links
    data['joint_names'] = [j['name'] for j in joints]
    data['link_names'] = [l['name'] for l in links]
    data['ee_link_candidates'] = terminal_links
    data['root_link'] = robot.get_root()

    return data

from glob import glob
class RobotInfoExtractor(Node):
    # def __init__(self):
    #     super().__init__('extractor')
    #     self.declare_parameter('xacro_path', '/home/lab/ros2_ws/src/arms_sim/urdf/gen3_lite.urdf.xacro')
    #     path = self.get_parameter('xacro_path').value
    #     info = extract_robot_info(path)
    #     self.get_logger().info(str(info))
    #     # DO NOT shutdown here

    def __init__(self):
        super().__init__('extractor')

        urdf_dir = '/home/lab/ros2_ws/src/arms_sim/urdf/'
        # Cherche tous les fichiers xacro et urdf dans le dossier
        files = glob(os.path.join(urdf_dir, '*.xacro')) + glob(os.path.join(urdf_dir, '*.urdf'))

        for file_path in files:
            self.get_logger().info(f"Extraction pour le fichier: {file_path}")
            try:
                info = extract_robot_info(file_path)
                self.get_logger().info(str(info))
            except Exception as e:
                self.get_logger().error(f"Erreur avec le fichier {file_path}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoExtractor()

    # If you want to spin:
    rclpy.spin(node)

    # After spin returns, shutdown
    node.destroy_node()
    rclpy.shutdown()

