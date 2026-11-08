"""Kinematics, gaits and stability for the 18-DOF hexapod. Pure Python + numpy, no ROS."""
from .body_ik import BodyPoseResult, body_pose_ik, nominal_stance
from .checks import (JOINT_LIMIT, LEG_COLLISION, SERVO_SPEED, TOO_FEW_FEET, UNREACHABLE, UNSTABLE, Issue,
                     check_joint_limits, check_leg_collisions, check_pose, check_reachable, check_servo_speed,
                     leg_capsules, leg_clearances)
from .export import csv_header, write_csv
from .gait import (DEFAULT_STEP_HEIGHT, DEFAULT_STEP_LENGTH, DEFAULTS, PHASES, GaitGenerator, GaitParams, GaitSample, GaitTrajectory, VelocityCommand,
                   generate_gait, min_feet_for)
from .kinematics import (IKSolution, body_to_leg, feet_fk, feet_ik, leg_fk, leg_ik, leg_to_body, link_frames,
                         rpy_matrix)
from .model import JOINT_ORDER, RobotModel, find_parameters_file
from .stability import Stability, center_of_mass, convex_hull, stability_margin
