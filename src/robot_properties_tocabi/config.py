# @namespace robot_properties_tocabi.config
""" This module includes configuration for the Tocabi.

    @file config.py
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np
from math import pi
import sys
import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from robot_properties_tocabi.resources import Resources


class TocabiAbstract(object):
    """ Abstract class used for all Tocabi robots. """

    # PID gains
    kp = 5.0
    kd = 0.1
    ki = 0.0

    # The Kt constant of the motor [Nm/A]: tau = I * Kt.
    motor_torque_constant = 0.025

    # Control time period.
    control_period = 0.001
    dt = control_period

    # MaxCurrent = 12 # Ampers
    max_current = 2

    # Maximum torques.
    max_torque = motor_torque_constant * max_current

    # Maximum control one can send, here the control is the current.
    max_control = max_current

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain = 1.0

    max_qref = pi

    base_link_name = "base_link"
    end_effector_names = ["R_AnkleRoll_Joint", "L_AnkleRoll_Joint"]

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path, se3.JointModelFreeFlyer()
        )
        
        jointsToLock = ['Waist1_Joint', 'Waist2_Joint', 'Upperbody_Joint', 
    'L_Shoulder1_Joint', 'L_Shoulder2_Joint', 'L_Shoulder3_Joint', 'L_Armlink_Joint', 'L_Elbow_Joint', 'L_Forearm_Joint', 'L_Wrist1_Joint', 'L_Wrist2_Joint',
 'Neck_Joint', 'Head_Joint',
    'R_Shoulder1_Joint', 'R_Shoulder2_Joint', 'R_Shoulder3_Joint', 'R_Armlink_Joint', 'R_Elbow_Joint', 'R_Forearm_Joint', 'R_Wrist1_Joint', 'R_Wrist2_Joint']
        # Get the joint IDs
        jointsToLockIDs = []
    
        for jn in range(len(jointsToLock)):
            jointsToLockIDs.append(robot.model.getJointId(jointsToLock[jn]))
    # Set initial configuration
    
        fixedJointConfig = np.matrix([0, 0, 0.80783, 0, 0, 0, 1, 
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0, 
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0,
    0, 0, 0,  
    0.2, 0.6, 1.5, -1.47, -1, 0 ,-1, 0, 
    0, 0, 
    -0.2, -0.6 ,-1.5, 1.47, 1, 0, 1, 0]).T
    
        robot = RobotWrapper.buildReducedRobot(robot, jointsToLockIDs, fixedJointConfig)
        
        return robot

    @classmethod
    def buildSimuRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.simu_urdf_path, cls.meshes_path, se3.JointModelFreeFlyer()
        )
        #robot.model.rotorInertia[6:] = cls.motor_inertia
        #robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names


class TocabiConfig(TocabiAbstract):
    robot_family = "tocabi"
    robot_name = "tocabi"
    
    # Here we use the same urdf as for the quadruped but without the freeflyer.
    resources = Resources(robot_name)
    meshes_path = resources.meshes_path
    simu_urdf_path = resources.simu_urdf_path
    urdf_path = resources.urdf_path

    # pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path, se3.JointModelFreeFlyer())
    #robot_model.rotorInertia[6:] = motor_inertia
    #robot_model.rotorGearRatio[6:] = motor_gear_ration

    mass = np.sum([i.mass for i in robot_model.inertias])

    base_name = "base_link"#robot_model.frames[2].name

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6
    
    # pinocchio model.
    pin_robot_wrapper = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path, se3.JointModelFreeFlyer()
    )

    jointsToLock = ['Waist1_Joint', 'Waist2_Joint', 'Upperbody_Joint', 
    'L_Shoulder1_Joint', 'L_Shoulder2_Joint', 'L_Shoulder3_Joint', 'L_Armlink_Joint', 'L_Elbow_Joint', 'L_Forearm_Joint', 'L_Wrist1_Joint', 'L_Wrist2_Joint',  
'Neck_Joint', 'Head_Joint',
    'R_Shoulder1_Joint', 'R_Shoulder2_Joint', 'R_Shoulder3_Joint', 'R_Armlink_Joint', 'R_Elbow_Joint', 'R_Forearm_Joint', 'R_Wrist1_Joint', 'R_Wrist2_Joint']
    # Get the joint IDs
    jointsToLockIDs = []
    
    for jn in range(len(jointsToLock)):
        jointsToLockIDs.append(pin_robot_wrapper.model.getJointId(jointsToLock[jn]))
    # Set initial configuration
    
    fixedJointConfig = np.matrix([0, 0, 0.80783, 0, 0, 0, 1, 
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0, 
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0,
    0, 0, 0,  
    0.2, 0.6, 1.5, -1.47, -1, 0 ,-1, 0, 
    0, 0, 
    -0.2, -0.6 ,-1.5, 1.47, 1, 0, 1, 0]).T
    
    pin_robot_wrapper.model = se3.buildReducedModel(pin_robot_wrapper.model, jointsToLockIDs, fixedJointConfig)
    
    # End effectors informations
    robot_model = pin_robot_wrapper.model

    end_eff_ids = []
    #for leg in ["FL", "FR"]:
    #    end_eff_ids.append(robot_model.getFrameId(leg + "_ANKLE"))
    end_eff_ids =["R_AnkleRoll_Joint", "L_AnkleRoll_Joint"]
    nb_ee = len(end_eff_ids)

    joint_names = ["R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint", "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint", "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint", "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(6))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]
    print(robot_model.names)
    # Define the initial state.
    initial_configuration = np.array(
        [0, 0, 0.80783, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]
    )

    initial_velocity = (6 + 12) * [
        0,
    ]

    q0 = np.zeros(robot_model.nq)
    q0[:] = initial_configuration
    v0 = np.zeros(robot_model.nv)
    v0[:] = initial_velocity
    a0 = np.zeros(robot_model.nv)

class TocabiHumanoidConfig(TocabiAbstract):
    robot_family = "tocabi"
    robot_name = "tocabi"
    
    resources = Resources(robot_name)
    meshes_path = resources.meshes_path
    #dgm_yaml_path = resources.dgm_yaml_path
    simu_urdf_path = resources.simu_urdf_path
    urdf_path = resources.urdf_path

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ration = 9.0

    # pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path, se3.JointModelFreeFlyer())

    mass = np.sum([i.mass for i in robot_model.inertias])

    base_name = robot_model.frames[2].name

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6
	
    # pinocchio model.
    pin_robot_wrapper = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path, se3.JointModelFreeFlyer()
    )
    
    jointsToLock = ['Waist1_Joint', 'Waist2_Joint', 'Upperbody_Joint', 
    'L_Shoulder1_Joint', 'L_Shoulder2_Joint', 'L_Shoulder3_Joint', 'L_Armlink_Joint', 'L_Elbow_Joint', 'L_Forearm_Joint', 'L_Wrist1_Joint', 'L_Wrist2_Joint',
 'Neck_Joint', 'Head_Joint',
    'R_Shoulder1_Joint', 'R_Shoulder2_Joint', 'R_Shoulder3_Joint', 'R_Armlink_Joint', 'R_Elbow_Joint', 'R_Forearm_Joint', 'R_Wrist1_Joint', 'R_Wrist2_Joint']
    # Get the joint IDs
    jointsToLockIDs = []
    
    for jn in range(len(jointsToLock)):
        jointsToLockIDs.append(pin_robot_wrapper.model.getJointId(jointsToLock[jn]))
    # Set initial configuration
    
    fixedJointConfig = np.matrix([0, 0, 0.80783, 0, 0, 0, 1, 
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0, 
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0,
    0, 0, 0,  
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

    pin_robot_wrapper.model = se3.buildReducedModel(pin_robot_wrapper.model, jointsToLockIDs, fixedJointConfig)
    
    # End effectors informations
    robot_model = pin_robot_wrapper.model
    end_eff_ids = []
    end_eff_ids =["R_AnkleRoll_Joint", "L_AnkleRoll_Joint"]
    nb_ee = len(end_eff_ids)

    joint_names = ["R_HipYaw_Joint", "R_HipRoll_Joint", "R_HipPitch_Joint", "R_Knee_Joint", "R_AnklePitch_Joint", "R_AnkleRoll_Joint", "L_HipYaw_Joint", "L_HipRoll_Joint", "L_HipPitch_Joint", "L_Knee_Joint", "L_AnklePitch_Joint", "L_AnkleRoll_Joint"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(9))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = np.array(
                [0, 0, 0.80783, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0]

    )

    initial_velocity = (6 + 12) * [
        0,
    ]

    q0 = np.zeros(robot_model.nq)
    q0[:] = initial_configuration
    v0 = np.zeros(robot_model.nv)
    v0[:] = initial_velocity
    a0 = np.zeros(robot_model.nv)
