import pybullet as p
import numpy as np
import pinocchio as pin
import hppfcl
from dynamic_graph_head import ThreadHead, SimHead, HoldPDController
from datetime import datetime
import dynamic_graph_manager_cpp_bindings
from mim_robots.robot_loader import load_bullet_wrapper, load_pinocchio_wrapper
from mim_robots.pybullet.env import BulletEnvWithGround
from mim_robots.robot_list import MiM_Robots
import pathlib
import os
python_path = pathlib.Path('.').absolute().parent/'StagewiseSQP'
os.sys.path.insert(1, str(python_path))

import launch_utils
from mpc_utils import transform_model_into_capsules



# # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# Choose experiment, load config and import controller  #  
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
SIM           = True
EXP_NAME      = 'binpicking_cssqp' # <<<<<<<<<<<<< Choose experiment here (cf. launch_utils)
config        = launch_utils.load_config_file(EXP_NAME)
MPCController = launch_utils.import_mpc_controller(EXP_NAME)
    
    
    
    
# # # # # # # # # # # #
# Import robot model  #
# # # # # # # # # # # #
pin_robot   = load_pinocchio_wrapper('iiwa_convex')




# # # # # # # # # # # # #
# Setup control thread  #
# # # # # # # # # # # # #
if SIM:
    from mpc_utils import display_ball
    # Sim env + set initial state 
    config['T_tot'] = 15              
    env = BulletEnvWithGround(p.GUI)
    robot_simulator = load_bullet_wrapper('iiwa_convex')
    env.add_robot(robot_simulator)
    q_init = np.asarray(config['q0'] )
    v_init = np.asarray(config['dq0'])
    robot_simulator.reset_state(q_init, v_init)
    robot_simulator.forward_robot(q_init, v_init)

    # <<<<< Customize your PyBullet environment here if necessary
    ## Adding the obstacle 
    OBSTACLE_POSE =pin.SE3(np.eye(3), np.array([0.4,0,1.0]))
    OBSTACLE_RADIUS = 0.05

    TARGET1 = pin.SE3(np.eye(3),np.asarray(config['target1']) )
    TARGET2 = pin.SE3(np.eye(3),np.asarray(config['target2']) )

    print(f'robot_simulator.pin_robot.model.upperPositionLimit : {robot_simulator.pin_robot.model.upperPositionLimit}')
    print(f'robot_simulator.pin_robot.model.lowerPositionLimit : {robot_simulator.pin_robot.model.lowerPositionLimit}')
    print(f'robot_simulator.pin_robot.model.velocityLimit : {robot_simulator.pin_robot.model.velocityLimit / 3}')
    print(f'robot_simulator.pin_robot.model.effortLimit : {robot_simulator.pin_robot.model.effortLimit / 2}')

    display_ball(OBSTACLE_POSE, OBSTACLE_RADIUS)
    display_ball(TARGET1, 5e-2, COLOR=np.concatenate((np.random.rand(3), np.ones(1))))
    display_ball(TARGET2, 5e-2, COLOR=np.concatenate((np.random.rand(3), np.ones(1))))

    # Creating the hppfcl shape
    OBSTACLE = hppfcl.Sphere(OBSTACLE_RADIUS)

    # Adding the shape to the collision model
    OBSTACLE_GEOM_OBJECT = pin.GeometryObject(
        "obstacle",
        0,
        0,
        OBSTACLE,
        OBSTACLE_POSE,
)
    robot_simulator.pin_robot.collision_model = transform_model_into_capsules(robot_simulator.pin_robot.collision_model)
    pin_robot.collision_model = transform_model_into_capsules(pin_robot.collision_model)

    robot_simulator.pin_robot.collision_model.addGeometryObject(OBSTACLE_GEOM_OBJECT)
    pin_robot.collision_model.addGeometryObject(OBSTACLE_GEOM_OBJECT)

    head = SimHead(robot_simulator, with_sliders=False)


# !!!!!!!!!!!!!!!!
# !! REAL ROBOT !!
# !!!!!!!!!!!!!!!!
else:
    config['T_tot'] = 400              
    path = MiM_Robots['iiwa'].dgm_path  
    print(path)
    head = dynamic_graph_manager_cpp_bindings.DGMHead(path)
    target = None
    env = None

ctrl = MPCController(head, pin_robot, config, run_sim=SIM)

thread_head = ThreadHead(
    1./config['ctrl_freq'],                                         # dt.
    HoldPDController(head, 50., 0.5, with_sliders=False),           # Safety controllers.
    head,                                                           # Heads to read / write from.
    [], 
    env                                                             # Environment to step.
)

thread_head.switch_controllers(ctrl)





# # # # # # # # #
# Data logging  #
# # # # # # # # # <<<<<<<<<<<<< Choose data save path & log config here (cf. launch_utils)
# prefix     = "/home/skleff/data_sqp_paper_croc2/constrained/circle/"
prefix     = "/tmp/"
suffix     = "_"+config['SOLVER'] +'_CODE_SPRINT'
LOG_FIELDS = launch_utils.get_log_config(EXP_NAME) 
# print(LOG_FIELDS)
# LOG_FIELDS = launch_utils.LOGS_NONE 
# LOG_FIELDS = launch_utils.SSQP_LOGS_MINIMAL 
# LOG_FIELDS = launch_utils.CSSQP_LOGS_MINIMAL 







# # # # # # # # # # # 
# Launch experiment #
# # # # # # # # # # # 
if SIM:
    thread_head.start_logging(int(config['T_tot']), prefix+EXP_NAME+"_SIM_"+str(datetime.now().isoformat())+suffix+".mds", LOG_FIELDS=LOG_FIELDS)
    thread_head.sim_run_timed(int(config['T_tot']))
    thread_head.stop_logging()
else:
    thread_head.start()
    thread_head.start_logging(50, prefix+EXP_NAME+"_REAL_"+str(datetime.now().isoformat())+suffix+".mds", LOG_FIELDS=LOG_FIELDS)
    
thread_head.plot_timing() # <<<<<<<<<<<<< Comment out to skip timings plot
