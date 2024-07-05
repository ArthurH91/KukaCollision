from plot_utils import SimpleDataPlotter
from mim_data_utils import DataReader
from croco_mpc_utils.pinocchio_utils import *
import numpy as np
import matplotlib.pyplot as plt 

import pathlib
import os
python_path = pathlib.Path('.').absolute().parent/'kuka_dgh'
os.sys.path.insert(1, str(python_path))
from demos import launch_utils

from utils.reduced_model import get_controlled_joint_ids
from mim_robots.robot_loader import load_pinocchio_wrapper

# Load robot model
LOCKED_JOINTS = ['A7']
pinrobot = load_pinocchio_wrapper('iiwa_convex', locked_joints=LOCKED_JOINTS)
model    = pinrobot.model
data     = model.createData()
frameId  = model.getFrameId('contact')
nq = model.nq ; nv = model.nv ; nc = 1
# Overwrite effort limit as in DGM
model.effortLimit = np.array([100, 100, 50, 50, 20, 10, 10])
controlled_joint_ids = get_controlled_joint_ids('iiwa_convex', locked_joints=LOCKED_JOINTS)




# Load config file
SIM           = False
EXP_NAME      = 'binpicking_cssqp' # <<<<<<<<<<<<< Choose experiment here (cf. launch_utils)
config        = launch_utils.load_config_file(EXP_NAME)


# Create data Plottger
s = SimpleDataPlotter(dt=1./config['ctrl_freq'])

if(SIM):
    data_path = '/tmp/'
    data_name = 'binpicking_cssqp_REAL_2024-07-05T18:04:23.178979' 
    
else:
    data_path = '/tmp/'
    data_name = 'binpicking_cssqp_REAL_2024-07-05T18:14:25.652491' 
    
r = DataReader(data_path+data_name+'.mds')
N = r.data['absolute_time'].shape[0]
print("Total number of control cycles = ", N)

fig, ax = plt.subplots(7, 1, sharex='col') 
ax[0].plot(r.data['qp_iters'], label='qp iters')
ax[1].plot(r.data['constraint_norm'], label='constraint_norm')
ax[2].plot(r.data['gap_norm'], label='gap_norm')
ax[3].plot(r.data['KKT'], label='KKT residual')
ax[3].plot(N*[config['solver_termination_tolerance']], label= 'KKT residual tolerance', color = 'r')
ax[4].plot(r.data['ddp_iter'], label='# solver iterations')
ax[5].plot(r.data['t_child']*1000, label='OCP solve time')
ax[5].plot(N*[1000./config['ctrl_freq']], label= 'dt_MPC', color='r')
ax[6].plot((r.data['timing_control'])* 1000, label='Control cycle time')
ax[6].plot(N*[1000./config['ctrl_freq']], label= 'dt_MPC', color='r')
for i in range(len(ax)):
    ax[i].legend()
    
    
# Limits
if(EXP_NAME == 'binpicking_cssqp'):
    s.plot_joint_pos( [r.data['joint_positions'][:,controlled_joint_ids], 
                       r.data['x_des'][:,:nq]], 
                    ['Measured', 
                     'Predicted',
                     'lb',
                     'ub'], 
                    ['r', 'b', 'k', 'k'],
                    ylims=[model.lowerPositionLimit, model.upperPositionLimit],
                    linestyle=['solid', 'solid', 'dotted', 'dotted'])

    s.plot_joint_vel( [r.data['joint_velocities'][:,controlled_joint_ids], 
                       r.data['x_des'][:,nq:]], 
                    ['Measured', 
                     'Predicted'], 
                    ['r', 'b', 'k', 'k'],
                    ylims=[model.velocityLimit, -model.velocityLimit],
                    linestyle=['solid', 'solid', 'dotted', 'dotted'])

# For SIM robot only
if(SIM):
    s.plot_joint_tau( [r.data['tau'], 
                       r.data['tau_ff']],
                    #    r.data['tau_riccati'], 
                    #    r.data['tau_gravity']], 
                      ['total', 
                       'ff', 
                       'riccati', 
                       'gravity'], 
                      ['r', 
                       'g', 
                       'b', 
                       [0.2, 0.2, 0.2, 0.5]],
                      ylims=[-model.effortLimit/2, +model.effortLimit/2] )
    print("u bound = \n", model.effortLimit/2)
# For REAL robot only !! DEFINITIVE FORMULA !!
else:
    # Our self.tau was subtracted gravity, so we add it again
    # joint_torques_measured DOES include the gravity torque from KUKA
    # There is a sign mismatch in the axis so we use a minus sign
    s.plot_joint_tau( [-r.data['joint_cmd_torques'][:, controlled_joint_ids], 
                       r.data['joint_torques_measured'][:,controlled_joint_ids], 
                       r.data['tau'][:,controlled_joint_ids] + r.data['tau_gravity'], 
                       r.data['tau_ff']], 
                  ['-cmd (FRI)', 
                   'Measured', 
                   'Desired (sent to robot) [+g(q)]', 
                   'tau_ff (OCP solution) [+g(q)]', 
                   'Measured - EXT'], 
                  ['k', 'r', 'b', 'g', 'y'],
                  ylims=[-model.effortLimit, +model.effortLimit],
                  linestyle=['dotted', 'solid', 'solid', 'solid', 'solid'])


p_mea = get_p_(r.data['joint_positions'][:, controlled_joint_ids], pinrobot.model, pinrobot.model.getFrameId('contact'))
p_des = get_p_(r.data['x_des'][:,:nq], pinrobot.model, pinrobot.model.getFrameId('contact'))
target_position = r.data['target_position'] #np.zeros((N,3))
# target_position[0] = r.data['target_position_x'][0]
# target_position[1] = r.data['target_position_y'][0]
# target_position[2] = r.data['target_position_z'][0]

s.plot_ee_pos([p_mea, 
                p_des,
                target_position],  
            ['Measured', 
                'Predicted',
                'Reference'], 
            ['r', 'b', 'g'], 
            linestyle=['solid', 'dotted', 'solid'],
            ylims=[-np.ones(3), +1.5*np.ones(3)])
plt.show()