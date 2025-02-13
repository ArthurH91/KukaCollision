import numpy as np
import pinocchio as pin 
import mim_solvers

import time

from croco_mpc_utils.ocp_constraints import OptimalControlProblemClassicalWithConstraints
import croco_mpc_utils.pinocchio_utils as pin_utils

from utils.reduced_model import get_controlled_joint_ids

from croco_mpc_utils.utils import CustomLogger, GLOBAL_LOG_LEVEL, GLOBAL_LOG_FORMAT
logger = CustomLogger(__name__, GLOBAL_LOG_LEVEL, GLOBAL_LOG_FORMAT).logger



# @profile
def solveOCP(q, v, solver, max_sqp_iter, max_qp_iter, target_reach):
    t = time.time()
    # Update initial state + warm-start
    x = np.concatenate([q, v])
    solver.problem.x0 = x
    
    xs_init = list(solver.xs[1:]) + [solver.xs[-1]]
    xs_init[0] = x
    us_init = list(solver.us[1:]) + [solver.us[-1]] 
    
    # Update OCP 
    # Updates nodes between node_id and terminal node 
    for k in range( solver.problem.T ):
        solver.problem.runningModels[k].differential.costs.costs["translation"].active = True
        solver.problem.runningModels[k].differential.costs.costs["translation"].cost.residual.reference = target_reach
        solver.problem.runningModels[k].differential.costs.costs["translation"].weight = 100.
    solver.problem.terminalModel.differential.costs.costs["translation"].active = True
    solver.problem.terminalModel.differential.costs.costs["translation"].cost.residual.reference = target_reach
    solver.problem.terminalModel.differential.costs.costs["translation"].weight = 100.

    solver.max_qp_iters = max_qp_iter
    solver.solve(xs_init, us_init, maxiter=max_sqp_iter, isFeasible=False)
    solve_time = time.time()
    
    return  solver.us[0], solver.xs[1], solver.K[0], solve_time - t, solver.iter, solver.cost, solver.constraint_norm, solver.gap_norm, solver.qp_iters, solver.KKT




class KukaBinPickingCSSQP:

    def __init__(self, head, pin_robot, config, run_sim, locked_joints=[]):
        """
        Input:
            head              : thread head
            pin_robot         : pinocchio wrapper
            config            : MPC config yaml file
            run_sim           : boolean sim or real
        """
        self.robot   = pin_robot
        self.head    = head
        self.RUN_SIM = run_sim
        self.joint_positions  = head.get_sensor('joint_positions')
        self.joint_velocities = head.get_sensor("joint_velocities")
        self.joint_accelerations = head.get_sensor("joint_accelerations")
        if not self.RUN_SIM:
            self.joint_torques     = head.get_sensor("joint_torques_total")
            self.joint_ext_torques = head.get_sensor("joint_torques_external")
            self.joint_cmd_torques = head.get_sensor("joint_torques_commanded")      


        self.nq = self.robot.model.nq
        self.nv = self.robot.model.nv

        logger.warning("Controlled model dimensions : ")
        logger.warning(" nq = "+str(self.nq))
        logger.warning(" nv = "+str(self.nv))

        # Controlled ids (reduced model)
        self.controlled_joint_ids = get_controlled_joint_ids('iiwa_ft_sensor_shell', locked_joints=locked_joints)
        logger.warning("Controlled joint ids = "+str(self.controlled_joint_ids))
        self.fixed_ids = [i for i in range(7) if i not in self.controlled_joint_ids]
        logger.warning("Fixed joint ids = "+str(self.fixed_ids))
        self.gain_P = 100.
        self.gain_D = 30.

        # Config
        self.config = config
        if(self.RUN_SIM):
            self.q0 = np.asarray(config['q0'])[self.controlled_joint_ids]
            self.v0 = self.joint_velocities[self.controlled_joint_ids]
        else:
            self.q0 = self.joint_positions[self.controlled_joint_ids]
            self.v0 = self.joint_velocities[self.controlled_joint_ids]
        self.x0 = np.concatenate([self.q0, self.v0])
        
        self.Nh = int(self.config['N_h'])
        self.dt_ocp  = self.config['dt']
        self.dt_ctrl = 1./self.config['ctrl_freq']
        self.OCP_TO_CTRL_RATIO = int(self.dt_ocp/self.dt_ctrl)

        # Create OCP 
        problem = OptimalControlProblemClassicalWithConstraints(self.robot, self.config).initialize(self.x0)
        # Initialize the solver
        if(config['SOLVER'] == 'proxqp'):
            logger.warning("Using the ProxQP solver.")
            self.solver = mim_solvers.SolverProxQP(problem)
        elif(config['SOLVER'] == 'cssqp'):
            logger.warning("Using the CSSQP solver.")
            self.solver = mim_solvers.SolverCSQP(problem)
        self.solver.with_callbacks         = self.config['with_callbacks']
        self.solver.use_filter_line_search = self.config['use_filter_line_search']
        self.solver.filter_size            = self.config['filter_size']
        self.solver.warm_start             = self.config['warm_start']
        self.solver.termination_tolerance  = self.config['solver_termination_tolerance']
        self.solver.max_qp_iters           = self.config['max_qp_iter']
        self.solver.eps_abs                = self.config['qp_termination_tol_abs']
        self.solver.eps_rel                = self.config['qp_termination_tol_rel']
        self.solver.warm_start_y           = self.config['warm_start_y']
        self.solver.reset_rho              = self.config['reset_rho']  
        self.solver.regMax                 = 1e6
        self.solver.reg_max                = 1e6
        
        # De-activate the constraint initially
        for i in range(self.Nh):
            for nc in range(len(self.robot.collision_model.collisionPairs)):
                self.solver.problem.runningModels[i].differential.constraints.constraints['collisionBox_'+str(nc)].constraint.active = False

        # Allocate MPC data
        self.K = self.solver.K[0]
        self.x_des = self.solver.xs[0]
        self.tau_ff = self.solver.us[0]
        self.tau = self.tau_ff.copy() ; self.tau_riccati = np.zeros(self.tau.shape)

        # Initialize torque measurements 
        if(self.RUN_SIM):
            logger.debug("Initial torque measurement signal : simulation --> use u0 = g(q0)")
            self.u0 = pin_utils.get_u_grav(self.q0, self.robot.model, np.zeros(self.robot.model.nq))
            self.joint_torques_total    = self.u0
            self.joint_torques_measured = self.u0
        # DANGER ZONE 
        else:
            logger.warning("Initial torque measurement signal : real robot --> use sensor signal 'joint_torques_total' ")
            self.joint_torques_total    = head.get_sensor("joint_torques_total")
            logger.warning("      >>> Correct minus sign in measured torques ! ")
            self.joint_torques_measured = -self.joint_torques_total 


        # Poses to follow
        self.target1 = np.asarray(self.config['target1']) 
        self.target2 = np.asarray(self.config['target2']) 

        # Targets over one horizon (initially = absolute target position)
        self.target_position = self.target1
        self.target_position_x = float(self.target_position[0]) 
        self.target_position_y = float(self.target_position[1]) 
        self.target_position_z = float(self.target_position[2])

        self.TASK_PHASE      = 1
        self.NH_SIMU   = int(self.Nh*self.dt_ocp/self.dt_ctrl)
        self.T_CYCLE  = int(self.config['T_CYCLE'])
        logger.debug("Size of MPC horizon in ctrl cycles = "+str(self.NH_SIMU))
        logger.debug("Start of circle phase in ctrl cycles = "+str(self.T_CYCLE))
        logger.debug("OCP to ctrl time ratio = "+str(self.OCP_TO_CTRL_RATIO))

        # Solver logs
        self.t_child         = 0
        self.cost            = 0
        self.cumulative_cost = 0
        self.gap_norm        = np.inf
        self.constraint_norm = np.inf
        self.qp_iters        = 0
        self.KKT             = np.inf


    def warmup(self, thread):
        self.max_sqp_iter = 10  
        self.max_qp_iter  = 100   
        self.u0 = pin_utils.get_u_grav(self.q0, self.robot.model, np.zeros(self.robot.model.nq))
        self.solver.xs = [self.x0 for i in range(self.config['N_h']+1)]
        self.solver.us = [self.u0 for i in range(self.config['N_h'])]
        self.tau_ff, self.x_des, self.K, self.t_child, self.ddp_iter, self.cost, self.constraint_norm, self.gap_norm, self.qp_iters, self.KKT = solveOCP(self.joint_positions[self.controlled_joint_ids], 
                                                                                          self.joint_velocities[self.controlled_joint_ids], 
                                                                                          self.solver, 
                                                                                          self.max_sqp_iter, 
                                                                                          self.max_qp_iter, 
                                                                                          self.target_position)
        self.cumulative_cost += self.cost
        self.max_sqp_iter = self.config['maxiter']
        self.max_qp_iter  = self.config['max_qp_iter']

    def run(self, thread):        
        # # # # # # # # # 
        # Read sensors  #
        # # # # # # # # # 
        q = self.joint_positions[self.controlled_joint_ids]
        v = self.joint_velocities[self.controlled_joint_ids]

        # When getting torque measurement from robot, do not forget to flip the sign
        if(not self.RUN_SIM):
            self.joint_torques_measured = -self.joint_torques_total  

        # # # # # # # # # 
        # # Update OCP  #
        # # # # # # # # # 


        time = int(thread.ti/100)
        if( time > self.T_CYCLE):
            for i in range(self.Nh):
                for nc in range(len(self.robot.collision_model.collisionPairs)):
                    self.solver.problem.runningModels[i].differential.constraints.constraints['collisionBox_'+str(nc)].constraint.active = True

        if time % self.T_CYCLE < self.T_CYCLE/2:
            self.TASK_PHASE = 1
            self.target_position_x = float( self.target1[0] )
            self.target_position_y = float(self.target1[1]) 
            self.target_position_z = float(self.target1[2])
            self.target_position = self.target1
        else:
            self.TASK_PHASE = 2
            self.target_position_x = float(self.target2[0]) 
            self.target_position_y = float(self.target2[1]) 
            self.target_position_z = float(self.target2[2])
            self.target_position = self.target2

        # # # # # # #  
        # Solve OCP #
        # # # # # # #  
        self.tau_ff, self.x_des, self.K, self.t_child, self.ddp_iter, self.cost, self.constraint_norm, self.gap_norm, self.qp_iters, self.KKT = solveOCP(q, 
                                                                                          v, 
                                                                                          self.solver, 
                                                                                          self.max_sqp_iter, 
                                                                                          self.max_qp_iter, 
                                                                                          self.target_position)

        # # # # # # # # 
        # Send policy #
        # # # # # # # #

        # Compute gravity
        self.tau_gravity = pin.rnea(self.robot.model, self.robot.data, self.joint_positions[self.controlled_joint_ids], np.zeros(self.nv), np.zeros(self.nv))

        if(self.RUN_SIM == False):
            self.tau_ff -= self.tau_gravity

        if(not self.RUN_SIM and len(self.fixed_ids) > 0):
            self.tau_PD   = -self.gain_P * self.joint_positions[self.fixed_ids] - self.gain_D * self.joint_velocities[self.fixed_ids]
            self.tau_full = np.zeros(7)
            self.tau_full[self.controlled_joint_ids] = self.tau_ff.copy()
            self.tau_full[self.fixed_ids] = self.tau_PD.copy()
            self.tau = self.tau_full.copy()
        else:
            self.tau = self.tau_ff.copy()

        ###### DANGER SEND ONLY GRAV COMP
        # self.tau = np.zeros_like(self.tau_full)
        
        self.head.set_control('ctrl_joint_torques', self.tau)     


        pin.framesForwardKinematics(self.robot.model, self.robot.data, q)
