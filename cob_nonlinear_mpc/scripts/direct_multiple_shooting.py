#
#     This file is part of CasADi.
#
#     CasADi -- A symbolic framework for dynamic optimization.
#     Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
#                             K.U. Leuven. All rights reserved.
#     Copyright (C) 2011-2014 Greg Horn
#
#!/usr/bin/env python
"""
 * \file
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.Brito@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2017
 *
 * \brief
 *   Implementation of special data kraken classes to subscribe to several topics.
 *   In callback methods data is collected, put into a list representing rows.
 *   Then the rows can be written to a csv file.
 *
"""
from casadi import *
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState

tgrid=0
x1_opt=0
x2_opt=0
u_opt=0
ns='arm_left'

def mpc_step():
    T = 10. # Time horizon
    N = 20 # number of control intervals

    # Declare model variables
    x1 = MX.sym('x1')
    x2 = MX.sym('x2')
    x = vertcat(x1, x2)
    u = MX.sym('u')

    # Model equations
    xdot = vertcat((1-x2**2)*x1 - x2 + u, x1)

    # Objective term
    L = x1**2 + x2**2 + u**2

    # Formulate discrete time dynamics
    if False:
       # CVODES from the SUNDIALS suite
       dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
       opts = {'tf':T/N}
       F = integrator('F', 'cvodes', dae, opts)
    else:
       # Fixed step Runge-Kutta 4 integrator
       M = 4 # RK4 steps per interval
       DT = T/N/M
       f = Function('f', [x, u], [xdot, L])
       X0 = MX.sym('X0', 2)
       U = MX.sym('U')
       X = X0
       Q = 0
       for j in range(M):
           k1, k1_q = f(X, U)
           k2, k2_q = f(X + DT/2 * k1, U)
           k3, k3_q = f(X + DT/2 * k2, U)
           k4, k4_q = f(X + DT * k3, U)
           X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
           Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
       F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

    # Evaluate at a test point
    Fk = F(x0=[0.2,0.3],p=0.4)
    print(Fk['xf'])
    print(Fk['qf'])

    # Start with an empty NLP
    w=[]
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g=[]
    lbg = []
    ubg = []

    # "Lift" initial conditions
    X0 = MX.sym('X0', 2)
    w += [X0]
    lbw += [0, 1]
    ubw += [0, 1]
    w0 += [0, 1]

    # Formulate the NLP
    Xk = MX([0, 1])
    for k in range(N):
        # New NLP variable for the control
        Uk = MX.sym('U_' + str(k))
        w   += [Uk]
        lbw += [-1]
        ubw += [1]
        w0  += [0]

        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Uk)
        Xk_end = Fk['xf']
        J=J+Fk['qf']

        # New NLP variable for state at end of interval
        Xk = MX.sym('X_' + str(k+1), 2)
        w   += [Xk]
        lbw += [-0.25, -inf]
        ubw += [  inf,  inf]
        w0  += [0, 0]

        # Add equality constraint
        g   += [Xk_end-Xk]
        lbg += [0, 0]
        ubg += [0, 0]

    # Create an NLP solver
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob);

    # Solve the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()

    # Plot the solution
    x1_opt = w_opt[0::3]
    x2_opt = w_opt[1::3]
    u_opt = w_opt[2::3]

    tgrid = [T/N*k for k in range(N+1)]

def plot():
    plt.figure(1)
    plt.clf()
    plt.plot(tgrid, x1_opt, '--')
    plt.plot(tgrid, x2_opt, '-')
    plt.step(tgrid, vertcat(DM.nan(1), u_opt), '-.')
    plt.xlabel('t')
    plt.legend(['x1', 'x2', 'u'])
    plt.grid()
    plt.show()

def init():
    if rospy.has_param(ns+'/joint_names'):
        joint_names = rospy.get_param(ns+'/joint_names')
    else:
        rospy.logerr('Parameter joint_names not set');
        #rospy.logwarn('Could not find parameter ~base_dir. Using default base_dir: ' + base_dir)

    if rospy.has_param(ns+'/nmpc/shooting_nodes'):
        shooting_nodes = rospy.get_param(ns+'/nmpc/shooting_nodes')
        print(shooting_nodes)
    else:
        rospy.logerr('Parameter shooting_nodes not set')

    if rospy.has_param(ns+'/nmpc/time_horizon'):
        time_horizon = rospy.get_param(ns+'/nmpc/time_horizon')
    else:
        rospy.logerr('Parameter time_horizon not set')

    if rospy.has_param(ns+'/nmpc/state_dim'):
        state_dim = rospy.get_param(ns+'/nmpc/state_dim')
    else:
        rospy.logerr('Parameter state_dim not set')

    if rospy.has_param(ns+'/nmpc/control_dim'):
        control_dim = rospy.get_param(ns+'/nmpc/control_dim')
    else:
        rospy.logerr('Parameter control_dim not set')

    if rospy.has_param(ns+'base/base_active'):
        base_active = rospy.get_param(ns+'base/base_active')
    else:
        rospy.logerr('Parameter base/base_active not set')

    if rospy.has_param(ns+'/nmpc/state/path_constraints/min'):
        state_path_constraints_min = rospy.get_param(ns+'/nmpc/state/path_constraints/min')
    else:
        rospy.logerr('Parameter state/path_constraints/min not set')

    if rospy.has_param(ns+'/nmpc/state/path_constraints/max'):
        state_path_constraints_max = rospy.get_param(ns+'/nmpc/state/path_constraints/max')
    else:
        rospy.logerr('Parameter state/path_constraints/max not set')

    if rospy.has_param(ns+'/nmpc/state/terminal_constraints/min'):
        state_terminal_constraints_min = rospy.get_param(ns+'/nmpc/state/terminal_constraints/min')
    else:
        rospy.logerr('Parameter state/terminal_constraints/min not set')

    if rospy.has_param(ns+'/nmpc/state/terminal_constraints/max'):
        state_terminal_constraints_max = rospy.get_param(ns+'/nmpc/state/terminal_constraints/max')
    else:
        rospy.logerr('Parameter state/terminal_constraints/max not set')

    if rospy.has_param(ns+'/nmpc/input/input_constraints/min'):
        input_constraints_min = rospy.get_param(ns+'/nmpc/input/input_constraints/min')
    else:
        rospy.logerr('Parameter input/input_constraints/min not set')

    if rospy.has_param(ns+'/nmpc/input/input_constraints/max'):
        input_constraints_max = rospy.get_param(ns+'/nmpc/input/input_constraints/max')
    else:
        rospy.logerr('Parameter input/input_constraints/max not set')

    if rospy.has_param(ns+'/nmpc/input/input_constraints/min'):
        min_dist = rospy.get_param(ns+'/nmpc/input/input_constraints/min')
    else:
        rospy.logerr('Parameter input/input_constraints/min not set')

    if rospy.has_param(ns+'chain_tip_link'):
        chain_tip_link = rospy.get_param('chain_tip_link')
    else:
        rospy.logwarn('Could not find parameter chain_tip_link.')
        exit(-1)

    if rospy.has_param(ns+'chain_base_link'):
        chain_base_link = rospy.get_param('chain_base_link')
    else:
        rospy.logwarn('Could not find parameter chain_base_link.')
        exit(-1)

    if rospy.has_param(ns+'frame_tracker/tracking_frame'):
        tracking_frame = rospy.get_param('frame_tracker/tracking_frame')
    else:
        rospy.logwarn('Could not find parameter frame_tracker/tracking_frame.')
        exit(-2)

    jointstate_sub_ = rospy.Subscriber("joint_states",JointState,jointstateCallback)

def jointstateCallback():
    print('Hello')

if __name__ == "__main__":
    rospy.init_node("nmpc")
    print('hello')
    init()
    mpc_step()
    plot()
