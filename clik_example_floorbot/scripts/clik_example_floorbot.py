#!/usr/bin/env python
"""This is an example of the CASCLIK module being used for closed-loop inverse kinematics of the Thrivaldi robot."""
import rospy

# CasADi and CASCLIK things
import casclik as cc
import casadi as cs
import casclik_basics.robot_interface as robintrfc
from urdf2casadi import converter as urdfconv

# To wait for the robot
from sensor_msgs.msg import JointState

# For visualization
from py_viz_marker.srv import AddMarker
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

if __name__ == "__main__":
    rospy.init_node("clik_example_floorbot")

    # This example uses CasADi and CASCLIK extensively.
    # For more information, look through the examples on the CASCLIK webpage
    # https://github.com/mahaarbo/casclik
    # Or the ROS examples
    # https://github.com/mahaarbo/casclik_examples

    ####################################################################
    # Forward kinematics and core symbols
    ####################################################################
    # Get the forward kinematics
    fk_dict = urdfconv.from_parameter_server(
        root="root",
        tip="floor_tool0",
        key="/robot_description")
    rospy.loginfo("--Got size(q):" + str(len(fk_dict["joint_names"])))

    # gantry forward kinematics
    fk_dict_g = urdfconv.from_parameter_server(
        root="root",
        tip="gantry_tool0",
        key="/robot_description"
    )

    # Floor  link 1 forward kinematics
    fk_dict_floor_link_1 = urdfconv.from_parameter_server(
        root="root",
        tip="floor_link_1",
        key="/robot_description"
    )

    # Setup time and robot_var
    t = cs.MX.sym("t")
    q = cs.MX.sym("q", len(fk_dict["joint_names"]))
    q_g = cs.MX.sym("q_g", len(fk_dict_g["joint_names"]))

    # Functions for end-effector things (casadi functions of q)
    T_fk = fk_dict["T_fk"]
    p_fk = cs.Function("p_fk", [t, q], [T_fk(q)[:3, 3]])
    R_fk = cs.Function("R_fk", [t, q], [T_fk(q)[:3, :3]])

    # Gantry forward kineamtics
    T_fk_g = fk_dict_g["T_fk"]
    p_fk_g = T_fk_g(q_g)[:3, 3]
    R_fk_g = T_fk_g(q_g)[:3, 3]

    # Floor link 1 forward kinematics
    fl1len = len(fk_dict_floor_link_1["joint_names"])
    T_fl1 = fk_dict_floor_link_1["T_fk"](q[:fl1len])
    p_fl1 = T_fl1[:3, 3]
    R_fl1 = T_fl1[:3, :3]
    Q_r_fl1 = fk_dict_floor_link_1["dual_quaternion_fk"](q[:fl1len])[:4]

    ####################################################################
    # Initialize robot
    ####################################################################
    # Wait for robot to start
    rospy.loginfo("Waiting for /joint_states to start")
    rospy.wait_for_message(
        "/joint_states",
        JointState,
        timeout=None)

    # Prep position controller
    rospy.loginfo("Waiting for controller switch.")
    resp = robintrfc.switch_hw_controller(  # sane switching of controllers
        "joint_position_controller",
        resources=fk_dict["joint_names"],
        namespace="/floor")

    if resp.ok == 1:
        rospy.loginfo("Controller switched!")
    else:
        rospy.logerr("Controller not switcheed!")
        rospy.logerr("Cannot run without joint_position_controller.")
        quit()

    ####################################################################
    # Setup the problem expressions
    ####################################################################
    # Let's make a 3D Lissajous curve to track
    n_x = 5
    n_y = 2
    n_z = 3
    omega = 0.1
    scale_x = 0.2
    scale_y = 0.2
    scale_z = 0.2
    offset_x = -1.4
    offset_y = 0.7
    offset_z = 0.7
    traj_des = cs.vertcat(scale_x*cs.sin(omega*t*n_x) + offset_x,
                          scale_y*cs.sin(omega*t*n_y + 2*cs.np.pi) + offset_y,
                          scale_z*cs.sin(omega*t*n_z + 2*cs.np.pi) + offset_z)

    # And let's make an expression for how well we're pointing down
    POV_des = cs.vertcat(0., 0., -1.)
    POV_rob = R_fk(t, q)[:3, 2]  # Looks in Z direction
    fov_max = 2*cs.np.sin(cs.np.pi*10./(2*180))

    # Let's make an expression for gantry EE to stay out of an ellipse
    # that surrounds the floor bot
    d_end = p_fk_g - p_fl1
    ell_height = 1.206  # m
    ell_depth = 1.911  # m
    ell_width = 1.611  # m
    P_ellipse = cs.diag([1/(ell_depth**2), 1/(ell_width**2), 1/(ell_height**2)])
    RD = cs.mtimes(R_fl1, d_end)
    ellipse_end = cs.mtimes(RD.T, cs.mtimes(P_ellipse, RD))

    ####################################################################
    # Setup constraints and skill
    ####################################################################
    # Tracking the trajectory
    traj_cnstr = cc.EqualityConstraint(
        label="track_traj",
        expression=p_fk(t, q) - traj_des,
        constraint_type="soft",
        gain=0.1
    )

    # POV constraint, keep looking somewhat in X direction
    fov_cnstr = cc.SetConstraint(
        label="fov",
        expression=cs.dot(POV_rob-POV_des, POV_rob-POV_des),
        set_max=fov_max**2,
        set_min=0.0,
        gain=10.0,
        constraint_type="soft"
    )

    # Joint constraints
    joint_lim_cnstr = cc.SetConstraint(
        label="joint_limits",
        expression=q,
        set_max=cs.np.array(fk_dict["upper"]),
        set_min=cs.np.array(fk_dict["lower"]),
        constraint_type="hard"
    )

    # Joint rate constraints
    deg2rad = cs.np.pi/180.0
    joint_rate_cnstr = cc.VelocitySetConstraint(
        label="joint_rate_limits",
        expression=q,
        set_max=0.75*deg2rad*cs.np.array([
            156,
            156,
            156,
            343,
            659
        ]),
        constraint_type="hard",
    )
    # Formulate skill
    constraints = []
    constraints += [traj_cnstr]
    constraints += [fov_cnstr]
    skill = cc.SkillSpecification(
        label="track_traj_skill",
        time_var=t,
        robot_var=q,
        constraints=constraints
    )
    skill.print_constraints()

    ####################################################################
    # Setup robot interfaces
    ####################################################################
    cntrllr_class = cc.ReactiveQPController
    timestep = 1.0/80
    casclik_joint_names = fk_dict["joint_names"]

    robot_interface = robintrfc.DefaultRobotInterface(
        skill,
        timestep=timestep,
        namespace="floor",
        cntrllr_class=cntrllr_class,
        casclik_joint_names=casclik_joint_names,
        max_robot_vel_var=[1.5]*len(casclik_joint_names),
        min_robot_vel_var=[-1.5]*len(casclik_joint_names)
    )

    ####################################################################
    # Setup Visualizaton
    ####################################################################
    rospy.loginfo("Checking for visualizer")
    try:
        rospy.wait_for_service("/add_marker", timeout=1)
        viz_available = True
        add_marker = rospy.ServiceProxy("/add_marker",
                                        AddMarker)
        rospy.loginfo("--Visualizer found!")
    except rospy.ROSException:
        rospy.loginfo("If you run vizhandler we can visualize the things.")
        viz_available = False
    if viz_available:
        line_mrkr = Marker()
        line_mrkr.type = Marker.LINE_STRIP
        line_mrkr.scale.x = 0.01
        line_mrkr.action = Marker.ADD
        line_mrkr.color.r = 1.0
        line_mrkr.color.a = 1.0
        line_mrkr.header.frame_id = "root"
        ftraj_des = cs.Function("ftraj_des", [t], [traj_des])
        points = []
        npoints = 100
        for i in range(npoints):
            des_p = ftraj_des(2*cs.pi*i/((npoints-1)*omega)).toarray()[:, 0]
            points += [Point(des_p[0], des_p[1], des_p[2])]
        line_mrkr.points = points
        add_marker(label="desired_traj",
                   marker=line_mrkr)

    ####################################################################
    # Start moving to the points
    ####################################################################
    rospy.loginfo("Starting robot_interface in 1 second!")
    rospy.sleep(1)
    robot_interface.start()
    try:
        while not rospy.is_shutdown() and robot_interface.running:
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        quit()
    rospy.loginfo("Stopped robot interface because: "
                  + robot_interface.stop_reason)
    robot_interface.disconnect()
