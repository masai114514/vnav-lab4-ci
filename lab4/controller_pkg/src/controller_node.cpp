#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 0 |  16.485 - Fall 2019  - Lab 3 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab 3 Handout for more details).
//
//  The include below is strongly suggested [but not mandatory if you have
//  better alternatives in mind :)]. Eigen is a C++ library for linear algebra
//  that will help you significantly with the implementation. Check the
//  quick reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf2 provides useful functions to convert tf2
// messages to eigen types and vice versa.
#include <tf2_eigen/tf2_eigen.h>

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                                 end part 0
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class controllerNode{
  ros::NodeHandle nh;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // In this section, you need to declare:
  //   1. two subscribers (for the desired and current UAVStates)
  //   2. one publisher (for the propeller speeds)
  //   3. a timer for your main control loop
  //
  // ~~~~ begin solution
  ros::Subscriber desired_state_sub;   // "desired_state"  (trajectory_msgs/MultiDOFJointTrajectoryPoint)
  ros::Subscriber current_state_sub;   // "current_state"  (nav_msgs/Odometry)
  ros::Publisher  rotor_speed_pub;     // "rotor_speed_cmds" (mav_msgs/Actuators)
  ros::Timer      control_timer;       // drives controlLoop() at hz
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  // Whether the first desired state has arrived. Before that the controller
  // stays silent (see onDesiredState / controlLoop). Mirrors the offline
  // plant's `hold_until_desired` convention: the vehicle is not commanded
  // until the race actually starts, so it does not fight a stale xd = 0.
  bool have_desired = false;


  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 2 |  Initialize ROS callback handlers
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // In this section, you need to initialize your handlers from part 1.
      // Specifically:
      //  - bind controllerNode::onDesiredState() to the topic "desired_state"
      //  - bind controllerNode::onCurrentState() to the topic "current_state"
      //  - bind controllerNode::controlLoop() to the created timer, at frequency
      //    given by the "hz" variable
      //
      // Hints:
      //  - use the nh variable already available as a class member
      //  - read the lab 3 handout to fnd the message type
      //
      // ~~~~ begin solution
      desired_state_sub = nh.subscribe("desired_state", 1,
                                       &controllerNode::onDesiredState, this);
      current_state_sub = nh.subscribe("current_state", 1,
                                       &controllerNode::onCurrentState, this);
      rotor_speed_pub   = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);

      // Control-loop rate. Defaults to the official 1000 Hz (the value baked
      // into the lab3 starter). It is exposed as a parameter because the
      // Unity/TESSE real-time loop steps the plant at ~50 Hz and relays every
      // command over a socket bridge, so on a CPU-starved machine 1000 Hz only
      // burns cycles that the renderer needs. Override with /hz.
      // NOTE: must be read *before* the timer is created below.
      nh.param("hz", hz, 1000.0);
      ROS_INFO("Control loop rate: %f Hz", hz);

      control_timer     = nh.createTimer(ros::Duration(1.0 / hz),
                                         &controllerNode::controlLoop, this);
      // ~~~~ end solution

      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 2
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 6 [NOTE: save this for last] |  Tune your gains!
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // Live the life of a control engineer! Tune these parameters for a fast
      // and accurate controller.
      //
      // Controller gains
      //
      //
      // **** FIDDLE WITH THESE! ***
      // Change them in the provided launch file.
      nh.getParam("kx", kx);
      nh.getParam("kv", kv);
      nh.getParam("kr", kr);
      nh.getParam("komega", komega);
      ROS_INFO("Gain values:\nkx: %f \nkv: %f \nkr: %f \nkomega: %f\n", kx, kv, kr, komega);

      //
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 6
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

      // Initialize constants
      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Build the wrench -> squared-rotor-speed allocation matrix F2W.
      //
      // Convention (Lab 3 handout / lecture notes eq. 6.8-6.9):
      //   * body x-axis (x_B) is at 45 deg between two propeller arms;
      //   * rotor i sits at rho_i = (d/sqrt(2)) * (cos(45+90(i-1)), sin(...), 0)
      //       rho_1 = +d/sqrt2 * ( 1, 1, 0)     rho_2 = +d/sqrt2 * (-1, 1, 0)
      //       rho_3 = +d/sqrt2 * (-1,-1, 0)     rho_4 = +d/sqrt2 * ( 1,-1, 0)
      //   * thrust along +e3, magnitude c_f * w_i|w_i|;
      //   * drag torque about e3, sign (-1)^{i+1} c_d * w_i|w_i|
      //       (tau_drag_1 > 0, tau_drag_2 < 0, tau_drag_3 > 0, tau_drag_4 < 0)
      //
      // The wrench is F_w = F * w, with w = [w_1|w_1|, ..., w_4|w_4|]^T and
      //   rows of F  =  [ f_z , tau_x , tau_y , tau_z ].
      // F2W = F^{-1} maps the desired wrench back to signed squares of rotor
      // speeds, so that w_i = signed_sqrt( (F2W * [f; tau])_i ).
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      const double s = 1.0 / std::sqrt(2.0);   // cos(45) = sin(45)
      Eigen::Matrix4d F;
      F <<  cf,          cf,          cf,          cf,
            cf*d*s,      cf*d*s,     -cf*d*s,     -cf*d*s,
           -cf*d*s,      cf*d*s,      cf*d*s,     -cf*d*s,
            cd,         -cd,          cd,         -cd;
      F2W = F.inverse();
      ROS_INFO_STREAM("F (rotor^2 -> wrench) =\n" << F);
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 3 | Objective: fill in xd, vd, ad, yawd
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // 3.1 Get the desired position, velocity and acceleration from the in-
      //     coming ROS message and fill in the class member variables xd, vd
      //     and ad accordingly. You can ignore the angular acceleration.
      //
      // Hint: use "v << vx, vy, vz;" to fill in a vector with Eigen.
      //
      // ~~~~ begin solution
      xd << des_state.transforms[0].translation.x,
            des_state.transforms[0].translation.y,
            des_state.transforms[0].translation.z;

      vd << des_state.velocities[0].linear.x,
            des_state.velocities[0].linear.y,
            des_state.velocities[0].linear.z;

      ad << des_state.accelerations[0].linear.x,
            des_state.accelerations[0].linear.y,
            des_state.accelerations[0].linear.z;
      // ~~~~ end solution
      //
      // 3.2 Extract the yaw component from the quaternion in the incoming ROS
      //     message and store in the yawd class member variable
      //
      //  Hints:
      //    - use the methods tf2::getYaw(...)
      //    - maybe you want to use also tf2::fromMsg(...)
      //
      // ~~~~ begin solution
      tf2::Quaternion q_des;
      tf2::fromMsg(des_state.transforms[0].rotation, q_des);
      yawd = tf2::getYaw(q_des);
      // ~~~~ end solution
      //
      // First desired state => the race has started, the controller may take
      // over. Before this the vehicle is left uncommanded (see controlLoop).
      if (!have_desired) {
        have_desired = true;
        ROS_INFO("First desired state received - controller engaged.");
      }
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 3
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 4 | Objective: fill in x, v, R and omega
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // Get the current position and velocity from the incoming ROS message and
      // fill in the class member variables x, v, R and omega accordingly.
      //
      //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
      //          needs to be in the body frame!
      //
      // ~~~~ begin solution
      x << cur_state.pose.pose.position.x,
           cur_state.pose.pose.position.y,
           cur_state.pose.pose.position.z;

      v << cur_state.twist.twist.linear.x,
           cur_state.twist.twist.linear.y,
           cur_state.twist.twist.linear.z;

      // rotation: quaternion -> rotation matrix (body -> world)
      tf2::Quaternion q_cur;
      tf2::fromMsg(cur_state.pose.pose.orientation, q_cur);
      tf2::Matrix3x3 R_tf(q_cur);
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          R(i, j) = R_tf[i][j];

      // angular velocity: world frame -> body frame (omega = R^T * omega_world)
      Eigen::Vector3d omega_world;
      omega_world << cur_state.twist.twist.angular.x,
                     cur_state.twist.twist.angular.y,
                     cur_state.twist.twist.angular.z;
      omega = R.transpose() * omega_world;
      // ~~~~ end solution
      //
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 4
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;

    // Stay silent until the race actually starts, i.e. until the trajectory
    // generator has produced its first desired state. Commanding xd = 0 in the
    // meantime would drive the vehicle towards the world origin (and through
    // the floor) instead of simply waiting at the spawn point.
    if (!have_desired) return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 5 | Objective: Implement the controller!
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // 5.1 Compute position and velocity errors. Objective: fill in ex, ev.
    //  Hint: [1], eq. (6), (7)
    //
    // ~~~~ begin solution
    ex = x - xd;
    ev = v - vd;
    // ~~~~ end solution

    // 5.2 Compute the Rd matrix.
    //
    //  Hint: break it down in 3 parts:
    //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    //    - check out [1] fig. (3) for the remaining axes [use cross product]
    //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    //
    //  CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term and
    //        ii) the overall sign (in front of the fraction) in equation (12)
    //            of the paper
    //    - remember to normalize your axes!
    //
    // ~~~~ begin solution
    // Desired thrust direction: A "points" along the body z-axis. In ROS the
    // world z-axis points UP, hence gravity enters with a POSITIVE +g*e3 term
    // (sign flip w.r.t. [1] where z points down).
    Eigen::Vector3d A = ad - kx*ex - kv*ev + g*e3;
    Eigen::Vector3d b3d = A.normalized();

    // auxiliary heading vector (same yaw as desired, no roll/pitch)
    Eigen::Vector3d x_cd(std::cos(yawd), std::sin(yawd), 0.0);

    // remaining axes via cross products (see lecture notes eq. 6.29-6.30)
    Eigen::Vector3d b2d = b3d.cross(x_cd).normalized();
    Eigen::Vector3d b1d = b2d.cross(b3d);

    Eigen::Matrix3d Rd;
    Rd << b1d, b2d, b3d;
    // ~~~~ end solution
    //
    // 5.3 Compute the orientation error (er) and the rotation-rate error (eomega)
    //  Hints:
    //     - [1] eq. (10) and (11)
    //     - you can use the Vee() static method implemented above
    //
    //  CAVEAT: feel free to ignore the second addend in eq (11), since it
    //          requires numerical differentiation of Rd and it has negligible
    //          effects on the closed-loop dynamics.
    //
    // ~~~~ begin solution
    er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);
    eomega = omega;   // ignoring the Omega_d term of [1] eq. (11)
    // ~~~~ end solution
    //
    // 5.4 Compute the desired wrench (force + torques) to control the UAV.
    //  Hints:
    //     - [1] eq. (15), (16)

    // CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term
    //        ii) the overall sign (in front of the bracket) in equation (15)
    //            of the paper
    //
    //    - feel free to ignore all the terms involving \Omega_d and its time
    //      derivative as they are of the second order and have negligible
    //      effects on the closed-loop dynamics.
    //
    // ~~~~ begin solution
    // Total thrust magnitude: projection of the desired force onto the ACTUAL
    // body z-axis (b3 = R * e3). F_des = m * A, so  f = F_des . (R e3).
    double f = m * A.dot(R * e3);

    // Moment (body frame): [1] eq. (16), dropping the Omega_d terms
    Eigen::Vector3d M = -kr*er - komega*eomega + omega.cross(J * omega);
    // ~~~~ end solution

    // 5.5 Recover the rotor speeds from the wrench computed above
    //
    //  Hints:
    //     - [1] eq. (1)
    //
    // CAVEATs:
    //     - we have different conventions for the arodynamic coefficients,
    //       Namely: C_{\tau f} = c_d / c_f
    //               (LHS paper [1], RHS our conventions [lecture notes])
    //
    //     - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    //       between b1 and b2). To resolve this, check out equation 6.9 in the
    //       lecture notes!
    //
    //     - The thrust forces are **in absolute value** proportional to the
    //       square of the propeller speeds. Negative propeller speeds - although
    //       uncommon - should be a possible outcome of the controller when
    //       appropriate. Note that this is the case in unity but not in real
    //       life, where propellers are aerodynamically optimized to spin in one
    //       direction!
    //
    // ~~~~ begin solution
    Eigen::Vector4d wrench;
    wrench << f, M(0), M(1), M(2);

    // signed squares of the rotor speeds
    Eigen::Vector4d w_sq = F2W * wrench;
    // ~~~~ end solution
    //
    // 5.6 Populate and publish the control message
    //
    // Hint: do not forget that the propeller speeds are signed (maybe you want
    // to use signed_sqrt function).
    //
    // ~~~~ begin solution
    mav_msgs::Actuators cmd;
    cmd.angular_velocities.resize(4);
    for (int i = 0; i < 4; ++i)
      cmd.angular_velocities[i] = signed_sqrt(w_sq(i));
    rotor_speed_pub.publish(cmd);
    // ~~~~ end solution
    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //           end part 5, congrats! Start tuning your gains (part 6)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}
