#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <iostream>
#include <sstream>

#include <math.h>

ros::Subscriber desire_traj_vertices_sub;
ros::Publisher desired_state_pub;
tf::TransformBroadcaster* br;

/**
 * Callback function for listening to the desired vertices msgs
 */
void trajCB(const geometry_msgs::PoseArray& traj_msg) {
  // sanity check for traj_msg size
  if (traj_msg.poses.size() == 0) {
    ROS_ERROR_THROTTLE(1, "Empty trajectory vertices msg.");
    return;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 0 |  16.485 - Fall 2020  - Lab 4 coding assignment  (10 pts)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  //  As a simple warm up exercise before we get to the actual 'real deal',
  //  let's just make our quadcopter fly to the first gate in the course.
  //  In this section:
  //   1. Extract the first vertex of the trajectory
  //   2. Set the acceleration and velocities to zero
  //   3. Publish the desired MultiDOFJointTrajectoryPoint
  //   4. Create and publish TF transform of the desired pose
  // ~~~~ begin solution

  // ---------------------------------------------------------------------------
  // 1. Extract the first vertex (gate 0) of the trajectory.
  // ---------------------------------------------------------------------------
  const geometry_msgs::Pose& first_pose = traj_msg.poses.front();

  // ---------------------------------------------------------------------------
  // 2. This is a fly-to-and-hover command, so the desired velocity and the
  //    desired acceleration are both zero. (geometry_msgs::Twist is
  //    zero-initialized by its default constructor.)
  // ---------------------------------------------------------------------------
  geometry_msgs::Twist zero_vel;
  geometry_msgs::Twist zero_acc;

  // ---------------------------------------------------------------------------
  // 3. Publish the desired MultiDOFJointTrajectoryPoint for the controller.
  //    The controller (controller_pkg) reads:
  //       transforms[0].translation / .rotation
  //       velocities[0].linear
  //       accelerations[0].linear
  // ---------------------------------------------------------------------------
  trajectory_msgs::MultiDOFJointTrajectoryPoint desired_point;
  desired_point.transforms.resize(1);
  desired_point.velocities.resize(1);
  desired_point.accelerations.resize(1);

  desired_point.transforms[0].translation.x = first_pose.position.x;
  desired_point.transforms[0].translation.y = first_pose.position.y;
  desired_point.transforms[0].translation.z = first_pose.position.z;
  desired_point.transforms[0].rotation = first_pose.orientation;
  desired_point.velocities[0] = zero_vel;
  desired_point.accelerations[0] = zero_acc;
  desired_point.time_from_start = ros::Duration(0.0);  // static hover point

  desired_state_pub.publish(desired_point);

  // ---------------------------------------------------------------------------
  // 4. Create and publish the TF transform of the desired pose
  //    (world -> desired_position), so it can be visualized in rviz.
  // ---------------------------------------------------------------------------
  tf::Transform desired_tf;
  desired_tf.setOrigin(tf::Vector3(first_pose.position.x,
                                   first_pose.position.y,
                                   first_pose.position.z));
  desired_tf.setRotation(tf::Quaternion(first_pose.orientation.x,
                                        first_pose.orientation.y,
                                        first_pose.orientation.z,
                                        first_pose.orientation.w));
  br->sendTransform(tf::StampedTransform(desired_tf, ros::Time::now(),
                                         "world", "desired_position"));

  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 0
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_planner");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);
  ros::Time start(ros::Time::now());

  // deired traj vertices subscriber
  desire_traj_vertices_sub = n.subscribe("desired_traj_vertices", 10, trajCB);

  // publisher for desired states for controller
  desired_state_pub =
      n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "desired_state", 1);

  br = new tf::TransformBroadcaster();

  ros::spin();
}
