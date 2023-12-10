/*********************************************************************
 *
 * Wuhan University of Technology
 * 
 * School of Mechanical and Electrical Engineering
 * Author: Liu Rui
 * 

*********************************************************************/

#ifndef EDGE_JERK_H
#define EDGE_JERK_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <geometry_msgs/Twist.h>

#include <iostream>

namespace teb_local_planner
{

  
/**
 * @class EdgeJerk
 * @brief Edge defining the cost function for limiting the translational and rotational jerk.
 * 
 * The edge depends on seven vertices \f$ \mathbf{s}_i, \mathbf{s}_{i+1}, \mathbf{s}_{i+2}, \mathbf{s}_{i+3}, 
 * \Delta T_i, \Delta T_{i+1}, \Delta T_{i+2} \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j, omegadotdot]^T ) \cdot weight \f$. \n
 * \e j is calculated using the difference quotient （thrice） and the position parts of four poses. \n
 * \e omegadotdot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational jerk and
 * the second one the rotational jerk
 * @see TebOptimalPlanner::AddEdgesJerk
 * @remarks Do not forget to call setTebConfig()
*/
class EdgeJerk : public BaseTebMultiEdge<2, double>
{
public:
  
  /**
   * @brief Construct edge
  */
  EdgeJerk()
  {
    this->resize(7);
  }

  /**
   * @brief Actual cost function
  */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on Edgejerk()");
    // Get four poses and three deltaT
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[5]);
    const VertexTimeDiff* dt3 = static_cast<const VertexTimeDiff*>(_vertices[6]);

    // Using the difference quotient to calculate difference quotient
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    const Eigen::Vector2d diff3 = pose4->position() - pose3->position();

    // Calculate the distance difference between poses
    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    double dist3 = diff3.norm();

    // Calculate the angle difference between poses
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
    const double angle_diff3 = g2o::normalize_theta(pose4->theta() - pose3->theta());

    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius = dist1 / (2 * sin(angle_diff1 / 2));
            dist1 = fabs(angle_diff1 * radius); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius = dist2 / (2 * sin(angle_diff2 / 2));
            dist2 = fabs(angle_diff2 * radius); // actual arg length!
        }
        if (angle_diff3 != 0)
        {
            const double radius = dist3 / (2 * sin(angle_diff3 / 2));
            dist3 = fabs(angle_diff3 * radius); // actual arg length!
        }
    }

    // LINEAR
    // Calculate the velocity
    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();
    double vel3 = dist3 / dt3->dt();

    // consider directions
    vel1 *= fast_sigmoid(100 * (diff1.x() * cos(pose1->theta()) + diff1.y() * sin(pose1->theta())));
    vel2 *= fast_sigmoid(100 * (diff2.x() * cos(pose2->theta()) + diff2.y() * sin(pose2->theta())));
    vel3 *= fast_sigmoid(100 * (diff3.x() * cos(pose3->theta()) + diff3.y() * sin(pose3->theta())));

    // Calculate the acceleration
    double acc_lin1 = (vel2 - vel1) * 2 / (dt1->dt() + dt2->dt());
    double acc_lin2 = (vel3 - vel2) * 2 / (dt2->dt() + dt3->dt());

    // Calculate and add the jerk
    const double jerk_lin = (acc_lin2 - acc_lin1) / ((dt1->dt() + dt2->dt()) / 4 + (dt2->dt() + dt3->dt()) / 4);

    _error[0] = penaltyBoundToInterval(jerk_lin, cfg_->robot.jerk_lim_x, cfg_->optim.penalty_epsilon);

    // ANGULAR
    // Calculate the omega
    double omega1 = angle_diff1 / dt1->dt();
    double omega2 = angle_diff2 / dt2->dt();
    double omega3 = angle_diff3 / dt3->dt();

    // Calculate the acceleration
    double acc_rot1 = (omega2 - omega1) * 2 / (dt1->dt() + dt2->dt());
    double acc_rot2 = (omega3 - omega2) * 2 / (dt2->dt() + dt3->dt());

    // Calculate and add the jerk
    const double jerk_rot = (acc_rot2 - acc_rot1) / ((dt1->dt() + dt2->dt()) / 4 + (dt2->dt() + dt3->dt()) / 4);

    _error[1] = penaltyBoundToInterval(jerk_rot, cfg_->robot.jerk_lim_theta, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n", _error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() rotational: _error[1]=%f\n", _error[1]);

    // 
  }

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};
    
    
/**
 * @class EdgeJerkStart
 * @brief Edge difining the cost function for limiting the translational and rotational jerk at the beginning of the trajectory.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i \f$, \Delta T_{i+1} \f$ 
 * an initial acceleration defined by setInitialAcceleration() and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j, omegadotdot ]^T ) \cdot weight \f$. \n
 * \e j is calculated using the difference quotient (thrice) and the position parts of the poses. \n
 * \e omegadotdot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational jerk and
 * the second one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerk
 * @see EdgeJerkGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationGoal() for defining boundary values at the end of the trajectory!
*/
class EdgeJerkStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Constuct edge.
  */
  EdgeJerkStart()
  {
    _measurement = NULL;
    this->resize(5);
  }
    
  /**
   * @brief Actual cost function
  */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkStart()");

    // Obtain pose and time difference information from vertices
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // Calculate the distance difference and angle difference between poses
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius =  dist1/(2*sin(angle_diff1/2));
            dist1 = fabs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius =  dist2/(2*sin(angle_diff2/2));
            dist2 = fabs( angle_diff2 * radius ); // actual arg length!
        }
    }
    
    // LINEAR
    // Calculate the velocity
    const double vel0 = _measurement->linear.x;
    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();

    // consider directions
    vel1 *= fast_sigmoid(100 * (diff1.x() * cos(pose1->theta()) + diff1.y() * sin(pose1->theta())));
    vel2 *= fast_sigmoid(100 * (diff2.x() * cos(pose2->theta()) + diff2.y() * sin(pose2->theta())));

    // Calculate the acceleration
    double acc_lin0 = (vel1 - vel0) / dt1->dt();
    double acc_lin1 = (vel2 - vel1) * 2 / (dt1->dt() + dt2->dt());

    // Calculate and add the jerk
    const double jerk_lin = (acc_lin1 - acc_lin0) / (dt1->dt() / 2 + (dt1->dt() + dt2->dt()) / 4);

    _error[0] = penaltyBoundToInterval(jerk_lin, cfg_->robot.jerk_lim_x, cfg_->optim.penalty_epsilon);

    // ANGULAR
    // Calculate the omega
    const double omega0 = _measurement->angular.z;
    double omega1 = angle_diff1 / dt1->dt();
    double omega2 = angle_diff2 / dt2->dt();

    // Calculate the acceleration
    double acc_rot0 = (omega1 - omega0) / dt1->dt();
    double acc_rot1 = (omega2 - omega1) * 2 / (dt1->dt() + dt2->dt());

    // Calculate and add the jerk
    const double jerk_rot = (acc_rot1 - acc_rot0) / (dt1->dt() / 2 + (dt1->dt() + dt2->dt()) / 4);

    _error[1] = penaltyBoundToInterval(jerk_rot, cfg_->robot.jerk_lim_theta, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerkStart::computeError() translational: _error[0]=%f\n", _error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerkStart::computeError() rotational: _error[1]=%f\n", _error[1]);
  }

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * and use this speed to initialize the initial acceleration for the next step
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
    
    
    

/**
 * @class EdgeJerkGoal
 * @brief Edge difining the cost function for limiting the translational and rotational jerk at the end of the trajectory.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i \f$, \Delta T_{i+1} \f$ 
 * an initial acceleration defined by setGoalAcceleration() and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j, omegadotdot ]^T ) \cdot weight \f$. \n
 * \e j is calculated using the difference quotient (thrice) and the position parts of the poses. \n
 * \e omegadotdot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational jerk and
 * the second one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerk
 * @see EdgeJerkGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationGoal() for defining boundary values at the end of the trajectory!
*/
class EdgeJerkGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Constuct edge.
  */
  EdgeJerkGoal()
  {
    _measurement = NULL;
    this->resize(5);
  }
    
  /**
   * @brief Actual cost function
  */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeJerkStart()");

    // Obtain pose and time difference information from vertices
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // Calculate the distance difference and angle difference between poses
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose_goal->position() - pose2->position();
    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose_goal->theta() - pose2->theta());

    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius =  dist1/(2*sin(angle_diff1/2));
            dist1 = fabs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius =  dist2/(2*sin(angle_diff2/2));
            dist2 = fabs( angle_diff2 * radius ); // actual arg length!
        }
    }
    
    // LINEAR
    // Calculate the velocity
    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();
    const double vel3 = _measurement->linear.x;

    // consider directions
    vel1 *= fast_sigmoid(100 * (diff1.x() * cos(pose1->theta()) + diff1.y() * sin(pose1->theta())));
    vel2 *= fast_sigmoid(100 * (diff2.x() * cos(pose2->theta()) + diff2.y() * sin(pose2->theta())));

    // Calculate the acceleration
    double acc_lin1 = (vel2 - vel1) * 2 / (dt1->dt() + dt2->dt());
    double acc_lin2 = (vel3 - vel2) / dt2->dt();

    // Calculate and add the jerk
    const double jerk_lin = (acc_lin2 - acc_lin1) / ((dt1->dt() + dt2->dt()) / 4 + dt2->dt() / 2);

    _error[0] = penaltyBoundToInterval(jerk_lin, cfg_->robot.jerk_lim_x, cfg_->optim.penalty_epsilon);

    // ANGULAR
    // Calculate the omega
    double omega1 = angle_diff1 / dt1->dt();
    double omega2 = angle_diff2 / dt2->dt();
    const double omega3 = _measurement->angular.z;

    // Calculate the acceleration
    double acc_rot1 = (omega2 - omega1) * 2 / (dt1->dt() + dt2->dt());
    double acc_rot2 = (omega3 - omega2) / dt2->dt();

    // Calculate and add the jerk
    const double jerk_rot = (acc_rot2 - acc_rot1) / ((dt1->dt() + dt2->dt()) / 4 + dt2->dt() / 2);

    _error[1] = penaltyBoundToInterval(jerk_rot, cfg_->robot.jerk_lim_theta, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerkGoal::computeError() translational: _error[0]=%f\n", _error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerkGoal::computeError() rotational: _error[1]=%f\n", _error[1]);
  }

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * and use this speed to initialize the goal / final acceleration for the next step
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setGoalVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
    
    

/**
 * @class EdgeJerkHolonomic
 * @brief Edge defining the cost function for limiting the translational and rotational jerk.
 * 
 * The edge depends on seven vertices \f$ \mathbf{s}_i, \mathbf{s}_{i+1}, \mathbf{s}_{i+2}, \mathbf{s}_{i+3}, 
 * \Delta T_i, \Delta T_{i+1}, \Delta T_{i+2} \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [jx, jy, omegadotdot]^T ) \cdot weight \f$. \n
 * \e jx is calculated using the difference quotient （thrice） and the position parts of four poses. \n
 * \e jy is calculated using the difference quotient （thrice） and the position parts of four poses. \n
 * \e omegadotdot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: the first component represents the translational jerk (x-dir), 
 * the second one the strafing jerk and the third one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomicStart
 * @see EdgeJerkHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomicStart() and EdgeJerkHolonomicGoal() for defining boundary values!
*/
class EdgeJerkHolonomic : public BaseTebMultiEdge<3, double>
{
public:
  
  /**
   * @brief Construct edge
  */
  EdgeJerkHolonomic()
  {
    this->resize(7);
  }

  /**
   * @brief Actual cost function
  */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on Edgejerk()");
    // Get four poses and three deltaT
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[5]);
    const VertexTimeDiff* dt3 = static_cast<const VertexTimeDiff*>(_vertices[6]);

    // Using the difference quotient to calculate difference quotient
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    const Eigen::Vector2d diff3 = pose4->position() - pose3->position();

    // Calculate the angle difference between poses
    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta());
    double cos_theta2 = std::cos(pose2->theta());
    double sin_theta2 = std::sin(pose2->theta());
    double cos_theta3 = std::cos(pose3->theta());
    double sin_theta3 = std::sin(pose3->theta());

    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();
    // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
    double p2_dx =  cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();
    // transform pose4 into robot frame pose3 (inverse 2d rotation matrix)
    double p3_dx =  cos_theta3 * diff3.x() + sin_theta3 * diff3.y();
    double p3_dy = -sin_theta3 * diff3.x() + cos_theta3 * diff3.y();

    // LINEAR
    // Calculate the velocity
    double vel1_x = p1_dx / dt1->dt();
    double vel1_y = p1_dy / dt1->dt();
    double vel2_x = p2_dx / dt2->dt();
    double vel2_y = p2_dy / dt2->dt();
    double vel3_x = p3_dx / dt3->dt();
    double vel3_y = p3_dy / dt3->dt();

    // Calculate total duration
    double dt12 = dt1->dt() + dt2->dt();
    double dt23 = dt2->dt() + dt3->dt();

    // Calculate the acceleration
    double acc_x1 = (vel2_x - vel1_x) * 2 / dt12;
    double acc_y1 = (vel2_y - vel1_y) * 2 / dt12;
    double acc_x2 = (vel3_x - vel2_x) * 2 / dt23;
    double acc_y2 = (vel3_y - vel2_y) * 2 / dt23;

    // Calculate and add the jerk
    const double jerk_x = (acc_x2-acc_x1) / (dt12 / 4 + dt23 / 4);
    const double jerk_y = (acc_y2-acc_y1) / (dt12 / 4 + dt23 / 4);

    _error[0] = penaltyBoundToInterval(jerk_x, cfg_->robot.jerk_lim_x, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(jerk_y, cfg_->robot.jerk_lim_y, cfg_->optim.penalty_epsilon);

    // ANGULAR
    // Calculate the angle difference between poses
    double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
    double angle_diff3 = g2o::normalize_theta(pose4->theta() - pose3->theta());

    // Calculate the omega
    double omega1 = angle_diff1 / dt1->dt();
    double omega2 = angle_diff2 / dt2->dt();
    double omega3 = angle_diff3 / dt3->dt();

    // Calculate the acceleration
    double acc_rot1 = (omega2 - omega1) * 2 / dt12;
    double acc_rot2 = (omega3 - omega2) * 2 / dt23;

    // Calculate and add the jerk
    const double jerk_rot = (acc_rot2 - acc_rot1) / (dt12 / 4 + dt23 / 4);

    _error[2] = penaltyBoundToInterval(jerk_rot, cfg_->robot.jerk_lim_theta, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n", _error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() strafing: _error[1]=%f\n", _error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeJerk::computeError() rotational: _error[2]=%f\n",_error[2]);
  }

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};
    
    
/**
 * @class EdgeJerkHolonomicStart
 * @brief Edge difining the cost function for limiting the translational and rotational jerk at the beginning of the trajectory.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i \f$, \Delta T_{i+1} \f$ 
 * an initial acceleration defined by setInitialAcceleration() and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [jx, jy, omegadotdot ]^T ) \cdot weight \f$. \n
 * \e jx is calculated using the difference quotient (thrice) and the position parts of the poses. \n
 * \e jy is calculated using the difference quotient (thrice) and the position parts of the poses. \n
 * \e omegadotdot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational jerk (x-dir), 
 * the second one the strafing jerk and the third one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic
 * @see EdgeJerkHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkGoal() for defining boundary values at the end of the trajectory!
*/
class EdgeJerkHolonomicStart : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Constuct edge.
  */
  EdgeJerkHolonomicStart()
  {
    _measurement = NULL;
    this->resize(5);
  }
    
  /**
   * @brief Actual cost function
  */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig and setStartVelocity() on EdgeJerkStart()");
    // Get four poses and three deltaT
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // Using the difference quotient to calculate difference quotient
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

    // Calculate the angle difference between poses
    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta());
    double cos_theta2 = std::cos(pose2->theta());
    double sin_theta2 = std::sin(pose2->theta());

    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();
    // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
    double p2_dx =  cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

    // LINEAR
    // Calculate the velocity
    double vel0_x = _measurement->linear.x;
    double vel0_y = _measurement->linear.y;
    double vel1_x = p1_dx / dt1->dt();
    double vel1_y = p1_dy / dt1->dt();
    double vel2_x = p2_dx / dt2->dt();
    double vel2_y = p2_dy / dt2->dt();

    // Calculate total duration
    double dt12 = dt1->dt() + dt2->dt();

    // Calculate the acceleration
    double acc_x0 = (vel1_x - vel0_x) / dt1->dt();
    double acc_y0 = (vel1_y - vel0_y) / dt1->dt();
    double acc_x1 = (vel2_x - vel1_x) * 2 / dt12;
    double acc_y1 = (vel2_y - vel1_y) * 2 / dt12;

    // Calculate and add the jerk
    const double jerk_x = (acc_x1 - acc_x0) / (dt1->dt() / 2 + dt12 / 4);
    const double jerk_y = (acc_y1 - acc_y0) / (dt1->dt() / 2 + dt12 / 4);

    _error[0] = penaltyBoundToInterval(jerk_x, cfg_->robot.jerk_lim_x, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(jerk_y, cfg_->robot.jerk_lim_y, cfg_->optim.penalty_epsilon);

    // ANGULAR
    // Calculate the angle difference between poses
    double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

    // Calculate the omega
    double omega0 = _measurement->angular.z;
    double omega1 = angle_diff1 / dt1->dt();
    double omega2 = angle_diff2 / dt2->dt();

    // Calculate the acceleration
    double acc_rot0 = (omega1 - omega0) / dt1->dt();
    double acc_rot1 = (omega2 - omega1) * 2 / dt12;

    // Calculate and add the jerk
    const double jerk_rot = (acc_rot1 - acc_rot0) / (dt1->dt() / 2 + dt12 / 4);

    _error[2] = penaltyBoundToInterval(jerk_rot, cfg_->robot.jerk_lim_theta, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n", _error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() strafing: _error[1]=%f\n", _error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeJerk::computeError() rotational: _error[2]=%f\n",_error[2]);
  }

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * and use this speed to initialize the initial acceleration for the next step
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
    
    
    

/**
 * @class EdgeJerkHolonomicGoal
 * @brief Edge difining the cost function for limiting the translational and rotational jerk at the end of the trajectory.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i \f$, \Delta T_{i+1} \f$ 
 * an initial acceleration defined by setGoalAcceleration() and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [jx, jy, omegadotdot ]^T ) \cdot weight \f$. \n
 * \e j is calculated using the difference quotient (thrice) and the position parts of the poses. \n
 * \e omegadotdot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational jerk (x-dir), 
 * the second one the strafing jerk and the third one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic
 * @see EdgeJerkHolonomicStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkStart() for defining boundary values at the beginning of the trajectory!
*/
class EdgeJerkHolonomicGoal : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Constuct edge.
  */
  EdgeJerkHolonomicGoal()
  {
    _measurement = NULL;
    this->resize(5);
  }
    
  /**
   * @brief Actual cost function
  */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeJerkGoal()");

    // Obtain pose and time difference information from vertices
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // Calculate the distance difference and angle difference between poses
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose_goal->position() - pose2->position();

    // Calculate the angle difference between poses
    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta());
    double cos_theta2 = std::cos(pose2->theta());
    double sin_theta2 = std::sin(pose2->theta());

    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();
    // transform pose_goal into robot frame pose2 (inverse 2d rotation matrix)
    double p2_dx =  cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();
    
    // LINEAR
    // Calculate the velocity
    double vel1_x = p1_dx / dt1->dt();
    double vel1_y = p1_dy / dt1->dt();
    double vel2_x = p2_dx / dt2->dt();
    double vel2_y = p2_dy / dt2->dt();
    double vel3_x = _measurement->linear.x;
    double vel3_y = _measurement->linear.y;

    // Calculate total duration
    double dt12 = dt1->dt() + dt2->dt();

    // Calculate the acceleration
    double acc_x1 = (vel2_x - vel1_x) * 2 / dt12;
    double acc_y1 = (vel2_y - vel1_y) * 2 / dt12;
    double acc_x2 = (vel3_x - vel2_x) / dt2->dt();
    double acc_y2 = (vel3_y - vel2_y) / dt2->dt();

    // Calculate and add the jerk
    const double jerk_x = (acc_x2 - acc_x1) / (dt12 / 4 + dt2->dt() / 2);
    const double jerk_y = (acc_y2 - acc_y1) / (dt12 / 4 + dt2->dt() / 2);

    _error[0] = penaltyBoundToInterval(jerk_x, cfg_->robot.jerk_lim_x, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(jerk_y, cfg_->robot.jerk_lim_y, cfg_->optim.penalty_epsilon);

    // ANGULAR
    // Calculate the angle difference between poses
    double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    double angle_diff2 = g2o::normalize_theta(pose_goal->theta() - pose2->theta());

    // Calculate the omega
    double omega1 = angle_diff1 / dt1->dt();
    double omega2 = angle_diff2 / dt2->dt();
    double omega3 = _measurement->angular.z;

    // Calculate the acceleration
    double acc_rot1 = (omega2 - omega1) * 2 / dt12;
    double acc_rot2 = (omega3 - omega2) / dt2->dt();

    // Calculate and add the jerk
    const double jerk_rot = (acc_rot2 - acc_rot1) / (dt12 / 4 + dt2->dt() / 2);

    _error[2] = penaltyBoundToInterval(jerk_rot, cfg_->robot.jerk_lim_theta, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n", _error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() strafing: _error[1]=%f\n", _error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeJerk::computeError() rotational: _error[2]=%f\n",_error[2]);
  }

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * and use this speed to initialize the goal / final acceleration for the next step
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setGoalVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}; // end namespace

#endif /* EDGE_JERK_H_ */
