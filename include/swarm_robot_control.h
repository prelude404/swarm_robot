/* 
 * Date: 2021-11-29
 * Description: the basic function
 */

#include <ros/ros.h>


#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>


using std::cout;
using std::endl;

class SwarmRobot{

public:

    SwarmRobot(ros::NodeHandle *nh, std::vector<int> swarm_robot_id_);
    ~SwarmRobot();

    std::vector<int> swarm_robot_id;

    int robot_num;

    double ObsMinwidth;
    Eigen::Vector2d Obs_start_point;
    Eigen::Vector2d Obs_end_point;
    Eigen::Vector2d swarm_vel;
    double del_distance;

    bool getRobotPose(int index, std::vector<double> &pose_cur);

    bool getRobotPose(std::vector<std::vector<double> > &swarm_pose_cur);

    bool moveRobot(int index, double v, double w);

    bool moveRobot(std::vector< std::vector<double> > &speed);

    bool moveRobotByXY(int index, double theta, double vx, double vy, double MAX_V, double MAX_W);

    bool moveSwarm(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y, Eigen::Vector2d expect_point, 
                               double abs_vel, Eigen::Vector2d swarm_vel);

    bool stopRobot(int index);

    bool stopRobot();

    double checkVel(double v, double max_v, double min_v);

    bool avoidCollision(double alpha_1, double alpha_2, double rad, double df, Eigen::VectorXd cur_x,
                        Eigen::VectorXd cur_y, Eigen::VectorXd &ac_x, Eigen::VectorXd &ac_y);

    bool moveSwarm(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y, Eigen::Vector2d expect_point, double abs_vel);

    void FormationProcess(Eigen::Vector2d except_point, Eigen::VectorXd final_target_x, Eigen::VectorXd final_target_y,
                          double  MAX_V, double MAX_W, double conv_x, double conv_y, double conv_dis);

    /* get AccessArea */
    // void AcesssArea(Eigen::VectorXd obs_x_in, Eigen::VectorXd obs_y_in);

private:

   tf::TransformListener tf_listener;
   ros::Publisher cmd_vel_pub[10];

   //ROS Nodehandle
   ros::NodeHandle nh_;

};