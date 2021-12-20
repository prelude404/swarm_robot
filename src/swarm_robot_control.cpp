/* 
 * Date: 2021-11-29
 * Description: the basic function
 */

#include <swarm_robot_control.h>

SwarmRobot::SwarmRobot(ros::NodeHandle *nh, std::vector<int> swarm_robot_id_):
        swarm_robot_id(swarm_robot_id_)
{

    this->robot_num = swarm_robot_id.size();

    std::cout << "robot_num="<< robot_num <<std::endl;
    /* Initialize swarm robot */
    for(int i = 0; i < 10; i++) {
        std::string vel_topic = "/robot_" + std::to_string(i+1) + "/cmd_vel";
        cmd_vel_pub[i] = nh_.advertise<geometry_msgs::Twist>(vel_topic, 10);
    }

}


SwarmRobot::~SwarmRobot()
{

}


/* Get the [index]th robot's pose to vector*/
bool SwarmRobot::getRobotPose(int index, std::vector<double> &pose_cur) {

	pose_cur.resize(3,0.0);
    tf::StampedTransform transform;
    std::string robot_frame = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/base_footprint";
    std::string base_marker = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/odom";


    // Try to get pose of robot from tf
    try{
        this->tf_listener.waitForTransform(base_marker, robot_frame, ros::Time(0), ros::Duration(0.5));
        this->tf_listener.lookupTransform(base_marker, robot_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
        return false;
    }

    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get pose
    pose_cur.resize(3);
    pose_cur[0] = transform.getOrigin().x();
    pose_cur[1] = transform.getOrigin().y();
    pose_cur[2] = yaw;

    // ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << pose_cur[0] << " y=" << pose_cur[1] << " theta=" << pose_cur[2]);
    return true;
}


/* Get gazebo pose of swarm robot */
bool SwarmRobot::getRobotPose(std::vector<std::vector<double> > &current_robot_pose) {

    current_robot_pose.resize(this->robot_num);
    std::vector<bool> flag_pose(this->robot_num, false);

    bool flag = false;

    while(! flag) {
        flag = true;
        for(int i = 0; i < this->robot_num; i++) {
            flag = flag && flag_pose[i];
        }
        for(int i = 0; i < this->robot_num; i++) {
            std::vector<double> pose_robot(3);
            if(getRobotPose(i, pose_robot)) {
                current_robot_pose[i] = pose_robot;
                flag_pose[i] = true;
            }
        }
    }
    ROS_INFO_STREAM("Succeed getting pose!");
    return true;   
}

/* Move robot: index represents the order in swarm_robot_id*/
bool SwarmRobot::moveRobot(int index, double v, double w) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = v;
    vel_msg.angular.z = w;
    cmd_vel_pub[swarm_robot_id[index]-1].publish(vel_msg);
    ROS_INFO_STREAM("Move robot_" << swarm_robot_id[index] << " with v=" << v << " w=" << w);
    return true;
}

/* Move all robots*/
bool SwarmRobot::moveRobot(std::vector< std::vector<double> > &speed) {
  if(this->robot_num != speed.size())
  {
    ROS_INFO_STREAM("The robot number does not equal the speed number!");
    return false;
  }

  for(int i=0; i<this->robot_num; i++)
  {
    if(!this->moveRobot(this->swarm_robot_id[i],speed[i][0],speed[i][1]))
    {
      return false;
    }
  }
  return true;
}

/* Move robot by vx、vy */
bool SwarmRobot::moveRobotByXY(int index, double theta, double vx, double vy, double MAX_V, double MAX_W)
{
    double const_x = 0.1;
    double const_y = 0.0;
    Eigen::Matrix2d mat_trans;
    mat_trans << cos(theta), -(const_y * cos(theta) + const_x * sin(theta)),
                 sin(theta), const_x * cos(theta) - const_y * sin(theta);
    Eigen::Matrix2d mat_trans_inv = mat_trans.inverse();

    double temp_v = mat_trans_inv(0,0) * vx +mat_trans_inv(0,1) * vy;
    double temp_w = mat_trans_inv(1,0) * vx +mat_trans_inv(1,1) * vy;

    //超过阈值，等比缩放
    double Scale_v = abs(temp_v / MAX_V);
    double Scale_w = abs(temp_w / MAX_W);
    if(Scale_v > 1 || Scale_w > 1){
        double Scale = (Scale_v > Scale_w) ? Scale_v : Scale_w;
        temp_v = temp_v / Scale;
        temp_w = temp_w / Scale;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = temp_v;
    vel_msg.angular.z = temp_w;
    cmd_vel_pub[swarm_robot_id[index] - 1].publish(vel_msg);

    //TEST
    if (index == 2){
        ROS_INFO_STREAM("Move robot_theta" << swarm_robot_id[index] << theta);
        ROS_INFO_STREAM("Move robot_" << swarm_robot_id[index] << " with v=" << vel_msg.linear.x << " w=" << vel_msg.angular.z);
    }

    return true;
}

/* Move robot in Swarm */
bool SwarmRobot::moveSwarm(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y, Eigen::Vector2d expect_point, double abs_vel)
{
    double mean_x = cur_x.mean();
    double mean_y = cur_y.mean();
    double del_x = expect_point(0) - mean_x;
    double del_y = expect_point(1) - mean_y;
    SwarmRobot::swarm_vel(0) = abs_vel * del_x / sqrt(pow(del_x, 2) + pow(del_y, 2));
    SwarmRobot::swarm_vel(1) = abs_vel * del_y / sqrt(pow(del_x, 2) + pow(del_y, 2));
    return true;
}


/* Stop robot */
bool SwarmRobot::stopRobot(int index) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    cmd_vel_pub[swarm_robot_id[index]-1].publish(vel_msg);
    ROS_INFO_STREAM("Stop robot_" << swarm_robot_id[swarm_robot_id[index]]);
    return true;
}

/* Stop all robot */
bool SwarmRobot::stopRobot() {

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    for(int i = 0; i < this->robot_num; i++) {
        cmd_vel_pub[this->swarm_robot_id[i]-1].publish(vel_msg);
    }
    ROS_INFO_STREAM("Stop all robots.");
    return true;
}

/* Check velocity */
double SwarmRobot::checkVel(double v, double max_v, double min_v) {
    if(max_v <= 0 || min_v <= 0) {
        std::cout << "Error input of checkW()" << std::endl;
        return v;
    }
    
    if(v > 0) {
        v = std::max(v, min_v);
        v = std::min(v, max_v);
    } else {
        v = std::min(v, -min_v);
        v = std::max(v, -max_v);
    }
    return v;
}

/* Collision Avoidence */
bool SwarmRobot::avoidCollision(double alpha_1, double alpha_2, double rad, double df, Eigen::VectorXd cur_x, 
                                Eigen::VectorXd cur_y, Eigen::VectorXd &ac_x, Eigen::VectorXd &ac_y)
{
    bool flag = false;
    Eigen::MatrixXd r(this->robot_num, this->robot_num);
    Eigen::MatrixXd v(this->robot_num, this->robot_num);
    for(int i=0; i < this->robot_num; i++){
        ac_x(i)=0.0; ac_y(i)=0.0;
    }
    
    for(int i=0; i < this->robot_num; i++){
        r(i,i) = 0.0; v(i,i) = 0.0;
        for(int j=i+1; j < this->robot_num; j++){
            r(i,j) = sqrt(pow((cur_x(j)-cur_x(i)),2) + pow((cur_y(j)-cur_y(i)),2));
            if(r(i,j) <= rad){
                //rapel
                v(i,j) = alpha_1 * ((1/r(i,j) - 1/rad) - (1/df - 1/rad));
                flag = flag||true;
            }
            else if(r(i,j)>rad && r(i,j)<df){
                //attract
                v(i,j) = - alpha_2 * (df - r(i,j));
                flag = flag||true;
            }
            else{
                //distant
                v(i,j) = 0;
            }
            r(j,i) = r(i,j);
            v(j,i) = v(i,j);
            ac_x(i) += -(cur_x(j)-cur_x(i))/r(i,j) * v(i,j);
            ac_y(i) += -(cur_y(j)-cur_y(i))/r(i,j) * v(i,j);
            ac_x(j) -= ac_x(i);
            ac_y(j) -= ac_y(i);
        }
    }

    return flag;
}

// /* get AccessArea */
// void SwarmRobot::AcesssArea(Eigen::VectorXd obs_x_in, Eigen::VectorXd obs_y_in)
// {
//     int obs_num = obs_x_in.cols();
//     bool swapped;

//     Eigen::VectorXd obs_x = obs_x_in;
//     Eigen::VectorXd obs_y = obs_y_in;
//     double temp_x;
//     double temp_y;
//     std::cout << "IMHERE" << std::endl;
//     //rank obsacles from left_up to right_down
//     for(int i=0; i<obs_num; i++)
//     {
//         swapped = false;
//         for(int j = 0; j < obs_num - 1 - i; j++)
//             if(obs_y(j) < obs_y(j+1))
//             {
//                 temp_x = obs_x(j);
//                 temp_y = obs_y(j);
//                 obs_x(j) = obs_x(j+1);
//                 obs_y(j) = obs_y(j+1);
//                 obs_x(j+1) = temp_x;
//                 obs_y(j+1) = temp_y;
//                 if (!swapped) {swapped = true;}

//             }
//         if (!swapped) {break;}
//     }
//     for(int i=0; i<(obs_num/2); i++)
//     {
//         swapped = false;
//         for(int j = 0; j < (obs_num/2 - 1 - i); j++)
//             if(obs_x(j) > obs_x(j+1))
//             {
//                 temp_x = obs_x(j);
//                 temp_y = obs_y(j);
//                 obs_x(j) = obs_x(j+1);
//                 obs_y(j) = obs_y(j+1);
//                 obs_x(j+1) = temp_x;
//                 obs_y(j+1) = temp_y;
//                 if (!swapped) {swapped = true;}

//             }
//         if (!swapped) {break;}
//     }
//     for(int i=0; i <(obs_num/2); i++)
//     {
//         swapped = false;
//         for(int j = obs_num/2; j < (obs_num - 1 - i); j++)
//             if(obs_x(j) > obs_x(j+1))
//             {
//                 temp_x = obs_x(j);
//                 temp_y = obs_y(j);
//                 obs_x(j) = obs_x(j+1);
//                 obs_y(j) = obs_y(j+1);
//                 obs_x(j+1) = temp_x;
//                 obs_y(j+1) = temp_y;
//                 if (!swapped) {swapped = true;}

//             }
//         if (!swapped) {break;}
//     }
    
//     //pick access area
//     SwarmRobot::ObsMinwidth = 1e8;
//     for(int i=0;i<(obs_num/2); i++)
//     {
//         SwarmRobot::ObsMinwidth = std::min((obs_x(i) - obs_x(i + obs_num / 2)), SwarmRobot::ObsMinwidth);
//     }

//     SwarmRobot::Obs_start_point(0) = (obs_x(0) + obs_x(obs_num/2))/2;
//     SwarmRobot::Obs_start_point(1) = (obs_y(0) + obs_y(obs_num/2))/2;

//     SwarmRobot::Obs_end_point(0) = (obs_x(obs_num/2-1) + obs_x(obs_num-1))/2;
//     SwarmRobot::Obs_end_point(1) = (obs_y(obs_num/2-1) + obs_y(obs_num-1))/2;

// }

void SwarmRobot::FormationProcess(Eigen::Vector2d except_point, Eigen::VectorXd final_target_x, Eigen::VectorXd final_target_y,
                                  double  MAX_V, double MAX_W, double conv_x, double conv_y, double conv_dis)
{   del_distance = 1000;
    is_conv_x = false;
    is_conv_y = false;
    std::vector<std::vector<double> > current_robot_pose(this->robot_num);
    num_robot = this->robot_num;

    while (((!is_conv_x) || (!is_conv_y) || del_distance > conv_dis) && ros::ok())
    {
        /* Get swarm robot poses */
        this->getRobotPose(current_robot_pose);

        for(int i = 0; i < num_robot; i++) {
            cur_x(i) = current_robot_pose(i)(0);
            cur_y(i) = current_robot_pose(i)(1);
            cur_theta(i) = current_robot_pose(i)(2);
        }
        
        /* Judge whether reached */
        this->moveSwarm(cur_x, cur_y, except_point, 0.1);
        del_distance = this->del_distance;
        swarm_vel = this->swarm_vel;

        del_x = -lap * cur_x + lap * final_target_x;
        del_y = -lap * cur_y + lap * final_target_y;
 
        is_conv_x = true;
        is_conv_y = true;

        for (int i = 0; i < num_robot; i++)
        {
            if (std::fabs(del_x(i)) > conv_x){
                is_conv_x = false;
            }
            if (std::fabs(del_y(i)) > conv_y) {
                is_conv_y = false;
            }
        }

        /* Swarm robot move */
        for(int i = 0; i < num_robot; i++) {
            double vx = del_x(i) * k_v + swarm_vel(0);
            double vy = del_y(i) * k_v + swarm_vel(1);
            this->moveRobotByXY(i, cur_theta(i), vx, vy, MAX_V, MAX_W);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

    }

}