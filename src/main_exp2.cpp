/* 
 * Date: 2021-11-29
 * Description: To a line
 */

#include <swarm_robot_control.h>
#include <Hungary.h>

using namespace std;

/************************** Global Parameter ************************/ 
/* Convergence threshold */
double conv_x = 0.1; // Threshold of x, in m
double conv_y = 0.1;  // Threshold of y, in m

/* Velocity scale and threshold */
double MAX_W = 1;       // Maximum angle velocity (rad/s)
double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
double MAX_V = 0.2;     // Maximum linear velocity(m/s)
double MIN_V = 0.0;    // Minimum linear velocity(m/s)
double k_w = 0.1;       // Scale of angle velocity
double k_v = 0.05;       // Scale of linear velocity

/* Parameters for Collision Avoidence */
double weight_repel = 1.0;
double weight_attract = 1.0;
double dis_repel = 0.4;
double dis_attract = 1.0;  

/************************** Function Statement ************************/ 
void getParameterList(ros::NodeHandle nh);

/************************** Main Function ************************/
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    getParameterList(nh);
    // cout <<"dis_repel:   "<< dis_repel << endl;

    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5, 6, 7, 8};
    const int num_robot = 6;//元素个数
    const int num_obs = 2;//障碍物个数

    /*Hungary*/
    Hungary hungary_swarm;
    hungary_swarm.num_swarm = num_robot;

    /*Formation to be choiced*/
    // // LAB2
    // const int num_formation = 3;
    // std::vector<Eigen::Matrix<double,num_robot,1>> formation_x_list(num_formation);
    // std::vector<Eigen::Matrix<double,num_robot,1>> formation_y_list(num_formation);
    // std::vector<string> formation_name_list = {"十字", "圆形", "楔形"};

    // LAB3
    const int num_formation = 2;
    std::vector<Eigen::Matrix<double,num_robot,1>> formation_x_list(num_formation);
    std::vector<Eigen::Matrix<double,num_robot,1>> formation_y_list(num_formation);
    std::vector<string> formation_name_list = {"六边形", "矩形"};



    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    /* Set L Matrix */
    Eigen::MatrixXd lap(num_robot, num_robot);
    lap << 5, -1, -1, -1, -1, -1,
           -1, 5, -1, -1, -1, -1,
           -1, -1, 5, -1, -1, -1,
           -1, -1, -1, 5, -1, -1,
           -1, -1, -1, -1, 5, -1,
           -1, -1, -1, -1, -1, 5;

    /* Set Shape */
    Eigen::VectorXd final_target_x(num_robot);
    Eigen::VectorXd final_target_y(num_robot);
    // // LAB1： "十字", "圆形", "楔形"
    // formation_x_list[0] << 0.0, 1.0, 0.0, -1.0, 0.0;
    // formation_y_list[0] << 0.0, 0.0, 1.0, 0.0, -1.0;
    // formation_x_list[1] << 0.0, 0.45, -0.45, 0.4, -0.4;
    // formation_y_list[1] << 1.0, 0.2, 0.2, -0.3, -0.3;
    // formation_x_list[2] << 0.0, 0.5, -0.5, 1.0, -1.0;
    // formation_y_list[2] << 1.0, 0.0, 0.0, -1.0, -1.0;
    
    // LAB2: 六边形 矩形
    formation_x_list[0] << 0.5, -0.5, -1.0, 1.0, -0.5, 0.5;
    formation_y_list[0] << 0.87, 0.87, 0.0, 0.0, -0.87, -0.87;
    formation_x_list[1] << 0.0, 0.5, -0.5, 0.0, 0.5, -0.5;
    formation_y_list[1] << 0.2, 0.2, 0.2, -0.2, -0.2,-0.2;
  

    /* Mobile robot poses and for next poses */
    Eigen::VectorXd cur_x(num_robot);
    Eigen::VectorXd cur_y(num_robot);
    Eigen::VectorXd cur_theta(num_robot);
    Eigen::VectorXd del_x(num_robot);
    Eigen::VectorXd del_y(num_robot);
    Eigen::VectorXd del_theta(num_robot);
    Eigen::VectorXd ac_x(num_robot);
    Eigen::VectorXd ac_y(num_robot);

    Eigen::VectorXd obs_x(num_obs);
    Eigen::VectorXd obs_y(num_obs);

 
    /* Get swarm robot poses firstly */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size());
    swarm_robot.getRobotPose(current_robot_pose);

    /* x,y,theta */
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        if(i < num_robot){
            cur_x(i) = current_robot_pose[i][0];
            cur_y(i) = current_robot_pose[i][1];
            cur_theta(i) = current_robot_pose[i][2];
        }
        else{
            obs_x(i-num_robot) = current_robot_pose[i][0];
            obs_y(i-num_robot) = current_robot_pose[i][1];
        }
        
    }


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> FORMATION SELECT <<<<<<<<<<<<<<<<<<<<<<<<<<<<
    double final_assign[num_robot];
    double totalCost_min = 10000.0;
    int final_formation_index = 0;

    cout << "\033[33m>>>>>>>>>>>>>>>>>>> FORMATION SELECT <<<<<<<<<<<<<<<<<\033[0m" << endl;
    for (int i = 0; i < 1; i++) //TODO
    {
        hungary_swarm.hungary(cur_x, cur_y, formation_x_list[i], formation_y_list[i]);
        cout << "The total cost of formation \033[32m" << formation_name_list[i] << "\033[0m is: \033[32m" << hungary_swarm.totalCost << "\033[0m" << endl;
        if (hungary_swarm.totalCost < totalCost_min)
        {
            totalCost_min = hungary_swarm.totalCost;
            final_formation_index = i;
            for (int j = 0; j < num_robot; j++){
                final_assign[j] = hungary_swarm.assign[j];
            }
        }
    }

    cout << "The final formation is: \033[32m" << formation_name_list[final_formation_index] << "\033[0m" << endl;
    for (int i = 0; i < num_robot; i++)
    {
        cout << "Angent" << i + 1 << "-->"<< "Target" << final_assign[i] + 1 << endl;
        final_target_x(i) = formation_x_list[final_formation_index](final_assign[i]);
        final_target_y(i) = formation_y_list[final_formation_index](final_assign[i]);
    }

    cout << "press any key to continue" << endl;
    cin.get();


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> FORMATION CONTROL <<<<<<<<<<<<<<<<<<<<<<<<<<<<
    /* Convergence sign */
    bool is_conv_x = false;   // Convergence sign of x
    bool is_conv_y = false;   // Convergence sign of angle

    Eigen::Vector2d except_point;
    except_point(0) = obs_x.mean() - 1.0;
    except_point(1) = obs_y.mean();

    // TO Formation A
    while (((!is_conv_x) || (!is_conv_y)) && ros::ok())
    {
        /* Judge whether reached */
        Eigen::Vector2d swarm_vel;
        swarm_robot.moveSwarm(cur_x, cur_y, except_point, 0.1);
        swarm_vel = swarm_robot.swarm_vel;
        del_x = -lap * cur_x + lap * final_target_x;
        del_y = -lap * cur_y + lap * final_target_y;
        /* Collision Avoidence */
        // if(swarm_robot.avoidCollision(weight_repel, weight_attract, dis_repel, dis_attract,cur_x, cur_y, ac_x, ac_y)){
        //     del_x += ac_x;
        //     del_y += ac_y;
        // }

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
            // double vx = del_x(i) * k_v;
            // double vy = del_y(i) * k_v;
            double vx = del_x(i) * k_v + swarm_vel(0);
            double vy = del_y(i) * k_v + swarm_vel(1);
            swarm_robot.moveRobotByXY(i, cur_theta(i), vx, vy, MAX_V, MAX_W);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);

        for(int i = 0; i < num_robot; i++) {
            cur_x(i) = current_robot_pose[i][0];
            cur_y(i) = current_robot_pose[i][1];
            cur_theta(i) = current_robot_pose[i][2];
        }
    }
    
    // >>>>>>>>>>>>>>>>>>>>>>>From A To B<<<<<<<<<<<<<<<<<<<<<<<<
    /* Convergence sign */
    is_conv_x = false;   // Convergence sign of x
    is_conv_y = false;   // Convergence sign of angle

    double obs_width_half = abs(obs_y(0) - obs_y(1)) / 2;
    formation_y_list[1] << obs_width_half, obs_width_half, obs_width_half, -obs_width_half, -obs_width_half, -obs_width_half;

    hungary_swarm.hungary(cur_x, cur_y, formation_x_list[1], formation_y_list[1]);

    cout << "The final formation is: \033[32m" << formation_name_list[final_formation_index] << "\033[0m" << endl;
    for (int i = 0; i < num_robot; i++)
    {
        cout << "Angent" << i + 1 << "-->"<< "Target" << hungary_swarm.assign[i]+ 1 << endl;
        final_target_x(i) = formation_x_list[1](hungary_swarm.assign[i]);
        final_target_y(i) = formation_y_list[1](hungary_swarm.assign[i]);
    }

    cout << "press any key to continue" << endl;
    cin.get();

    // TO Formation A
    while (((!is_conv_x) || (!is_conv_y)) && ros::ok())
    {
        /* Judge whether reached */
        del_x = -lap * cur_x + lap * final_target_x;
        del_y = -lap * cur_y + lap * final_target_y;
        /* Collision Avoidence */
        // if(swarm_robot.avoidCollision(weight_repel, weight_attract, dis_repel, dis_attract,cur_x, cur_y, ac_x, ac_y)){
        //     del_x += ac_x;
        //     del_y += ac_y;
        // }

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
            // double vx = del_x(i) * k_v;
            // double vy = del_y(i) * k_v;
            double vx = del_x(i) * k_v;
            double vy = del_y(i) * k_v;
            swarm_robot.moveRobotByXY(i, cur_theta(i), vx, vy, MAX_V, MAX_W);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);

        for(int i = 0; i < num_robot; i++) {
            cur_x(i) = current_robot_pose[i][0];
            cur_y(i) = current_robot_pose[i][1];
            cur_theta(i) = current_robot_pose[i][2];
        }
    }




    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}

/* Read parameter list */
void getParameterList(ros::NodeHandle nh){

    nh.getParam("/gazebo_swarm_robot_control/conv_x", conv_x);
    nh.getParam("/gazebo_swarm_robot_control/conv_y", conv_y);
    nh.getParam("/gazebo_swarm_robot_control/MAX_W", MAX_W);
    nh.getParam("/gazebo_swarm_robot_control/MIN_W", MIN_W);
    nh.getParam("/gazebo_swarm_robot_control/MAX_V", MAX_V);
    nh.getParam("/gazebo_swarm_robot_control/MIN_V", MIN_V);
    nh.getParam("/gazebo_swarm_robot_control/k_w", k_w);
    nh.getParam("/gazebo_swarm_robot_control/k_v", k_v);
    nh.getParam("/gazebo_swarm_robot_control/weight_repel", weight_repel);
    nh.getParam("/gazebo_swarm_robot_control/weight_attract", weight_attract);
    nh.getParam("/gazebo_swarm_robot_control/dis_repel", dis_repel);
    nh.getParam("/gazebo_swarm_robot_control/dis_attract", dis_attract);

    ROS_INFO("Parameters list reading finished!");
}