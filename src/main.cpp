/* 
 * Date: 2021-11-29
 * Description: To a line
 */

#include <swarm_robot_control.h>
#include <Hungary.h>

using namespace std;

/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    
    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};
    const int num_swarm = 5;//元素个数

    /*Hungary*/
    Hungary hungary_swarm;
    hungary_swarm.num_swarm = num_swarm;

    /*Formation to be choiced*/
    const int num_formation = 3;
    std::vector<Eigen::Matrix<double,num_swarm,1>> formation_x_list(num_formation);
    std::vector<Eigen::Matrix<double,num_swarm,1>> formation_y_list(num_formation);
    std::vector<string> formation_name_list = {"十字", "圆形", "楔形"};

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    int robot_num = swarm_robot_id.size();

    /* Set L Matrix */
    Eigen::MatrixXd lap(robot_num, robot_num);
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;

    /* Set Shape */
    Eigen::VectorXd final_target_x(robot_num);
    Eigen::VectorXd final_target_y(robot_num);
    //"十字", "圆形", "楔形"
    formation_x_list[0] << 0.0, 1.0, 0.0, -1.0, 0.0;
    formation_y_list[0] << 0.0, 0.0, 1.0, 0.0, -1.0;
    formation_x_list[1] << 0.0, 0.45, -0.45, 0.4, -0.4;
    formation_y_list[1] << 1.0, 0.2, 0.2, -0.3, -0.3;
    formation_x_list[2] << 0.0, 0.5, -0.5, 1.0, -1.0;
    formation_y_list[2] << 1.0, 0.0, 0.0, -1.0, -1.0;


    /* Convergence threshold */
    double conv_y = 0.1;  // Threshold of y, in m
    double conv_x = 0.1; // Threshold of x, in m


    /* Velocity scale and threshold */
    double MAX_W = 1;       // Maximum angle velocity (rad/s)
    double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;     // Maximum linear velocity(m/s)
    double MIN_V = 0.0;    // Minimum linear velocity(m/s)
    double k_w = 0.1;       // Scale of angle velocity
    double k_v = 0.05;       // Scale of linear velocity


    /* Mobile robot poses and for next poses */
    Eigen::VectorXd cur_x(swarm_robot_id.size());
    Eigen::VectorXd cur_y(swarm_robot_id.size());
    Eigen::VectorXd cur_theta(swarm_robot_id.size());
    Eigen::VectorXd del_x(swarm_robot_id.size());
    Eigen::VectorXd del_y(swarm_robot_id.size());
    Eigen::VectorXd del_theta(swarm_robot_id.size());
    Eigen::VectorXd ac_x(swarm_robot_id.size());
    Eigen::VectorXd ac_y(swarm_robot_id.size());

    /* Parameters for Collision Avoidence */
    double alpha_1 = 1;
    double alpha_2 = 1;
    double rad = 0.4;
    double df = 1.0;    

    /* Get swarm robot poses firstly */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size());
    swarm_robot.getRobotPose(current_robot_pose);

    /* x,y,theta */
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0];
        cur_y(i) = current_robot_pose[i][1];
        cur_theta(i) = current_robot_pose[i][2];
    }


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> FORMATION SELECT <<<<<<<<<<<<<<<<<<<<<<<<<<<<
    double final_assign[num_swarm];
    double totalCost_min = 10000.0;
    int final_formation_index = 0;

    cout << "\033[33m>>>>>>>>>>>>>>>>>>> FORMATION SELECT <<<<<<<<<<<<<<<<<\033[0m" << endl;
    for (int i = 0; i < num_formation; i++)
    {
        hungary_swarm.hungary(cur_x, cur_y, formation_x_list[i], formation_y_list[i]);
        cout << "The total cost of formation \033[32m" << formation_name_list[i] << "\033[0m is: \033[32m" << hungary_swarm.totalCost << "\033[0m" << endl;
        if (hungary_swarm.totalCost < totalCost_min)
        {
            totalCost_min = hungary_swarm.totalCost;
            final_formation_index = i;
            for (int j = 0; j < num_swarm; j++){
                final_assign[j] = hungary_swarm.assign[j];
            }
        }
    }

    cout << "The final formation is: \033[32m" << formation_name_list[final_formation_index] << "\033[0m" << endl;
    for (int i = 0; i < num_swarm; i++)
    {
        cout << "Angent" << i + 1 << "-->"<< "Target" << final_assign[i] + 1 << endl;
        final_target_x(i) = formation_x_list[final_formation_index](final_assign[i]);
        final_target_y(i) = formation_y_list[final_formation_index](final_assign[i]);
        cout << final_target_x(i) << endl;
    }

    cout << "press any key to continue" << endl;
    cin.get();


    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> FORMATION CONTROL <<<<<<<<<<<<<<<<<<<<<<<<<<<<
    /* Convergence sign */
    bool is_conv_x = false;   // Convergence sign of x
    bool is_conv_y = false;   // Convergence sign of angle

    while(((!is_conv_x) || (!is_conv_y)) && ros::ok()) {
        /* Judge whether reached */
        del_x = -lap * cur_x + lap * final_target_x;
        del_y = -lap * cur_y + lap * final_target_y;

        /* Collision Avoidence */
        if(swarm_robot.avoidCollision(alpha_1, alpha_2, rad, df,cur_x, cur_y, ac_x, ac_y)){
            del_x += ac_x;
            del_y += ac_y;
        }

        is_conv_x = true;
        is_conv_y = true;

        for (int i = 0; i < swarm_robot_id.size(); i++)
        {
            if (std::fabs(del_x(i)) > conv_x){
                is_conv_x = false;
            }
            if (std::fabs(del_y(i)) > conv_y) {
                is_conv_y = false;
            }
        }

        /* Swarm robot move */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            double vx = del_x(i) * k_v;
            double vy = del_y(i) * k_v;
            swarm_robot.moveRobotByXY(i, cur_theta(i), vx, vy);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);

        for(int i = 0; i < swarm_robot_id.size(); i++) {
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