/*
* Date: 2021.12.8
* Describtion: Use Hungarian algorithm for formation selection
*/

#ifndef HUNGARY_H
#define HUNGARY_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>

class Hungary {

public:
    int num_swarm;//元素个数
    int *assign;//分配结果
    double **mat;//代价矩阵
    double **matRcd;//代价矩阵
    double totalCost;//总成本

    bool read(const char *ad);//数据读取
    bool get_matrix(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y, Eigen::VectorXd target_x, Eigen::VectorXd target_y);
    void rowSub(); //行归约
    void columnSub(); //列归约
    bool isOptimal(int *assign); //检验最优
    void matTrans(); //矩阵变换
    void hungary(Eigen::VectorXd cur_x, Eigen::VectorXd cur_y, Eigen::VectorXd target_x, Eigen::VectorXd target_y);
};

#endif //HUNGARY_H