#include <iostream>
#include <cmath>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

using namespace std;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << "please enter a 3D vector" << endl;
    }
    //first_step::初始化一个旋转矩阵sophus matrix都可以
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    Sophus::SO3 SO3_R(R);              // Sophus::SO(3)可以直接从旋转矩阵构造
    Sophus::SO3 SO3_v(0, 0, M_PI / 2); // 亦可从旋转向量构造
    Eigen::Quaterniond q(R);           // 或者四元数
    Sophus::SO3 SO3_q(q);
    // 上述表达方式都是等价的
    // 输出SO(3)时，以so(3)形式输出
    cout << "SO(3) from matrix: " << SO3_R << endl;
    cout << "SO(3) from vector: " << SO3_v << endl;
    cout << "SO(3) from quaternion :" << SO3_q << endl;

    //second_step::接受你输入的任意旋转向量转化成旋转矩阵和四元数更新形式
    int vector[3];
    for (int i = 0; i < 3; i++)
    {
        vector[i] = argv[i];
    }
    //matrix形式的矩阵 result_matrix
    //TODO
    Eigen::Vector3d VectorMatrix << vector[0], vector[1], vector[2];
    Eigen::Matrix<double, 3, 3> result_matrix = R * VectorMatrix;
    //sophus形式的矩阵 result_sophus
    //TODO
    Eigen::Vector3d update_so3(vector[0], vector[1], vector[2]);
    Sophus::SO3 result_sophus = Sophus::SO3::exp(update_so3) * SO3_R;

    //把他俩都变成一个形式做个对比 或者直接输出
    cout << "result_sophus = " << result_sophus.matrix() << endl;
    cout << "result_matrix = " << result_matrix << endl;
}