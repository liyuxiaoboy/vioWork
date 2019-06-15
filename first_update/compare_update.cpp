#include <iostream>
#include <cmath>
#include <ctime>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

using namespace std;
void QuaternMutiple(const Eigen::Quaterniond *src, const Eigen::Quaterniond *update_quatern, Eigen::Quaterniond *update_result)
{
    cout << "src : " << src->x() << "  " << src->y() << "  " << src->z() << "  " << src->w() << endl;
    double s_src = src->w();
    double s_update = update_quatern->w();
    Eigen::Vector3d src_v(src->x(), src->y(), src->z());
    Eigen::Vector3d update_v(update_quatern->x(), update_quatern->y(), update_quatern->z());
    double s_result = s_src * s_update - src_v.transpose() * update_v;
    cout << "s_result = " << s_result << endl
         << endl;
    Eigen::Vector3d result_v = s_src * update_v + s_update * src_v + src_v.cross(update_v);
    cout << "QuaternMutiple_result_v = \n"
         << result_v << endl
         << endl;
    Eigen::Quaterniond result(s_result, result_v[0], result_v[1], result_v[2]);
    *update_result = result;
}

void updateQuatern(const Eigen::Quaterniond *src, const double *update_vector, Eigen::Quaterniond *updateQuaternResult)
{
    cout << "src : " << src->x() << "  " << src->y() << "  " << src->z() << "  " << src->w() << endl
         << endl;
    cout << "update : " << update_vector[0] << "  " << update_vector[1] << "  " << update_vector[2] << endl
         << endl;
    double s_src = src->w();
    double s_update = 1;
    // Eigen::Vector3d src_3d(src->x(),src->y(),src->z());
    // Eigen::Vector3d update_3d(update_vector[0],update_vector[1],update_vector[2]);
    Eigen::Quaterniond update_quatern(update_vector[0], update_vector[1], update_vector[2], static_cast<double>(1));
    cout << "apppro_update_sophus = \n"
         << update_quatern.matrix() << endl
         << endl;
    Eigen::Quaterniond first_update_result;
    QuaternMutiple(&update_quatern, src, &first_update_result);
    Eigen::Quaterniond second_update_result;
    Eigen::Quaterniond update_quatern_inverse = update_quatern.inverse();
    QuaternMutiple(&first_update_result, &update_quatern_inverse, &second_update_result);
    cout << "update_result = \n"
         << second_update_result.coeffs() << endl;
    *updateQuaternResult = second_update_result;
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << "please enter a 3D vector" << endl;
        return -1;
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
    double vector[3];
    for (int i = 0; i < 3; i++)
    {
        vector[i] = stod(argv[i + 1]);
    }
    //vector2rotation_vector
    double stdVector = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    Eigen::AngleAxisd vectorUpdate(stdVector, Eigen::Vector3d(vector[0] / stdVector, vector[1] / stdVector, vector[2] / stdVector));

    Eigen::Quaterniond q_update(vectorUpdate);
    Eigen::Matrix3d r_update = vectorUpdate.toRotationMatrix();
    cout << "update_sophus = \n"
         << q_update.coeffs() << endl
         << endl;
    cout << "update_matrix = \n"
         << r_update << endl
         << endl;
    Eigen::Matrix<double, 3, 3> result_matrix = R * r_update;
    //update by quaternUpdate
    Eigen::Quaterniond sophusResult;
    updateQuatern(&q, vector, &sophusResult);
    cout << "apppro update result is : \n"
         << sophusResult.toRotationMatrix() << endl
         << endl;
    cout << "update result by matrix is : \n"
         << result_matrix << endl;
    //matrix形式的矩阵 result_matrix
    //TODO
    // Eigen::Matrix<double, 3, 1> result_matrix = R * VectorMatrix;
    // //sophus形式的矩阵 result_sophus
    // //TODO
    // Eigen::Vector3d update_so3(vector[0], vector[1], vector[2]);
    // Sophus::SO3 result_sophus = Sophus::SO3::exp(update_so3) * SO3_R;

    // //把他俩都变成一个形式做个对比 或者直接输出
    // cout << "result_sophus = " << result_sophus.matrix() << endl;
    // cout << "result_matrix = " << result_matrix << endl;
}