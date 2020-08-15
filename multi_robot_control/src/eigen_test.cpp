#include <iostream>  
#include <Eigen/Eigen>  
#include <stdlib.h>  
#include <Eigen/Geometry>  
#include <Eigen/Core>  
#include <vector>  
#include <math.h>  
#include "ros/ros.h"
  
using namespace std;  
using namespace Eigen;  
  
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)  
{  
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());  
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());  
  
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;  
    cout << "Euler2Quaternion result is:" <<endl;  
    cout << "x = " << q.x() <<endl;  
    cout << "y = " << q.y() <<endl;  
    cout << "z = " << q.z() <<endl;  
    cout << "w = " << q.w() <<endl<<endl;  
    return q;  
}  
  
Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;  
  
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  
    cout << "Quaterniond2Euler result is:" <<endl;  
    cout << "x = "<< euler[2] << endl ;  
    cout << "y = "<< euler[1] << endl ;  
    cout << "z = "<< euler[0] << endl << endl;  
}  
  
Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;  
  
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();  
    cout << "Quaternion2RotationMatrix result is:" <<endl;  
    cout << "R = " << endl << R << endl<< endl;  
    return R;  
}  
  
  
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)  
{  
    Eigen::Quaterniond q = Eigen::Quaterniond(R);  
    q.normalize();  
    cout << "RotationMatrix2Quaterniond result is:" <<endl;  
    cout << "x = " << q.x() <<endl;  
    cout << "y = " << q.y() <<endl;  
    cout << "z = " << q.z() <<endl;  
    cout << "w = " << q.w() <<endl<<endl;  
    return q;  
}  
  
Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw)  
{  
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());  
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());  
  
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;  
    Eigen::Matrix3d R = q.matrix();  
    cout << "Euler2RotationMatrix result is:" <<endl;  
    cout << "R = " << endl << R << endl<<endl;  
    return R;  
}  
  
Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R)  
{  
    Eigen::Matrix3d m;  
    m = R;  
    Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);  
    cout << "RotationMatrix2euler result is:" << endl;  
    cout << "x = "<< euler[2] << endl ;  
    cout << "y = "<< euler[1] << endl ;  
    cout << "z = "<< euler[0] << endl << endl;  
    return euler;  
}  
  
  
int main(int argc, char **argv)  
{ 
  cout<<"Euler angle is: "<<endl;
  Quaterniond2Euler(0.025711392457025174, 0.9973543463037003, 0.06268333327193465, 0.02634448589285042);  
}  