//
// Created by zm on 18-12-1.
//

#ifndef OFFB_POSCTL_PARAMETER_H
#define OFFB_POSCTL_PARAMETER_H

#include <Eigen/Eigen>

class Parameter {

public:
    Eigen::Vector3f pp;
    Eigen::Vector3f pi;
    Eigen::Vector3f pd;

    Eigen::Vector3f vp;
    Eigen::Vector3f vi;
    Eigen::Vector3f vd;



    bool readParam(const char* addr);

};


#endif //OFFB_POSCTL_PARAMATER_H
