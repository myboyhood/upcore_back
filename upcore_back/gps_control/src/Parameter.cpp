//
// Created by zm on 18-12-1.
//

#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include "string"
#include <time.h>
#include <queue>
#include <vector>
#include <gps_control/Parameter.h>

bool Parameter::readParam(const char *addr) {

    std::ifstream fs;

    std::string name = "";
    float value[3];

    fs.open(addr);

    if (!fs)
    {
        std::cout << "parameter file err" << std::endl;
        return false;
    }

    while (!fs.eof())
    {
        fs >> name >> value[0] >> value[1] >> value[2];

        if (name == "pp")
        {
            pp[0] = value[0];
            pp[1] = value[1];
            pp[2] = value[2];
        }

        if (name == "pi")
        {
            pi[0] = value[0];
            pi[1] = value[1];
            pi[2] = value[2];
        }
        if (name == "pd")
        {
            pd[0] = value[0];
            pd[1] = value[1];
            pd[2] = value[2];

        }
        if(name == "vp")
        {
            vp[0] = value[0];
            vp[1] = value[1];
            vp[2] = value[2];

        }
        if (name == "vi")
        {
            vi[0] = value[0];
            vi[1] = value[1];
            vi[2] = value[2];

        }
        if (name == "vd")
        {
            vd[0] = value[0];
            vd[1] = value[1];
            vd[2] = value[2];
        }

    }
    std::cout << "read config file successfully!"<<std::endl;

    fs.close();

    return true;
}
