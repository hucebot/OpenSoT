/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi,
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <OpenSoT/utils/cartesian_utils.h>
#include <cmath>
#define toDeg(X) (X*180.0/M_PI)


// void  cartesian_utils::computePanTiltMatrix(const Eigen::VectorXd &gaze, KDL::Frame &pan_tilt_matrix)
// {
//     double pan = std::atan2(gaze[1], gaze[0]);
//     double tilt = std::atan2(gaze[2],sqrt(gaze[1]*gaze[1] + gaze[0]*gaze[0]));

//     pan_tilt_matrix.Identity();
//     pan_tilt_matrix.M.DoRotZ(pan);
//     pan_tilt_matrix.M.DoRotY(-tilt);
// }

void cartesian_utils::computePanTiltMatrix(const Eigen::VectorXd &gaze,
                                           Eigen::Affine3d &pan_tilt_matrix)
{
    double pan  = std::atan2(gaze[1], gaze[0]);
    double tilt = std::atan2(gaze[2], std::sqrt(gaze[1]*gaze[1] + gaze[0]*gaze[0]));

    pan_tilt_matrix.setIdentity();

    Eigen::AngleAxisd Rz(pan,  Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(-tilt, Eigen::Vector3d::UnitY());

    pan_tilt_matrix.linear() =
        (Rz * Ry).toRotationMatrix();
}


void cartesian_utils::computeCartesianError(const Eigen::Affine3d &T,
                                  const Eigen::Affine3d &Td,
                                  Eigen::Vector3d& position_error,
                                  Eigen::Vector3d& orientation_error)
{
    Eigen::Quaterniond q(T.rotation());
    Eigen::Quaterniond qd(Td.rotation());

    position_error = Td.translation()- T.translation();

    //This is needed to move along the short path in the quaternion error
    if(q.dot(qd) < 0.0)
        orientation_error = quaternion::error(-q.x(), -q.y(), -q.z(), -q.w(),
                                              qd.x(), qd.y(), qd.z(), qd.w());
    else
        orientation_error = quaternion::error(q.x(),  q.y(),  q.z(),  q.w(),
                                              qd.x(), qd.y(), qd.z(), qd.w());
}


