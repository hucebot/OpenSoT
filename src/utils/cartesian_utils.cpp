/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi,
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <OpenSoT/utils/cartesian_utils.h>
#include <memory>
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


