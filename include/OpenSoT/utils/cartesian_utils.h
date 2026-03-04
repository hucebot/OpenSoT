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

#ifndef _CARTESIAN_UTILS_H__
#define _CARTESIAN_UTILS_H__

#include <limits>
#include <Eigen/Dense>

/**
 * @brief The CostFunction class pure virtual function used to describe functions for computeGradient method.
 */
class CostFunction {
public:
    /**
     * @brief compute value of function in x
     * @param x
     * @return scalar
     */

    virtual double compute(const Eigen::VectorXd &x) = 0;
};


/**
  This struct implements quaternion error as in the paper:
    "Operational Space Control: A Theoretical and Empirical Comparison"
  Authors: Jun Nakanishi, Rick Cory, Michael Mistry, Jan Peters and Stefan Schaal
  The International Journal of Robotics Research, Vol. 27, No. 6, June 2008, pp. 737–757

  REMEMBER: if e is the quaternion error, the orientation error is defined as:
                o_error = -Ke
            with K positive definite!
  **/
struct quaternion
{
    static Eigen::Vector3d error(const double& qx,const double& qy,const double& qz,const double& qw,
                                 const double& qdx,const double& qdy,const double& qdz,const double& qdw)
    {
        Eigen::Vector3d e(0.0, 0.0, 0.0);

        Eigen::Vector3d eps(qx, qy, qz);
        Eigen::Vector3d epsd(qdx, qdy, qdz);

        Eigen::Matrix3d skew;
        skew<<  0.0,  -qdz,  qdy,
             qdz,   0.0, -qdx,
            -qdy, qdx,    0.0;

        e = qdw*eps - qw*epsd + skew*eps;

        return e;
    }
};

/**
 * LDLTInverse provide a simple template Eigen-based (implemented using LDLT) class to compute Inverse
 * NOTE: FAST, works with squared Positive/Negative-SemiDefinite matrices, RT-safe
 */
template<class _Matrix_Type_> class LDLTInverse
{
public:
    LDLTInverse(const _Matrix_Type_ &a)
    {
        I.resize(a.rows(), a.cols()); I.setIdentity(I.rows(), I.cols());
    }
    
    void compute(const _Matrix_Type_ &a, _Matrix_Type_ &ainv)
    {
        LDLT.compute(a);
        ainv = LDLT.solve(I);
    }
    
private:
    Eigen::LDLT<_Matrix_Type_> LDLT;
    _Matrix_Type_ I;
};



/**
 * SVDPseudoInverse provide a simple template Eigen-based (implemented using SVD) class to compute pseudo inverse
 * NOTE: SLOW, works with any matrix, NON RT-safe
 */
template<class _Matrix_Type_> class SVDPseudoInverse
{
public:
    SVDPseudoInverse(const _Matrix_Type_ &a, const double epsilon = std::numeric_limits<double>::epsilon()):
        _epsilon(epsilon),
        _svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV)
    {

    }

    void compute(const _Matrix_Type_ &a, _Matrix_Type_ &ainv, const double epsilon = std::numeric_limits<double>::epsilon())
    {
        _svd.compute(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        _singularValues = _svd.singularValues().array().abs();
        
        _tolerance = epsilon * std::max(a.cols(), a.rows()) *_singularValues(0);
        
        for(unsigned int i = 0; i < _singularValues.size(); ++i)
        {
            if(_singularValues[i] < _tolerance)
                _singularValues[i] = 0.0;
            else
                _singularValues[i] = 1./_singularValues[i];
        }
        
        _tmp = _svd.matrixV() *  _singularValues.asDiagonal();
        ainv.noalias() =  _tmp * _svd.matrixU().transpose();
    }

    void setEpsilon(const double epsilon)
    {
        _epsilon = epsilon;
    }

private:
    double _epsilon;
    Eigen::JacobiSVD<_Matrix_Type_> _svd;
    double _tolerance;
    
    Eigen::VectorXd _singularValues;
    Eigen::MatrixXd _tmp;
};

class cartesian_utils
{
public:
    /**
     * @brief computePanTiltMatrix given a gaze vector computes the Homogeneous Matrix to control the
     * YAW-PITCH angles.
     * The algorithm used is based on the paper: "Adaptive Predictive Gaze Control of a Redundant Humanoid
     * Robot Head, IROS2011".
     *
     * @param gaze vector [3x1]
     * @param pan_tilt_matrix Homogeneous Matrix [4x4] in the same reference frame of the gaze vector
     */
    static void computePanTiltMatrix(const Eigen::VectorXd& gaze, Eigen::Affine3d& pan_tilt_matrix);



    /**
     * @brief computeCartesianError orientation and position error
     * @param T actual pose
     * @param Td desired pose
     * @param position_error position error
     * @param orientation_error orientation error
     */
    static void computeCartesianError(const Eigen::Affine3d &T,
                                      const Eigen::Affine3d &Td,
                                      Eigen::Vector3d& position_error,
                                      Eigen::Vector3d& orientation_error);
};


#endif
