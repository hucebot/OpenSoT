#pragma once

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT::oc {

    class FrictionConeConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef std::shared_ptr<FrictionConeConstraint> Ptr;

    private:
        XBot::ModelInterface& _robot;

        std::string _distal_frame;
        std::shared_ptr<VariableXd> _force_var;
        Eigen::Affine3d _d_T_w;

        Eigen::MatrixXd _J;
        Eigen::MatrixXd __A;
        Eigen::Matrix6d _Adj;

        

        Eigen::Affine3d _error;

        Eigen::Vector3d _w_force;

        Eigen::Matrix6d _J_l6_inv;

        void _update();
    
    public:
        FrictionConeConstraint(XBot::ModelInterface &robot,
                            const std::string& frame_name,
                            const std::shared_ptr<VariableXd> force,
                            const AffineHelper &dX,
                            const AffineHelper &dU);


    };
} 