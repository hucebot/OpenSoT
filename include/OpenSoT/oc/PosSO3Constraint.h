#ifndef __OPENSOT_POSSO3_CONSTRAINT_OC_H__
#define __OPENSOT_POSSO3_CONSTRAINT_OC_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/oc/PosSO3Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT::oc
{

    class PosSO3Constraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
    {

    public:
        typedef std::shared_ptr<PosSO3Constraint> Ptr;

        PosSO3Constraint(const XBot::ModelInterface &robot,
                         const AffineHelper &dX,
                         const std::string &distal_frame);

        void setUpperLimits(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rotation);
        void setLowerLimits(const Eigen::Vector3d &pos, const Eigen::Matrix3d &rotation);

    private:
        const XBot::ModelInterface &_robot;
        AffineHelper _dX;
        std::string _distal_frame;

        PosSO3Task _task;

        Eigen::Vector6d _upper_lim;
        Eigen::Vector6d _lower_lim;

        virtual void _update();
    };
}

#endif
