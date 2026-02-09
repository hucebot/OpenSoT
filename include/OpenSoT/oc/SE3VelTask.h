#ifndef __OPENSOT_SE3VEL_TASK_OC_H__
#define __OPENSOT_SE3VEL_TASK_OC_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/LieGroupsUtils.h>


namespace OpenSoT::oc{


class SE3VelTask : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{

    public:
        typedef std::shared_ptr<SE3VelTask> Ptr;


        SE3VelTask(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame);

        void setReferenceVelocity(const Eigen::Vector6d v)
        {
            _ref = v;
        }

    private:
        const XBot::ModelInterface& _robot;
        AffineHelper _dx;

        Eigen::MatrixXd _dV_dq;
        Eigen::MatrixXd _dV_dv;


        std::string _distal_frame;
        Eigen::Vector6d _ref;


        virtual void _update();

};
}




#endif
