#ifndef __OPENSOT_POSSO3_TASK_OC_H__
#define __OPENSOT_POSSO3_TASK_OC_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/LieGroupsUtils.h>


namespace OpenSoT::oc{


class PosSO3Task : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{

    public:
        typedef std::shared_ptr<PosSO3Task> Ptr;


        PosSO3Task(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame);

        void setReference(const Eigen::Affine3d& T)
        {
            _ref = T;
        }

        const Eigen::Affine3d& getReference()
        {
            return _ref;
        }

    private:
        const XBot::ModelInterface& _robot;
        AffineHelper _dx;

        Eigen::Affine3d _wTd;

        Eigen::MatrixXd _J;

        Eigen::MatrixXd _dT_dq;
        // Eigen::MatrixXd _dV_dv;


        std::string _distal_frame;
        Eigen::Affine3d _ref;
        Eigen::Vector6d _error;


        virtual void _update();

};
}




#endif
