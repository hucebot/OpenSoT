#ifndef __OPENSOT_OC_CONTACT_H__
#define __OPENSOT_OC_CONTACT_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/oc/TorquesTask.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT::oc {

    class ContactTask : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
    public:
        typedef std::shared_ptr<ContactTask> Ptr;

        ContactTask(XBot::ModelInterface& robot, const std::string& frame_name, const AffineHelper& dX, const AffineHelper& dU);

    private:
        XBot::ModelInterface& _robot;
        AffineHelper _dU;
        AffineHelper _dX;

        Eigen::MatrixXd _dvc_dq;
        Eigen::MatrixXd _dvc_dv;

        //AffineHelper _dcontact;
        Eigen::MatrixXd _Fx;
        Eigen::MatrixXd _Fu;

        const std::string _frame_name;

        virtual void _update();
        
    };




    class ContactConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef std::shared_ptr<ContactConstraint> Ptr;

    private:
        XBot::ModelInterface& _robot;

        ContactTask _task;

        void _update();
    
    public:
        ContactConstraint(XBot::ModelInterface &robot,
                            const std::string& frame_name,
                            const AffineHelper &dX,
                            const AffineHelper &dU);


    };
}

#endif
