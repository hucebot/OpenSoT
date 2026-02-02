#ifndef __OPENSOT_OC_CONTACT_H__
#define __OPENSOT_OC_CONTACT_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/oc/TorquesTask.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT::oc {


class ContactConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef std::shared_ptr<ContactConstraint> Ptr;

    private:
        XBot::ModelInterface& _robot;

        AffineHelper _dU;
        AffineHelper _dX;

        const std::string _frame_name;
        Eigen::Affine3d _wTd;

        Eigen::MatrixXd _dvc_dq;
        Eigen::MatrixXd _dvc_dv;

        Eigen::MatrixXd _J;
        Eigen::Matrix6d _Adj;
        Eigen::VectorXd _b;

        Eigen::VectorXd _active;
        double _ground_height;
    

        void _update();
    
    public:
     ContactConstraint(XBot::ModelInterface& robot,
                       const std::string& frame_name, const AffineHelper& dX);

     void activate(double ground_height);
     void deactivate();


    };
}

#endif
