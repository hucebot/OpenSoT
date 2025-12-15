#ifndef __OPENSOT_SE3_TASK_OC_H__
#define __OPENSOT_SE3_TASK_OC_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/LieGroupsUtils.h>
// #include 


namespace OpenSoT::oc{


class SE3Task : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>{

    public:
        typedef std::shared_ptr<SE3Task> Ptr;

        enum class ReferenceFrame{
            LOCAL,
            WORLD
        };

        SE3Task(const std::string& id, const XBot::ModelInterface& robot, const AffineHelper& dx, const std::string& distal_frame, const ReferenceFrame reference_frame=ReferenceFrame::LOCAL);

        /*
        @param T: reference in world frame
        */
        void setReference(const Eigen::Affine3d& T)
        {
            _ref = T;
        }

        const Eigen::Affine3d& getReference() const
        {
            return _ref;
        }

        const std::string& getDistalFrame() const
        {
            return _distal_frame;
        }

        const Eigen::Vector6d& getError();

        const ReferenceFrame& getReferenceFrame() const
        {
            return _reference_frame;
        }

        void setReferenceFrame(const ReferenceFrame& reference_frame)
        {
            _reference_frame = reference_frame;
        }

        const Eigen::MatrixXd& getFrameJacobian() const
        {
            return _J;
        }

        const Eigen::Affine3d& getSE3Error() const
        {
            return _error;
        }

        const Eigen::Vector6d& getse3Error() const
        {
            return _w;
        }

    private:
        inline void adjoint(const Eigen::Affine3d& T, Eigen::Matrix6d& Adj) {
            Adj.setZero();

            Adj.setZero();
            Adj.topLeftCorner<3,3>() = T.linear();
            Adj.topRightCorner<3,3>() = OpenSoT::hat(T.translation()) * T.linear();
            Adj.bottomRightCorner<3,3>() =  T.linear();
        }

        const XBot::ModelInterface& _robot;
        AffineHelper _dx;
        //AffineHelper _task;

        std::string _distal_frame;

        Eigen::Affine3d _ref;
        Eigen::Affine3d _d_T_w;

        Eigen::MatrixXd _J;
        Eigen::MatrixXd __A;
        Eigen::Matrix6d _Adj;

        Eigen::Affine3d _error;

        Eigen::Vector6d _w;

        ReferenceFrame _reference_frame;

        Eigen::Matrix6d _J_l6_inv;

        virtual void _update();

};
}





#endif
