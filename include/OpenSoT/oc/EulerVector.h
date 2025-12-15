#ifndef __OPENSOT_EULER_VECTOR_H__
#define __OPENSOT_EULER_VECTOR_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/LieGroupsUtils.h>
// #include 

namespace OpenSoT { namespace oc {
class EulerVector : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
public:
    typedef std::shared_ptr<EulerVector> Ptr;

    EulerVector(const XBot::ModelInterface& robot,
                               const AffineHelper& dX,
                               const AffineHelper& dU,
                               std::shared_ptr<AffineHelper> Xk,
                               std::shared_ptr<AffineHelper> Uk,
                               std::shared_ptr<AffineHelper> Xk_1,
                               const double dt);


private:
    const XBot::ModelInterface& _robot;
    AffineHelper _dU;
    AffineHelper _dX;
    std::shared_ptr<AffineHelper> _Xk;
    std::shared_ptr<AffineHelper> _Uk;
    std::shared_ptr<AffineHelper> _Xk_1;
    double _dt;

    AffineHelper _dXnext;

    Eigen::MatrixXd _Fx;
    Eigen::MatrixXd _Fu;



    virtual void _update();
    
};


}
}


#endif
