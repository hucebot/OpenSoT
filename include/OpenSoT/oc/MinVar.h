#ifndef __OPENSOT_OC_MINVAR_H__
#define __OPENSOT_OC_MINVAR_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT::oc {

class MinVar : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> {
   public:
    typedef std::shared_ptr<MinVar> Ptr;

    MinVar(const std::string& id, const AffineHelper& dx,
           std::shared_ptr<AffineHelper> x);

    void setReference(const Eigen::VectorXd& ref) { _ref = ref; }

   private:
    AffineHelper _dx;
    std::shared_ptr<AffineHelper> _x;

    Eigen::VectorXd _ref;

    virtual void _update();
};
}  // namespace OpenSoT::oc

#endif
