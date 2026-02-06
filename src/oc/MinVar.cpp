#include <OpenSoT/oc/MinVar.h>

using namespace OpenSoT::oc;

MinVar::MinVar(const std::string& id, const AffineHelper& dx,
               std::shared_ptr<AffineHelper> x)
    : Task<Eigen::MatrixXd, Eigen::VectorXd>("MinVar", dx.getInputSize()),
      _dx(dx),
      _x(x) {

    _ref.setZero(_x->getOutputSize());

    // _task = _dx + (_x->getValue()-_ref);
    _A = _dx.getM();

    _W.setIdentity(_dx.getOutputSize(), _dx.getOutputSize());

    update();
}

void MinVar::_update() {
    _x->update();
    _b = -(_x->getValue() - _ref);
}
