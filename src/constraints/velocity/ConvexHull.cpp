/*
 * Copyright (C) 2026 MeRLin
 * Author: Enrico Mingo Hoffman
 */

#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <cmath>
#include <algorithm>

using namespace OpenSoT::constraints::velocity;

/* ============================ Utility ============================ */

namespace {

/* 2D cross product (O->A) x (O->B) */
inline double cross2D(const Eigen::Vector2d& O,
                      const Eigen::Vector2d& A,
                      const Eigen::Vector2d& B)
{
    return (A.x() - O.x()) * (B.y() - O.y()) -
           (A.y() - O.y()) * (B.x() - O.x());
}

/* Andrew’s monotone chain (CCW order) */
std::vector<Eigen::Vector3d>
computeConvexHull3D(const std::vector<Eigen::Vector3d>& pts3)
{
    std::vector<Eigen::Vector3d> pts = pts3;

    if(pts.size() <= 2)
        return pts;

    /* Sort lexicographically by (x,y) */
    std::sort(pts.begin(), pts.end(),
              [](const Eigen::Vector3d& a, const Eigen::Vector3d& b)
              {
                  if(a.x() < b.x()) return true;
                  if(a.x() > b.x()) return false;
                  return a.y() < b.y();
              });

    std::vector<Eigen::Vector3d> H(2 * pts.size());
    size_t k = 0;

    /* Lower hull */
    for(size_t i = 0; i < pts.size(); ++i)
    {
        while(k >= 2)
        {
            Eigen::Vector2d O = H[k-2].head<2>();
            Eigen::Vector2d A = H[k-1].head<2>();
            Eigen::Vector2d B = pts[i].head<2>();

            if(cross2D(O, A, B) > 0.0) break;
            k--;
        }
        H[k++] = pts[i];
    }

    /* Upper hull */
    for(int i = pts.size() - 2, t = k + 1; i >= 0; --i)
    {
        while(k >= t)
        {
            Eigen::Vector2d O = H[k-2].head<2>();
            Eigen::Vector2d A = H[k-1].head<2>();
            Eigen::Vector2d B = pts[i].head<2>();

            if(cross2D(O, A, B) > 0.0) break;
            k--;
        }
        H[k++] = pts[i];
    }

    H.resize(k - 1);
    return H;
}

} // anonymous namespace

/* ============================ Class ============================ */

ConvexHull::ConvexHull(XBot::ModelInterface& robot,
                       const std::list<std::string>& links_in_contact,
                       const double safetyMargin) :
    Constraint("convex_hull", robot.getNv()),
    _links_in_contact(links_in_contact),
    _robot(robot),
    _boundScaling(safetyMargin),
    _JCoM(3, _x_size),
    _C(links_in_contact.size(), 2)
{
    _bUpperBound.resize(links_in_contact.size());
    _bLowerBound.resize(links_in_contact.size());
    _bLowerBound = -1.0e20 *
                   Eigen::VectorXd::Ones(_bUpperBound.size());

    this->update();
}

void ConvexHull::_update()
{
    _robot.getCOMJacobian(_JCoM);

    if(getConvexHull(_ch))
        getConstraints(_ch, _C, _bUpperBound, _boundScaling);

    _Aineq = _C * _JCoM.block(0,0,2,_x_size);
}

/* ================= Convex Hull Computation ================= */

bool ConvexHull::getConvexHull(std::vector<Eigen::Vector3d>& ch)
{
    std::vector<Eigen::Vector3d> support_pts;

    /* Get support points in COM frame */
    Eigen::Vector3d world_com = _robot.getCOM();

    for(const auto& link : _links_in_contact)
    {
        Eigen::Affine3d world_T_link = _robot.getPose(link);
        Eigen::Vector3d p_world = world_T_link.translation();
        Eigen::Vector3d p_com   = p_world - world_com;
        support_pts.push_back(p_com);
    }

    if(support_pts.size() < 3)
        return false;

    ch = computeConvexHull3D(support_pts);
    return true;
}

/* ================= Constraint Construction ================= */

void ConvexHull::getConstraints(const std::vector<Eigen::Vector3d>& convex_hull,
                                Eigen::MatrixXd& A,
                                Eigen::VectorXd& b,
                                const double boundScaling)
{
    A.setZero(A.rows(), A.cols());
    b = 1.0e10 * Eigen::VectorXd::Ones(b.size());

    for(size_t j = 0; j < convex_hull.size(); ++j)
    {
        size_t k = (j + 1) % convex_hull.size();

        double a, bb, c;
        getLineCoefficients(convex_hull[j], convex_hull[k], a, bb, c);

        if(c <= 0.0)
        {
            A(j,0) = +a;
            A(j,1) = +bb;
            b(j)   = -c;
        }
        else
        {
            A(j,0) = -a;
            A(j,1) = -bb;
            b(j)   = +c;
        }

        double normalizedBoundScaling =
            boundScaling * std::sqrt(a*a + bb*bb);

        b(j) -= normalizedBoundScaling;
    }
}

void ConvexHull::getLineCoefficients(const Eigen::Vector3d& p0,
                                     const Eigen::Vector3d& p1,
                                     double& a,
                                     double& b,
                                     double& c)
{
    double x1 = p0.x();
    double x2 = p1.x();
    double y1 = p0.y();
    double y2 = p1.y();

    a = y1 - y2;
    b = x2 - x1;
    c = -b*y1 - a*x1;
}

void ConvexHull::setSafetyMargin(const double safetyMargin)
{
    _boundScaling = safetyMargin;
}
