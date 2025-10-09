//
// Created by yu on 2025/9/28.
//

#ifndef FASTLIO2_CERES_OPTIMIZATION_H
#define FASTLIO2_CERES_OPTIMIZATION_H
#include "commons.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/local_parameterization.h>

struct PointToPlaneFactor {
    PointToPlaneFactor(const Eigen::Vector3d &point,
                       const Eigen::Vector4d &plane,
                       const Eigen::Matrix3d &r_il,
                       const Eigen::Vector3d &t_il)
        : point_(point), plane_(plane), r_il_(r_il), t_il_(t_il) {}

    template <typename T>
    bool operator()(const T* const q,  // 四元数 (w,x,y,z)
                    const T* const t,  // 平移 (x,y,z)
                    T* residual) const {
        Eigen::Quaternion<T> rot(q[0], q[1], q[2], q[3]);
        Eigen::Matrix<T,3,1> trans(t[0], t[1], t[2]);
        Eigen::Matrix<T,3,1> pt(T(point_(0)), T(point_(1)), T(point_(2)));

        // body → lidar → world
        Eigen::Matrix<T,3,1> pt_l = r_il_.cast<T>() * pt + t_il_.cast<T>();
        Eigen::Matrix<T,3,1> pt_w = rot * pt_l + trans;

        T r = T(plane_(0)) * pt_w(0) +
              T(plane_(1)) * pt_w(1) +
              T(plane_(2)) * pt_w(2) +
              T(plane_(3));

        residual[0] = r;
        return true;
    }

    const Eigen::Vector3d point_;
    const Eigen::Vector4d plane_;
    const Eigen::Matrix3d r_il_;
    const Eigen::Vector3d t_il_;
};

#endif //FASTLIO2_CERES_OPTIMIZATION_H