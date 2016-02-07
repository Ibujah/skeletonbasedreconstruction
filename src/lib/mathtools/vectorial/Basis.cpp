#include "Basis.h"

template<>
mathtools::vectorial::Basis<2>::Ptr mathtools::vectorial::Basis<2>::canonicbasis =
										mathtools::vectorial::Basis<2>::CreateBasis(Eigen::Matrix2d::Identity());

template<>
mathtools::vectorial::Basis<3>::Ptr mathtools::vectorial::Basis<3>::canonicbasis =
										mathtools::vectorial::Basis<3>::CreateBasis(Eigen::Matrix3d::Identity());

template<>
mathtools::vectorial::Basis<4>::Ptr mathtools::vectorial::Basis<4>::canonicbasis =
										mathtools::vectorial::Basis<4>::CreateBasis(Eigen::Matrix4d::Identity());
