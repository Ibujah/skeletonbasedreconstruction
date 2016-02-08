#include "Frame.h"

template<>
mathtools::affine::Frame<2>::Ptr mathtools::affine::Frame<2>::canonicframe = 
									mathtools::affine::Frame<2>::CreateFrame(Eigen::Vector2d::Zero(),mathtools::vectorial::Basis<2>::CanonicBasis());

template<>
mathtools::affine::Frame<3>::Ptr mathtools::affine::Frame<3>::canonicframe = 
									mathtools::affine::Frame<3>::CreateFrame(Eigen::Vector3d::Zero(),mathtools::vectorial::Basis<3>::CanonicBasis());

template<>
mathtools::affine::Frame<4>::Ptr mathtools::affine::Frame<4>::canonicframe = 
									mathtools::affine::Frame<4>::CreateFrame(Eigen::Vector4d::Zero(),mathtools::vectorial::Basis<4>::CanonicBasis());

