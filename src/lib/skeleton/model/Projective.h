/*
Copyright (c) 2016 Bastien Durix

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/**
 *  \file Projective.h
 *  \brief Defines skeleton projective model
 *  \author Bastien Durix
 */

#ifndef _PROJECTIVEMODEL_H_
#define _PROJECTIVEMODEL_H_

#include <memory>
#include "MetaModel.h"
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>
#include <mathtools/geometry/euclidian/HyperSphere.h>

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Skeletal models
	 */
	namespace model
	{
		class Projective;
		
		/**
		 *  \brief Describe Projective model meta data
		 */
		template<>
		struct meta<Projective>
		{
			/**
			 *  \brief Describe storage dimension of the model
			 */
			static constexpr unsigned int stordim = 3;
		};

		/**
		 *  \brief Describe projective skeleton model
		 */
		class Projective
		{
			public:
				/**
				 *  \brief Shared pointer to the model
				 */
				using Ptr = std::shared_ptr<Projective>;
				
				/**
				 *  \brief Projective skeleton type
				 */
				enum class Type
				{
					perspective,
					orthographic
				};

			protected:
				/**
				 *  \brief Frame of the skeleton
				 */
				mathtools::affine::Frame<3>::Ptr m_frame;

			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param frame skeleton frame
				 */
				Projective(const mathtools::affine::Frame<3>::Ptr frame = mathtools::affine::Frame<3>::CanonicFrame());

				/**
				 *  \brief Copy constructor
				 *
				 *  \param model model to copy
				 */
				Projective(const Projective &model);

				/**
				 *  \brief Frame getter
				 *
				 *  \return skeleton frame
				 */
				const mathtools::affine::Frame<3>::Ptr getFrame() const;

				/**
				 *  \brief Converts an object into a vector
				 *
				 *  \tparam TypeObj type of the object to convert
				 *
				 *  \param obj object to convert
				 *
				 *  \return vector associated to obj
				 */
				template<typename TypeObj>
				Eigen::Matrix<double,meta<Projective>::stordim,1> toVec(const TypeObj &obj) const
				{
					return toVec(obj);
				}

				/**
				 *  \brief Converts a vector into an object
				 *
				 *  \tparam TypeObj type of the object to convert in
				 *
				 *  \param vec vector to convert
				 *
				 *  \return object associated to vec
				 */
				template<typename TypeObj>
				TypeObj toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec) const
				{
					return toObj(vec,TypeObj{});
				}
				
				/**
				 *  \brief Skeleton type getter
				 *
				 *  \return Skeleton type
				 */
				virtual Type getType() const = 0;

				/**
				 *  \brief Size getter (used in nodes comparison)
				 *
				 *  \param vec voctor to evaluate the size
				 *
				 *  \return size associated to vec
				 */
				virtual double getSize(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec) const = 0;

			protected:
				/**
				 *  \brief Conversion function from hypersphere to vector
				 *
				 *  \param sph hypersphere to convert
				 *
				 *  \return vector associated to hyperpshere
				 */
				virtual Eigen::Matrix<double,meta<Projective>::stordim,1> toVec(const mathtools::geometry::euclidian::HyperSphere<2> &sph) const;

				/**
				 *  \brief Associate the center of the sphere to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return center associated to vec
				 */
				virtual mathtools::affine::Point<2> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
														  const mathtools::affine::Point<2> &) const;

				/**
				 *  \brief Associate an hypersphere to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return hypersphere
				 */
				virtual mathtools::geometry::euclidian::HyperSphere<2> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
																			 const mathtools::geometry::euclidian::HyperSphere<2> &) const;

		};
	}
}


#endif //_PROJECTIVEMODEL_H_
