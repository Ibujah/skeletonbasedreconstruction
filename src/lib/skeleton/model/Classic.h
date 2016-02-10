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
 *  \file Classic.h
 *  \brief Defines skeleton classical model
 *  \author Bastien Durix
 */

#ifndef _CLASSICMODEL_H_
#define _CLASSICMODEL_H_

#include <memory>
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>
#include <mathtools/geometry/euclidian/HyperSphere.h>
#include "MetaModel.h"

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
		/**
		 *  \brief Describe classical skeleton model
		 *
		 *  \tparam Dim : Dimension of the hyperspheres composing the skeleton
		 */
		template<unsigned int Dim>
		class Classic
		{
			public:
				/**
				 *  \brief Shared pointer to the model
				 */
				using Ptr = std::shared_ptr<Classic<Dim> >;

			protected:
				/**
				 *  \brief Frame of the skeleton
				 */
				typename mathtools::affine::Frame<Dim>::Ptr m_frame;

			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param frame frame of the skeleton
				 */
				Classic(const typename mathtools::affine::Frame<Dim>::Ptr frame = mathtools::affine::Frame<2>::CanonicFrame()) : m_frame(frame) {}
				
				/**
				 *  \brief Frame getter
				 *
				 *  \return frame of the model
				 */
				const typename mathtools::affine::Frame<Dim>::Ptr getFrame() const
				{
					return m_frame;
				}

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
				Eigen::Matrix<double,meta<Classic>::stordim,1> toVec(const TypeObj &obj)
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
				TypeObj toObj(const Eigen::Matrix<double,meta<Classic>::stordim,1> &vec)
				{
					return toObj(vec,TypeObj{});
				}

			protected:
				/**
				 *  \brief Conversion function from hypersphere to vector
				 *
				 *  \param sph hypersphere to convert
				 *
				 *  \return vector associated to hyperpshere
				 */
				Eigen::Matrix<double,meta<Classic>::stordim,1> toVec(const mathtools::geometry::euclidian::HyperSphere<Dim> &sph)
				{
					if(!sph.getFrame()->getBasis()->getMatrix().isApprox(m_frame->getBasis()->getMatrix(),std::numeric_limits<double>::epsilon()))
						throw std::logic_error("skeleton::model::Classic::toVec: hypersphere coordinates cannot be expressed in this model");
					Eigen::Matrix<double,meta<Classic>::stordim,1> vec;
					vec.template block<Dim,1>(0,0) = sph.getCenter().getCoords(m_frame);
					vec(Dim,0) = sph.getRadius();
					return vec;
				}

				/**
				 *  \brief Associate the center of the sphere to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return center associated to vec
				 */
				inline mathtools::affine::Point<Dim> toObj(const Eigen::Matrix<double,meta<Classic>::stordim,1> &vec,
														   const mathtools::affine::Point<Dim> &)
				{
					return mathtools::affine::Point<Dim>(vec.template block<Dim,1>(0,0),m_frame);
				}

				/**
				 *  \brief Associate an hypersphere to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return hypersphere
				 */
				inline mathtools::geometry::euclidian::HyperSphere<Dim> toObj(const Eigen::Matrix<double,meta<Classic>::stordim,1> &vec,
																			  const mathtools::geometry::euclidian::HyperSphere<Dim> &)
				{
					return mathtools::geometry::euclidian::HyperSphere<Dim>(
								mathtools::affine::Point<Dim>(vec.template block<Dim,1>(0,0),m_frame),
								vec(Dim,0),
								m_frame);
				}
		};
		
		/**
		 *  \brief Describe Classic model meta data
		 *
		 *  \tparam Dim : Dimension of the hyperspheres composing the skeleton
		 */
		template<unsigned int Dim>
		struct meta<Classic<Dim> >
		{
			/**
			 *  \brief Describe storage dimension of the model
			 */
			static constexpr unsigned int stordim = Dim+1;
		};
	}
}


#endif //_CLASSICMODEL_H_
