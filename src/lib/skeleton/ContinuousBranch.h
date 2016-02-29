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
 *  \file ContinuousBranch.h
 *  \brief Defines continuous skeletal branch
 *  \author Bastien Durix
 */

#ifndef _CONTINUOUSBRANCH_H_
#define _CONTINUOUSBRANCH_H_

#include <memory>
#include <Eigen/Dense>
#include "model/MetaModel.h"
#include <mathtools/application/Application.h>
#include <mathtools/application/Compositor.h>

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Describes continuous skeletal branch
	 *
	 *  \tparam Model Class giving a meaning to the skeleton (dimensions, geometric interpretation...)
	 */
	template<typename Model>
	class ContinuousBranch
	{
		public:
			/**
			 *  \brief Graph skeletal branch shared pointer
			 */
			using Ptr = std::shared_ptr<ContinuousBranch<Model> >;
			
			/**
			 *  \brief Storage type of the objects in the skeleton
			 */
			using Stor = Eigen::Matrix<double,model::meta<Model>::stordim,1>;

			/*
			 *  \brief Node function type
			 */
			using NodeFun = mathtools::application::Compositor<mathtools::application::Application<Stor,double>,
															   mathtools::application::Application<double,double> >;
			
		protected:
			/**
			 *  \brief Model used to give a meaning to the skeleton
			 *
			 *  \details This class will not be modified inside the skeleton:
			 *			 once it is created, it should not be changed
			 */
			typename Model::Ptr m_model;
			
			/**
 			 *  \brief Function giving nodes in the branch
 			 */
			std::shared_ptr<NodeFun> m_nodefun;
			
		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param model initialisation of the model to use
			 *  \param nodefun node function
			 */
			ContinuousBranch(const typename Model::Ptr model, const std::shared_ptr<NodeFun> nodefun) :
				m_model(model), m_nodefun(new std::vector<Stor>(nodefun)) {}
			
			/**
			 *  \brief Constructor
			 *
			 *  \param model   initialisation of the model to use
			 *  \param nodefun node function
			 */
			ContinuousBranch(const Model &model, const NodeFun &nodefun) :
				ContinuousBranch(typename Model::Ptr(new Model(model)),std::shared_ptr<NodeFun>(new NodeFun(nodefun))) {}
			
			/**
			 *  \brief Copy constructor
			 *
			 *  \param grbr skeleton to copy
			 */
			ContinuousBranch(const ContinuousBranch<Model> &grcont) :
				m_model(grcont.m_model), m_nodefun(grcont.m_nodefun) {}
			
			/**
			 *  \brief Model getter
			 *
			 *  \return Const shared pointer to skeleton model
			 */
			inline const typename Model::Ptr getModel() const
			{
				return m_model;
			}
			
			/**
			 *  \brief Node getter by parameter
			 *
			 *  \param t parameter of the node to get
			 *
			 *  \return storage associated to the node
			 */
			const Stor& getNode(double t) const
			{
				return (*m_nodefun)(t);
			}
			
			/**
			 *  \brief Node getter by parameter
			 *
			 *  \tparam TypeNode type of the node to get
			 *
			 *  \param t parameter of the node to get
			 *
			 *  \return node associated to index
			 */
			template<typename TypeNode>
			const TypeNode getNode(double t) const
			{
				return m_model->template toObj<TypeNode>(getNode(t));
			}
	};
}

#endif //_CONTINUOUSBRANCH_H_
