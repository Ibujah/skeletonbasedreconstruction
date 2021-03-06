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
 *  \file Tracker.h
 *  \brief Provides methods to acquire calibrated images
 *  \author Bastien Durix
 */

#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <AR/ar.h>
#include <AR/arMulti.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <list>
#include <map>

#include <camera/Camera.h>

/**
 *  \brief Tracking methods
 */
namespace tracking
{
	/**
	 *  \brief Tracker class to estimate the position of the camera
	 */
	class Tracker
	{
		protected:
			/**
 			 *  \brief main ARToolKit structure, storing informations about camera, patterns, etc...
 			 */
			ARHandle *m_arhandle;
			
			/**
 			 *  \brief ARToolKit structure containing camera calibration
 			 */
			ARParamLT *m_arparamlt;

			/**
			 *  \brief ARToolKit structure for 3D informations
			 */
			AR3DHandle *m_ar3dhandle;

			/**
			 *  \brief ARToolKit structure to handle multiple 3D markers
			 */
			ARMultiMarkerInfoT *m_armulti;
			
			/**
			 *  \brief OpenCV calibration matrix
			 */
			cv::Mat m_matK;
			
			/**
			 *  \brief OpenCV distortion coefficients
			 */
			cv::Mat m_distCoeff;

			/**
			 *  \brief Number of detected markers
			 */
			int m_nbmarkers;

			/**
			 *  \brief Markers size
			 */
			double m_markersize;

			/**
			 *  \brief Estimated marker poses at each acquired image
			 */
			std::map<int, std::list<cv::Mat> > m_savedposes;

			/**
			 *  \brief Storage of marker global id
			 */
			std::map<int,int> m_globalid;

			/**
			 *  \brief matries containing informations about current position, to optimize the tracking
			 */
			cv::Mat rvec, tvec;
		public:
			/**
 			 *  \brief Constructor
 			 */
			Tracker(unsigned int nbmarkers = 6, double markersize = 4.0);
			
			/**
 			 *  \brief Destructor
 			 */
			~Tracker();
			
			/**
 			 *  \brief Method initialising the tracker
 			 *
 			 *  \param cam  Camera acquiring the scene
 			 */
			void init(const camera::Camera::Ptr cam);
			
			/**
 			 *  \brief Cleaning method
 			 */
			void clean();

			/**
 			 *  \brief Detects patterns from an image
			 *
			 *  \param img  Image where detect the patterns (out : draws the detected patterns)
			 *
			 *  \return number of detected patterns
 			 */
			int detect(cv::Mat &img);

			/**
			 *  \brief Adds current detection to the set of detection, which is used to compute the relative position of the markers
			 *
			 *  \return false if there is not enough detected markers
			 */
			bool addCurrDetection();

			/**
			 *  \brief Computes relative pose of markers 
			 *  \details Needs at least 3 viewpoints, added with addCurrDetection()
			 *
			 *  \return true if correctly computed
			 */
			bool computeMulti();

			/**
 			 *  \brief Gets current transformation matrix
 			 *  \details Needs relative pose of markers computed, with computeMulti()
 			 *
 			 *  \param matTr  Current transformation matrix
			 *
			 *  \return true if the matrix was correctly computed
 			 */
			 bool getCurrTr(Eigen::Matrix<double,3,4> &matTr);
	};
}

#endif //_TRACKER_H_
