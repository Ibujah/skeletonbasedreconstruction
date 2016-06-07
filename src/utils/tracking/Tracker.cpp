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
 *  \file Tracker.cpp
 *  \brief Provides methods to acquire calibrated images
 *  \author Bastien Durix
 */


#include "Tracker.h"
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

tracking::Tracker::Tracker(unsigned int nbmarkers, double markersize) : m_arhandle(NULL), m_arparamlt(NULL), m_armulti(NULL), m_nbmarkers(nbmarkers), m_markersize(markersize)
{
}

tracking::Tracker::~Tracker()
{
	clean();
}

void tracking::Tracker::init(const camera::Camera::Ptr cam)
{
	clean();
	ARParam arparam;
	arparam.xsize = cam->getIntrinsics()->getWidth();
	arparam.ysize = cam->getIntrinsics()->getHeight();

	arparam.mat[0][0] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(0,0);
	arparam.mat[0][1] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(0,1);
	arparam.mat[0][2] = cam->getIntrinsics()->getFrame()->getOrigin().x();
	arparam.mat[0][3] = 0.0;

	arparam.mat[1][0] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(1,0);
	arparam.mat[1][1] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(1,1);
	arparam.mat[1][2] = cam->getIntrinsics()->getFrame()->getOrigin().y();
	arparam.mat[1][3] = 0.0;

	arparam.mat[2][0] = 0.0; 
	arparam.mat[2][1] = 0.0; 
	arparam.mat[2][2] = 1.0;
	arparam.mat[2][3] = 0.0;
	
	arparam.dist_factor[0] = 0.0;
	arparam.dist_factor[1] = 0.0;
	arparam.dist_factor[2] = 0.0;
	arparam.dist_factor[3] = 0.0;
	arparam.dist_factor[4] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(0,0);
	arparam.dist_factor[5] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(1,1);
	arparam.dist_factor[6] = cam->getIntrinsics()->getFrame()->getOrigin().x();
	arparam.dist_factor[7] = cam->getIntrinsics()->getFrame()->getOrigin().y();
	arparam.dist_factor[8] = 1;
    arparam.dist_function_version = 4;

	m_arparamlt = arParamLTCreate(&arparam,AR_PARAM_LT_DEFAULT_OFFSET);
	
	m_arhandle = arCreateHandle(m_arparamlt);

	m_ar3dhandle = ar3DCreateHandle(&arparam);
	
	arSetPixelFormat(m_arhandle,AR_PIXEL_FORMAT_BGR);
	arSetPatternDetectionMode(m_arhandle,AR_MATRIX_CODE_DETECTION);
	arSetMatrixCodeType(m_arhandle,AR_MATRIX_CODE_6x6);
	arSetDebugMode(m_arhandle,AR_DEBUG_DISABLE);
}

void tracking::Tracker::clean()
{
	if(m_arhandle)
		arDeleteHandle(m_arhandle);
	m_arhandle = NULL;

	if(m_arparamlt)
    	arParamLTFree(&m_arparamlt);
	m_arparamlt = NULL;
	if(m_armulti)
	{
		delete[] m_armulti->marker;
		delete m_armulti;
		m_armulti = NULL;
	}
}

int tracking::Tracker::detect(cv::Mat &img)
{
	ARUint8 *dataPtr = img.ptr();

	arDetectMarker(m_arhandle,dataPtr);
	
	if(m_arhandle->marker_num)
	{
		for(int i = 0; i < m_arhandle->marker_num; i++)
		{
			for(int j = 0; j < 4; j++)
			{
				int ind1 = j;
				int ind2 = (ind1 + 1)%4;
				cv::Point2i pt1(m_arhandle->markerInfo[i].vertex[ind1][0],m_arhandle->markerInfo[i].vertex[ind1][1]),
							pt2(m_arhandle->markerInfo[i].vertex[ind2][0],m_arhandle->markerInfo[i].vertex[ind2][1]);
				if(m_arhandle->markerInfo[i].cf > 0.5)
					cv::line(img,pt1,pt2,cv::Scalar(0,255,0),2);
				else
					cv::line(img,pt1,pt2,cv::Scalar(0,0,255),2);
			}
		}
	}
	
	return m_arhandle->marker_num;
}

bool tracking::Tracker::addCurrDetection()
{
	std::map<int, std::vector<cv::Point2f> > curcoords;
	std::map<int, std::vector<cv::Point3f> > curcoords3d;
	
	if(m_arhandle->marker_num >= m_nbmarkers)
	{
		/*std::vector<cv::Point3f> refpts(4);
		refpts[0].x = 0;
		refpts[0].y = m_markersize;
		refpts[0].z = 0;
		refpts[1].x = m_markersize;
		refpts[1].y = m_markersize;
		refpts[1].z = 0;
		refpts[2].x = m_markersize;
		refpts[2].y = 0;
		refpts[2].z = 0;
		refpts[3].x = 0;
		refpts[3].y = 0;
		refpts[3].z = 0;
		std::vector<cv::Point2f> imgpts(4);*/
		std::cout << "Added image" << std::endl;
		for(int i = 0; i < m_arhandle->marker_num; i++)
		{
			if(m_arhandle->markerInfo[i].cf > 0.5)
			{
				ARdouble conv[3][4];
				arGetTransMatSquare( m_ar3dhandle, m_arhandle->markerInfo + i, m_markersize, conv);

				/*imgpts[0].x =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+0)%4][0];
				imgpts[0].y =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+0)%4][1];
				imgpts[1].x =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+1)%4][0];
				imgpts[1].y =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+1)%4][1];
				imgpts[2].x =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+2)%4][0];
				imgpts[2].y =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+2)%4][1];
				imgpts[3].x =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+3)%4][0];
				imgpts[3].y =m_arhandle->markerInfo[i].vertex[(4-m_arhandle->markerInfo[i].dir+3)%4][1];
				cv::Mat poseMat = cv::Mat::eye(4, 4, CV_64FC1);
				cv::Mat _matK = cv::Mat::zeros(3,3,CV_64FC1);
				_matK.at<double>(0,0) = m_arhandle->arParamLT->param.mat[0][0];
				_matK.at<double>(1,1) = m_arhandle->arParamLT->param.mat[1][1];
				_matK.at<double>(0,2) = m_arhandle->arParamLT->param.mat[0][2];
				_matK.at<double>(1,2) = m_arhandle->arParamLT->param.mat[1][2];
				cv::Mat _distCoeff = cv::Mat::zeros(1,5,CV_64FC1);
				cv::Mat rvec, tvec;
				cv::solvePnP(refpts,imgpts,_matK,_distCoeff,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
				cv::Mat rot(3,3,CV_32F);
				cv::Rodrigues(rvec,rot);

				for( int i = 0; i < 3; i++ )
				{
					for( int j = 0; j < 3; j++ )
					{
						poseMat.at<double>(i,j) = rot.at<double>(i,j);
						std::cout << poseMat.at<double>(i,j) << " ";
					}
					poseMat.at<double>(i,3) = tvec.at<double>(i,0);
					std::cout << poseMat.at<double>(i,3) << std::endl;
				}*/

				std::vector<cv::Point2f> coords(4);
				std::vector<cv::Point3f> coords3d(4);
				Eigen::Matrix4d tr;
				tr << conv[0][0], conv[0][1], conv[0][2], conv[0][3], 
				   conv[1][0], conv[1][1], conv[1][2], conv[1][3], 
				   conv[2][0], conv[2][1], conv[2][2], conv[2][3], 
				   0.0,        0.0,        0.0,        1.0;
				/*std::cout << "tr" << std::endl;
				std::cout << tr << std::endl;
				Eigen::Matrix4d trinv = Eigen::Matrix4d::Identity();
				trinv.block<3,3>(0,0) = tr.block<3,3>(0,0).transpose();
				trinv.block<3,1>(0,3) = - trinv.block<3,3>(0,0) * tr.block<3,1>(0,3);
				std::cout << "trinv" << std::endl;
				std::cout << trinv << std::endl;
				Eigen::Matrix<double,4,4> K;
				K << m_arhandle->arParamLT->param.mat[0][0], m_arhandle->arParamLT->param.mat[0][1], m_arhandle->arParamLT->param.mat[0][2], 0.0,
				  m_arhandle->arParamLT->param.mat[1][0], m_arhandle->arParamLT->param.mat[1][1], m_arhandle->arParamLT->param.mat[1][2], 0.0,
				  m_arhandle->arParamLT->param.mat[2][0], m_arhandle->arParamLT->param.mat[2][1], m_arhandle->arParamLT->param.mat[2][2], 0.0,
				  0.0,0.0,0.0,1.0;
				std::cout << "K" << std::endl;
				std::cout << K << std::endl;*/
				for(int j = 0; j < 4; j++)
				{
					int ind = (4-m_arhandle->markerInfo[i].dirMatrix + j)%4;
					//save 2d position for each point
					coords[j] = cv::Point2f(m_arhandle->markerInfo[i].vertex[ind][0],m_arhandle->markerInfo[i].vertex[ind][1]);
					//estimate 3d position for each point
					if(j == 0)
						coords3d[j] = cv::Point3f(tr(0,3),
												  tr(1,3),
												  tr(2,3));
					if(j == 1)
						coords3d[j] = cv::Point3f(m_markersize*tr(0,0) + tr(0,3),
												  m_markersize*tr(1,0) + tr(1,3),
												  m_markersize*tr(2,0) + tr(2,3));
					if(j == 2)
						coords3d[j] = cv::Point3f(m_markersize*tr(0,0) + m_markersize*tr(0,1) + tr(0,3),
												  m_markersize*tr(1,0) + m_markersize*tr(1,1) + tr(1,3),
												  m_markersize*tr(2,0) + m_markersize*tr(2,1) + tr(2,3));
					if(j == 3)
						coords3d[j] = cv::Point3f(m_markersize*tr(0,1) + tr(0,3),
												  m_markersize*tr(1,1) + tr(1,3),
												  m_markersize*tr(2,1) + tr(2,3));
					/*std::cout << coords3d[j] << std::endl;
					std::cout << coords[j] << std::endl;
					Eigen::Vector4d vec3(coords3d[j].x,coords3d[j].y,coords3d[j].z,1.0);
					Eigen::Vector4d vec2_2 = K*vec3;
					std::cout << vec2_2.x() / vec2_2.z() << " " << vec2_2.y() / vec2_2.z() << std::endl;*/
					
				}
				curcoords.insert(std::pair<int, std::vector<cv::Point2f> >(m_arhandle->markerInfo[i].idMatrix,coords));
				curcoords3d.insert(std::pair<int, std::vector<cv::Point3f> >(m_arhandle->markerInfo[i].idMatrix,coords3d));
			}
		}
	}
	
	bool added = false;
	if(curcoords.size() == (unsigned int)m_nbmarkers)
	{
		for(std::map<int, std::vector<cv::Point2f> >::iterator it = curcoords.begin(); it != curcoords.end(); it++)
		{
			m_savedcoords[it->first].push_back(it->second);
		}
		for(std::map<int, std::vector<cv::Point3f> >::iterator it = curcoords3d.begin(); it != curcoords3d.end(); it++)
		{
			m_savedcoords3d[it->first].push_back(it->second);
		}
		added = true;
	}
	
	return added;
}

bool tracking::Tracker::computeMulti()
{
	bool done = false;
	if(m_savedcoords.begin()->second.size() >= 3)
	{
		if(m_armulti)
		{
			delete[] m_armulti->marker;
			delete m_armulti;
			m_armulti = NULL;
		}
		m_armulti = new ARMultiMarkerInfoT;
		m_armulti->marker_num = m_nbmarkers;
		m_armulti->marker = new ARMultiEachMarkerInfoT[m_nbmarkers];
		m_armulti->prevF = 0;
		m_armulti->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_MATRIX;
		m_armulti->cfPattCutoff = AR_MULTI_CONFIDENCE_PATTERN_CUTOFF_DEFAULT;
		m_armulti->cfMatrixCutoff = AR_MULTI_CONFIDENCE_MATRIX_CUTOFF_DEFAULT;
		
		// for each marker, compute position relative to the first one
		std::vector<std::vector<cv::Point3f> > setPts(m_savedcoords3d.size());
		
		unsigned int nummark = 0;
		for(std::map<int, std::list<std::vector<cv::Point3f> > >::iterator itm = m_savedcoords3d.begin(); itm != m_savedcoords3d.end(); itm++)
		{
			unsigned int numlist = 0;
			setPts[nummark].resize(4*itm->second.size());
			std::cout << nummark << std::endl;
			for(std::list<std::vector<cv::Point3f> >::iterator itl = itm->second.begin(); itl != itm->second.end(); itl++)
			{
				for(unsigned int i = 0; i < 4; i++)
				{
					setPts[nummark][i+numlist*4] = (*itl)[i];
				}
				numlist++;
			}
			m_armulti->marker[nummark].patt_type = AR_MULTI_PATTERN_TYPE_MATRIX;
			m_armulti->marker[nummark].patt_id = itm->first & 0x00007fffULL;
			//std::cout << "id : " << m_armulti->marker[nummark].patt_id << std::endl;
			m_armulti->marker[nummark].globalID = itm->first;
			m_armulti->marker[nummark].width = m_markersize;
			
			nummark++;
		}
		
		for(unsigned int i = 0; i < setPts.size(); i++)
		{
			std::vector<cv::Point3f> meanPt(4,cv::Point3f(0,0,0));
			if(i != 0)
			{
				cv::Mat transform;
				cv::Mat inliers;
				cv::estimateAffine3D(setPts[i],setPts[0],transform,inliers);
				unsigned int nbview = setPts[i].size()/4;
				for(unsigned int j = 0; j < setPts[i].size(); j++)
				{
					meanPt[j%4].x +=
						(transform.at<double>(0,0)*setPts[i][j].x + transform.at<double>(0,1)*setPts[i][j].y +
 						 transform.at<double>(0,2)*setPts[i][j].z + transform.at<double>(0,3))/(float)nbview;
					meanPt[j%4].y +=
						(transform.at<double>(1,0)*setPts[i][j].x + transform.at<double>(1,1)*setPts[i][j].y +
						 transform.at<double>(1,2)*setPts[i][j].z + transform.at<double>(1,3))/(float)nbview;
					meanPt[j%4].z +=
						(transform.at<double>(2,0)*setPts[i][j].x + transform.at<double>(2,1)*setPts[i][j].y +
						 transform.at<double>(2,2)*setPts[i][j].z + transform.at<double>(2,3))/(float)nbview;
				}
				cv::Mat cur_tr = cv::Mat(4,4,CV_64F,cv::Scalar(0));

				cur_tr.at<double>(0,3) = meanPt[0].x;
				cur_tr.at<double>(1,3) = meanPt[0].y;
				cur_tr.at<double>(2,3) = meanPt[0].z;

				cur_tr.at<double>(0,0) = meanPt[1].x-meanPt[0].x;
				cur_tr.at<double>(1,0) = meanPt[1].y-meanPt[0].y;
				cur_tr.at<double>(2,0) = meanPt[1].z-meanPt[0].z;
				cur_tr.col(0) *= (1.0/sqrt(cur_tr.col(0).dot(cur_tr.col(0))));

				cur_tr.at<double>(0,1) = meanPt[2].x-meanPt[0].x;
				cur_tr.at<double>(1,1) = meanPt[2].y-meanPt[0].y;
				cur_tr.at<double>(2,1) = meanPt[2].z-meanPt[0].z;
				cur_tr.col(1) -= cur_tr.col(0)*(cur_tr.col(0).dot(cur_tr.col(1)));
				cur_tr.col(1) *= (1.0/sqrt(cur_tr.col(1).dot(cur_tr.col(1))));

				cur_tr.col(0).rowRange(0,3).cross(cur_tr.col(1).rowRange(0,3)).copyTo(cur_tr.col(2).rowRange(0,3));

				std::cout << i << std::endl;

				std::cout << nbview << std::endl;

				std::cout << "trans" << std::endl;
				for(unsigned int j = 0; j < 3; j++)
				{
					for(unsigned int k = 0; k < 4; k++)
					{
						m_armulti->marker[i].trans[j][k] = cur_tr.at<double>(j,k);
						std::cout << m_armulti->marker[i].trans[j][k] << " ";
					}
					std::cout << std::endl;
				}
				
				arUtilMatInv( (const ARdouble (*)[4])m_armulti->marker[i].trans, m_armulti->marker[i].itrans );
				
				m_armulti->marker[i].pos3d[0][0] = cur_tr.at<double>(0,3);
				m_armulti->marker[i].pos3d[0][1] = cur_tr.at<double>(1,3);
				m_armulti->marker[i].pos3d[0][2] = cur_tr.at<double>(2,3);
				
				m_armulti->marker[i].pos3d[1][0] = cur_tr.at<double>(0,3) + m_markersize*cur_tr.at<double>(0,0);
				m_armulti->marker[i].pos3d[1][1] = cur_tr.at<double>(1,3) + m_markersize*cur_tr.at<double>(1,0);
				m_armulti->marker[i].pos3d[1][2] = cur_tr.at<double>(2,3) + m_markersize*cur_tr.at<double>(2,0);
				
				m_armulti->marker[i].pos3d[2][0] = cur_tr.at<double>(0,3) + m_markersize*cur_tr.at<double>(0,0) + m_markersize*cur_tr.at<double>(0,1);
				m_armulti->marker[i].pos3d[2][1] = cur_tr.at<double>(1,3) + m_markersize*cur_tr.at<double>(1,0) + m_markersize*cur_tr.at<double>(1,1);
				m_armulti->marker[i].pos3d[2][2] = cur_tr.at<double>(2,3) + m_markersize*cur_tr.at<double>(2,0) + m_markersize*cur_tr.at<double>(2,1);
				
				m_armulti->marker[i].pos3d[3][0] = cur_tr.at<double>(0,3) + m_markersize*cur_tr.at<double>(0,1);
				m_armulti->marker[i].pos3d[3][1] = cur_tr.at<double>(1,3) + m_markersize*cur_tr.at<double>(1,1);
				m_armulti->marker[i].pos3d[3][2] = cur_tr.at<double>(2,3) + m_markersize*cur_tr.at<double>(2,1);
			}
			else
			{
				unsigned int nbview = setPts[i].size()/4;
				for(unsigned int j = 0; j < setPts[i].size(); j++)
				{
					meanPt[j%4].x += setPts[i][j].x/(float)nbview;
					meanPt[j%4].y += setPts[i][j].y/(float)nbview;
					meanPt[j%4].z += setPts[i][j].z/(float)nbview;
				}
				m_armulti->marker[i].trans[0][0] = 1.0;
				m_armulti->marker[i].trans[0][1] = 0.0;
				m_armulti->marker[i].trans[0][2] = 0.0;
				m_armulti->marker[i].trans[0][3] = 0.0;
				
				m_armulti->marker[i].trans[1][0] = 0.0;
				m_armulti->marker[i].trans[1][1] = 1.0;
				m_armulti->marker[i].trans[1][2] = 0.0;
				m_armulti->marker[i].trans[1][3] = 0.0;
				
				m_armulti->marker[i].trans[2][0] = 0.0;
				m_armulti->marker[i].trans[2][1] = 0.0;
				m_armulti->marker[i].trans[2][2] = 1.0;
				m_armulti->marker[i].trans[2][3] = 0.0;
				
				arUtilMatInv( (const ARdouble (*)[4])m_armulti->marker[i].trans, m_armulti->marker[i].itrans );
				
				std::cout << i << std::endl;
				for(unsigned int j = 0; j < 3; j++)
				{
					for(unsigned int k = 0; k < 4; k++)
					{
						std::cout << m_armulti->marker[i].trans[j][k] << " ";
					}
					std::cout << std::endl;
				}
				
				arUtilMatInv( (const ARdouble (*)[4])m_armulti->marker[i].trans, m_armulti->marker[i].itrans );
				
				m_armulti->marker[i].pos3d[0][0] = 0;
				m_armulti->marker[i].pos3d[0][1] = 0;
				m_armulti->marker[i].pos3d[0][2] = 0;
				
				m_armulti->marker[i].pos3d[1][0] = m_markersize;
				m_armulti->marker[i].pos3d[1][1] = 0;
				m_armulti->marker[i].pos3d[1][2] = 0;
				
				m_armulti->marker[i].pos3d[2][0] = m_markersize;
				m_armulti->marker[i].pos3d[2][1] = m_markersize;
				m_armulti->marker[i].pos3d[2][2] = 0;
				
				m_armulti->marker[i].pos3d[3][0] = 0;
				m_armulti->marker[i].pos3d[3][1] = m_markersize;
				m_armulti->marker[i].pos3d[3][2] = 0;
			}
		}
		
		done = true;
	}

	return done;
}

Eigen::Matrix<double,3,4> tracking::Tracker::getCurrTr()
{
    arGetTransMatMultiSquareRobust(m_ar3dhandle, m_arhandle->markerInfo, m_arhandle->marker_num, m_armulti);

	Eigen::Matrix<double,3,4> matTr;
	matTr <<
		m_armulti->trans[0][0], m_armulti->trans[0][1], m_armulti->trans[0][2], m_armulti->trans[0][3], 
		m_armulti->trans[1][0], m_armulti->trans[1][1], m_armulti->trans[1][2], m_armulti->trans[1][3], 
		m_armulti->trans[2][0], m_armulti->trans[2][1], m_armulti->trans[2][2], m_armulti->trans[2][3];

	return matTr;
}
