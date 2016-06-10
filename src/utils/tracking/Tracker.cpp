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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

tracking::Tracker::Tracker(unsigned int nbmarkers, double markersize) : m_arhandle(NULL), m_arparamlt(NULL), m_armulti(NULL), m_matK(3,3,CV_64F), m_distCoeff(cv::Mat::zeros(1,4,CV_64F)), m_nbmarkers(nbmarkers), m_markersize(markersize), rvec(1,1,CV_64F)
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

	for(unsigned int i = 0; i < 3; i++)
		for(unsigned int j = 0; j < 3; j++)
			m_matK.at<double>(i,j) = arparam.mat[i][j];
	
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
				int ind1 = (4 - m_arhandle->markerInfo[i].dirMatrix + j)%4;
				int ind2 = (ind1 + 1)%4;
				cv::Scalar col(255,0,0);
				if(j == 0)
					col = cv::Scalar(0,255,0);
				if(j == 1)
					col = cv::Scalar(0,0,255);
				cv::Point2i pt1(m_arhandle->markerInfo[i].vertex[ind1][0],m_arhandle->markerInfo[i].vertex[ind1][1]),
							pt2(m_arhandle->markerInfo[i].vertex[ind2][0],m_arhandle->markerInfo[i].vertex[ind2][1]);
				if(m_arhandle->markerInfo[i].cf > 0.5)
					cv::line(img,pt1,pt2,col,2);
				else
					cv::line(img,pt1,pt2,cv::Scalar(0,0,255),2);
			}
		}
	}
	
	return m_arhandle->marker_num;
}

bool tracking::Tracker::addCurrDetection()
{
	std::map<int, cv::Mat > curpose;

	if(m_arhandle->marker_num >= m_nbmarkers)
	{
		for(int i = 0; i < m_arhandle->marker_num; i++)
		{
			if(m_arhandle->markerInfo[i].cf > 0.5)
			{
				ARdouble conv[3][4];
				arGetTransMatSquare( m_ar3dhandle, m_arhandle->markerInfo + i, m_markersize, conv);

				std::vector<cv::Point2f> coords(4);
				std::vector<cv::Point3f> coords3d(4);
				cv::Mat posemat = cv::Mat::eye(4,4,CV_64F);
				for(int j = 0; j < 3; j++)
					for(int k = 0; k < 4; k++)
					{
						posemat.at<double>(j,k) = conv[j][k];
					}
				curpose.insert(std::pair<int, cv::Mat>(m_arhandle->markerInfo[i].idMatrix,posemat));
			}
			m_globalid.insert(std::pair<int,int>(m_arhandle->markerInfo[i].idMatrix,m_arhandle->markerInfo[i].globalID));
		}
	}
	
	bool added = false;
	if(curpose.size() == (unsigned int)m_nbmarkers)
	{
		for(std::map<int, cv::Mat>::iterator it = curpose.begin(); it != curpose.end(); it++)
		{
			m_savedposes[it->first].push_back(it->second);
		}
		added = true;
	}
	
	return added;
}

bool tracking::Tracker::computeMulti()
{
	bool done = false;
	if(m_savedposes.begin()->second.size() >= 3)
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
		std::vector<std::list<cv::Mat> > correctedMat(m_savedposes.size());
		
		unsigned int nummark = 0;
		for(std::map<int, std::list<cv::Mat> >::iterator itm = m_savedposes.begin(); itm != m_savedposes.end(); itm++)
		{
			cv::Point3d curTr(0,0,0);
			cv::Point3d curVec1(0,0,0);
			cv::Point3d curVec2(0,0,0);
			unsigned int nbImg = itm->second.size();
			for(std::list<cv::Mat>::iterator itl = itm->second.begin(), itl0 = m_savedposes.begin()->second.begin(); itl != itm->second.end() && itl0 != m_savedposes.begin()->second.end(); itl++, itl0++)
			{
				cv::Mat corr = (*itl0).inv()*(*itl);
				correctedMat[nummark].push_back(corr);
				curTr.x += corr.at<double>(0,3)/(double)nbImg;
				curTr.y += corr.at<double>(1,3)/(double)nbImg;
				curTr.z += corr.at<double>(2,3)/(double)nbImg;
				curVec1.x += corr.at<double>(0,0);
				curVec1.y += corr.at<double>(1,0);
				curVec1.z += corr.at<double>(2,0);
				curVec2.x += corr.at<double>(0,1);
				curVec2.y += corr.at<double>(1,1);
				curVec2.z += corr.at<double>(2,1);
			}
			double nor1 = cv::norm(curVec1);
			curVec1.x /= nor1;
			curVec1.y /= nor1;
			curVec1.z /= nor1;
			double nor2 = cv::norm(curVec2);
			curVec2.x /= nor2;
			curVec2.y /= nor2;
			curVec2.z /= nor2;
			cv::Point3f curVec3 = curVec1.cross(curVec2);
			curVec2 = curVec3.cross(curVec1);
			m_armulti->marker[nummark].patt_type = AR_MULTI_PATTERN_TYPE_MATRIX;
			m_armulti->marker[nummark].patt_id = itm->first;
			m_armulti->marker[nummark].globalID = m_globalid[itm->first];
			m_armulti->marker[nummark].width = m_markersize;

			m_armulti->marker[nummark].trans[0][0] = curVec1.x;
			m_armulti->marker[nummark].trans[1][0] = curVec1.y;
			m_armulti->marker[nummark].trans[2][0] = curVec1.z;

			m_armulti->marker[nummark].trans[0][1] = curVec2.x;
			m_armulti->marker[nummark].trans[1][1] = curVec2.y;
			m_armulti->marker[nummark].trans[2][1] = curVec2.z;

			m_armulti->marker[nummark].trans[0][2] = curVec3.x;
			m_armulti->marker[nummark].trans[1][2] = curVec3.y;
			m_armulti->marker[nummark].trans[2][2] = curVec3.z;

			m_armulti->marker[nummark].trans[0][3] = curTr.x;
			m_armulti->marker[nummark].trans[1][3] = curTr.y;
			m_armulti->marker[nummark].trans[2][3] = curTr.z;


			m_armulti->marker[nummark].pos3d[0][0] = curTr.x - m_markersize*curVec1.x/2.0 + m_markersize*curVec2.x/2.0;
			m_armulti->marker[nummark].pos3d[0][1] = curTr.y - m_markersize*curVec1.y/2.0 + m_markersize*curVec2.y/2.0;
			m_armulti->marker[nummark].pos3d[0][2] = curTr.z - m_markersize*curVec1.z/2.0 + m_markersize*curVec2.z/2.0;

			m_armulti->marker[nummark].pos3d[1][0] = curTr.x + m_markersize*curVec1.x/2.0 + m_markersize*curVec2.x/2.0;
			m_armulti->marker[nummark].pos3d[1][1] = curTr.y + m_markersize*curVec1.y/2.0 + m_markersize*curVec2.y/2.0;
			m_armulti->marker[nummark].pos3d[1][2] = curTr.z + m_markersize*curVec1.z/2.0 + m_markersize*curVec2.z/2.0;
			
			m_armulti->marker[nummark].pos3d[2][0] = curTr.x - m_markersize*curVec2.x/2.0 + m_markersize*curVec1.x/2.0;
			m_armulti->marker[nummark].pos3d[2][1] = curTr.y - m_markersize*curVec2.y/2.0 + m_markersize*curVec1.y/2.0;
			m_armulti->marker[nummark].pos3d[2][2] = curTr.z - m_markersize*curVec2.z/2.0 + m_markersize*curVec1.z/2.0;

			m_armulti->marker[nummark].pos3d[3][0] = curTr.x - m_markersize*curVec1.x/2.0 - m_markersize*curVec2.x/2.0;
			m_armulti->marker[nummark].pos3d[3][1] = curTr.y - m_markersize*curVec1.y/2.0 - m_markersize*curVec2.y/2.0;
			m_armulti->marker[nummark].pos3d[3][2] = curTr.z - m_markersize*curVec1.z/2.0 - m_markersize*curVec2.z/2.0;
			
			std::cout << nummark << std::endl;
			for(unsigned int i = 0; i < 4; i++)
			{
				for(unsigned int j = 0; j < 3; j++)
				{
					std::cout << m_armulti->marker[nummark].pos3d[i][j] << " ";
				}
				std::cout << std::endl;
			}

			arUtilMatInv( (const ARdouble (*)[4])m_armulti->marker[nummark].trans, m_armulti->marker[nummark].itrans );
			
			nummark++;
		}
		done = true;
	}

	return done;
}

bool tracking::Tracker::getCurrTr(Eigen::Matrix<double,3,4> &matTr)
{
	bool correct = false;
	std::vector<cv::Point2f> imgpts(0);
	std::vector<cv::Point3f> objpts(0);
	for ( int i = 0; i < m_arhandle->marker_num; i++ )
	{
		ARMarkerInfo marker = m_arhandle->markerInfo[i];
		if ( marker.idMatrix != -1)
		{
			unsigned int num;
			for(int j=0;j<m_armulti->marker_num;j++)
			{
				if(m_armulti->marker[j].patt_id==marker.idMatrix)
					num=j;
			}
			for(unsigned int j=0;j<4;j++)
			{
				imgpts.push_back(cv::Point2f( ( float ) marker.vertex[(4-marker.dirMatrix+j)%4][0], ( float ) marker.vertex[(4-marker.dirMatrix+j)%4][1] ));
				objpts.push_back(cv::Point3f( 
							( float ) m_armulti->marker[num].pos3d[j][0],
							( float ) m_armulti->marker[num].pos3d[j][1],
							( float ) m_armulti->marker[num].pos3d[j][2]));
			}
		}
	}
	
	if(imgpts.size() >= 4)
	{
		std::cout << "Im = [";
		for(unsigned int i = 0; i < imgpts.size(); i++)
		{
			std::cout << imgpts[i].x << " " << imgpts[i].y << std::endl;
		}
		std::cout << "]" << std::endl;
		std::cout << "Pt3d = [";
		for(unsigned int i = 0; i < imgpts.size(); i++)
		{
			std::cout << objpts[i].x << " " << objpts[i].y << " " << objpts[i].z << " 1" << std::endl;
		}
		std::cout << "]';" << std::endl;
		if(rvec.cols == 1 && rvec.rows == 1)
		{
			cv::solvePnP(objpts,imgpts,m_matK,m_distCoeff,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
		}
		else
		{
			cv::solvePnP(objpts,imgpts,m_matK,m_distCoeff,rvec,tvec,true,cv::SOLVEPNP_ITERATIVE);
		}
		cv::Mat rot(3,3,CV_32F);
		cv::Rodrigues(rvec,rot);

		std::cout << "matK = [";
		for( int i = 0; i < 3; i++ )
		{
			for( int j = 0; j < 3; j++ )
			{
				std::cout << m_matK.at<double>(i,j) << " ";
			}
			std::cout << "0";
			std::cout << std::endl;
		}
		std::cout << "];" << std::endl;
		
		std::cout << "Tr = [";
		for( int i = 0; i < 3; i++ )
		{
			for( int j = 0; j < 3; j++ )
			{
				matTr(i,j) = rot.at<double>(i,j);
				std::cout << matTr(i,j) << " ";
			}
			matTr(i,3) = tvec.at<double>(i,0);
			std::cout << matTr(i,3) << std::endl;;
		}
		std::cout << "0 0 0 1";
		std::cout << "];" << std::endl;

		std::cout << "ImHomog = matK*Tr*Pt3d;" << std::endl;
		std::cout << "C2d = [ImHomog(1,:)./ImHomog(3,:)" << std::endl;
		std::cout << "ImHomog(2,:)./ImHomog(3,:)]'" << std::endl;
		std::cout << "figure(1)" << std::endl;
		std::cout << "hold off" << std::endl;
		std::cout << "plot(Im(:,1),Im(:,2))" << std::endl;
		std::cout << "hold on" << std::endl;
		std::cout << "plot(Im(1,1),Im(1,2),'+','color','red')" << std::endl;
		std::cout << "axis equal" << std::endl;
		std::cout << "figure(2)" << std::endl;
		std::cout << "hold off" << std::endl;
		std::cout << "plot3(Pt3d(1,:),Pt3d(2,:),Pt3d(3,:))" << std::endl;
		std::cout << "hold on" << std::endl;
		std::cout << "plot3(Pt3d(1,1),Pt3d(2,1),Pt3d(3,1),'+','color','red')" << std::endl;
		std::cout << "axis equal" << std::endl;
		correct = true;
	}
    /*arGetTransMatMultiSquareRobust(m_ar3dhandle, m_arhandle->markerInfo, m_arhandle->marker_num, m_armulti);

	matTr <<
		m_armulti->trans[0][0], m_armulti->trans[0][1], m_armulti->trans[0][2], m_armulti->trans[0][3], 
		m_armulti->trans[1][0], m_armulti->trans[1][1], m_armulti->trans[1][2], m_armulti->trans[1][3], 
		m_armulti->trans[2][0], m_armulti->trans[2][1], m_armulti->trans[2][2], m_armulti->trans[2][3];*/

	return correct;
}
