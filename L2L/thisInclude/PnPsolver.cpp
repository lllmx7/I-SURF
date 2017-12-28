/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Ra煤l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#include "stdafx.h"
#include <iostream>

#include "PnPsolver.h"
#include "MapPoint.h"

#include <vector>
#include <cmath>
#include "math.h"
#include <cv.h>
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "highgui.h"
using namespace cv;
//#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <algorithm>

using namespace std;


#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#  define MIN(a,b)  ((a) > (b) ? (b) : (a))


//namespace ORB_SLAM2
//{
extern float fx;
extern float B;
extern float uc0;
extern float vc0;
extern int ifpDX0[8887];
extern int ifpDX1[8887];
int fcountt=2;
PnPsolver::PnPsolver( Match *m0, int matchesCountt, featureP *Pseries0, featureP *Pseries1, int *d0)://用r,c 
    pws(0), us(0), alphas(0), pcs(0), maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    //mvpMapPointMatches = vpMapPointMatches;//他是一个矢量 
    //this->m0=m0;
	 this->matchesCountt=matchesCountt;
	 
    int idx=0;
	 float Q[4][4];
	 for(size_t i=0, iend=matchesCountt; i<iend; i++)//for循环第一个逗号前都是初始化 
    {
        
//主要是p12D和p03D、
//////////////////
//P3D是源图像上的、P2D是目标图像的 
///////////////////
		  featureP *pt0,*pt1;
		  pt0 = &Pseries0[m0[i].idx0], pt1 = &Pseries1[m0[i].idx1];
			 
		  PP p0t;
		  p0t.r=pt0->r;
		  p0t.c=pt0->c;

		  PP3D p03D0=cvReprojectTo3D(p0t,d0[m0[i].idx0],Q);
		  ///////////////////////////////////////////////////////////////////////////////////////////////??修正过后图的中心点在哪？
		  ///////////////////////////////////////////////////////////////////////////////////////////////
        //cv::Mat Pos = pMP->GetWorldPos();//得自己写//把平面坐标转换为3D的 

		  if(p03D0.z>=0)
		  {
		  p03D.push_back(cv::Point3f(p03D0.x,p03D0.y,p03D0.z));//存点的3D坐标
		  p12D.push_back(cv::Point2f(pt1->c,pt1->r));

        mvKeyPointIndices.push_back(i);//里面存的是对应的m0的index
        mvAllIndices.push_back(idx);

        idx++;
		  }
    }
	 memset( ifpDX0, 0x00, matchesCountt*sizeof(int));
	 memset( ifpDX1, 0x00, matchesCountt*sizeof(int));

    // Set camera calibration parameters
    fu = fx;
    fv = fx;
    uc = uc0;
    vc = vc0;

    SetRansacParameters();
}
int ifpDX0[8887],ifpDX1[8887];
PnPsolver::PnPsolver( Mat im0L, Mat im1L, Match *m0, int matchesCountt, featureP *Pseries0, featureP *Pseries1, featureP *Pseries0X, featureP *Pseries1X, featureP *Pseries0Y, featureP *Pseries1Y, int *d0, int *d0X, int *d0Y)://用kpt 
    pws(0), us(0), alphas(0), pcs(0), maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    //mvpMapPointMatches = vpMapPointMatches;//他是一个矢量 
    //this->m0=m0;
	 this->matchesCountt=matchesCountt;
	 
    int idx=0;
	 float Q[4][4];
	 memset( ifpDX0, 0x00, matchesCountt*sizeof(int));
	 memset( ifpDX1, 0x00, matchesCountt*sizeof(int));
	 for(size_t i=0, iend=matchesCountt; i<iend; i++)//for循环第一个逗号前都是初始化 
    {
        
//主要是p12D和p03D、
//////////////////
//P3D是源图像上的、P2D是目标图像的 
///////////////////
		  featureP *pt0,*pt1;
		  PP p0t;
		  PP3D p03D0;
		  switch(m0[i].ifpdx)
		  {
		  case DXDY:
			  pt0 = &Pseries0[m0[i].idx0], pt1 = &Pseries1[m0[i].idx1];
			  p0t.r=pt0->kpt.pt.y;
			  p0t.c=pt0->kpt.pt.x;
			  p03D0=cvReprojectTo3D(p0t,d0[m0[i].idx0],Q);
			 break;
		  case pianDX:
			  pt0 = &Pseries0X[m0[i].idx0], pt1 = &Pseries1X[m0[i].idx1];
			  p0t.r=pt0->kpt.pt.y;
			  p0t.c=pt0->kpt.pt.x;
			  p03D0=cvReprojectTo3D(p0t,d0X[m0[i].idx0],Q);
			 break;
		  case pianDY:
			  pt0 = &Pseries0Y[m0[i].idx0], pt1 = &Pseries1Y[m0[i].idx1];
			  p0t.r=pt0->kpt.pt.y;
			  p0t.c=pt0->kpt.pt.x;
			  p03D0=cvReprojectTo3D(p0t,d0Y[m0[i].idx0],Q);
			 break;
		  default:
			  pt0 = &Pseries0[m0[i].idx0], pt1 = &Pseries1[m0[i].idx1];
			  p0t.r=pt0->kpt.pt.y;
			  p0t.c=pt0->kpt.pt.x;
			  p03D0=cvReprojectTo3D(p0t,d0[m0[i].idx0],Q);
			 break;
		  }
		  ///////////////////////////////////////////////////////////////////////////////////////////////??修正过后图的中心点在哪？
		  ///////////////////////////////////////////////////////////////////////////////////////////////
        //cv::Mat Pos = pMP->GetWorldPos();//得自己写//把平面坐标转换为3D的 

		  if(p03D0.z>=0)
		  {
		  p03D.push_back(cv::Point3f(p03D0.x,p03D0.y,p03D0.z));//存点的3D坐标
		  p12D.push_back(pt1->kpt.pt);

        mvKeyPointIndices.push_back(i);//里面存的是对应的m0的index
        mvAllIndices.push_back(idx);
		  ifpDX0[idx]=m0[i].ifpdx;

        idx++;
		  }
    }

    // Set camera calibration parameters
    fu = fx;
    fv = fx;
    uc = uc0;
    vc = vc0;

    SetRansacParameters();
}
PnPsolver::PnPsolver( vector<cv::DMatch> m0, vector<cv::KeyPoint> keypoints0, vector<cv::KeyPoint> keypoints1, int *d0):
    pws(0), us(0), alphas(0), pcs(0), maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    //mvpMapPointMatches = vpMapPointMatches;//他是一个矢量 
    //this->m0=m0;
	 this->matchesCountt=m0.size();
	 
    int idx=0;
	 for(size_t i=0, iend=m0.size(); i<iend; i++)//for循环第一个逗号前都是初始化 
    {
        
//主要是p12D和p03D、
//////////////////
//P3D是源图像上的、P2D是目标图像的 
///////////////////
		  int i0 = m0[i].queryIdx;
        int i1 = m0[i].trainIdx;
		  cv::Point2f *pt0 = &keypoints0[i0].pt, *pt1 = &keypoints1[i1].pt;

		  PP p0t;
		  p0t.r=pt0->y;
		  p0t.c=pt0->x;
		  float Q[4][4];
		  PP3D p03D0=cvReprojectTo3D(p0t,d0[i0],Q);
		  ///////////////////////////////////////////////////////////////////////////////////////////////??修正过后图的中心点在哪？
		  ///////////////////////////////////////////////////////////////////////////////////////////////
        //cv::Mat Pos = pMP->GetWorldPos();//得自己写//把平面坐标转换为3D的 

		  if(p03D0.z>=0)
		  {
		  p03D.push_back(cv::Point3f(p03D0.x,p03D0.y,p03D0.z));//存点的3D坐标
		  p12D.push_back(*pt1);

        mvKeyPointIndices.push_back(i);//里面存的是对应的m0的index
        mvAllIndices.push_back(idx);

        idx++;
		  }
    }
	 memset( ifpDX0, 0x00, m0.size()*sizeof(int));
	 memset( ifpDX1, 0x00, m0.size()*sizeof(int));

    // Set camera calibration parameters
    fu = fx;
    fv = fx;
    uc = uc0;
    vc = vc0;

    SetRansacParameters();
}

PnPsolver::~PnPsolver()
{
	delete [] pws;
	delete [] us;
	delete [] alphas;
	delete [] pcs;
}


void PnPsolver::SetRansacParameters(float probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2, cv::Mat TcwLast)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;
    mRansacEpsilon = epsilon;
    mRansacMinSet = minSet;

    N = p12D.size(); // number of correspondences//注意、实际上的匹配数 

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    int nMinInliers = N*mRansacEpsilon;//取几个里面最大的，主要还是取决于mRansacEpsilon
    if(nMinInliers<mRansacMinInliers)
        nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;
	mRansacMinInliers2 = N*0.3;//0.5991;

    if(mRansacEpsilon<(float)mRansacMinInliers/N)
        mRansacEpsilon=(float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mvMaxError.resize(mvSigma2.size());
    for(size_t i=0; i<mvSigma2.size(); i++)
        mvMaxError[i] = mvSigma2[i]*th2;

	 if(TcwLast.cols!=0)//导入估计值 
	 {
	 for(int i = 0; i < 3; i++) {
	 float *ptrF=(float *)TcwLast.ptr(i);
     for(int j = 0; j < 3; j++)
      mRi[i][j] = ptrF[j];
     mti[i] = ptrF[3];
	 }
	 }
}


/*cv::Mat PnPsolver::find(vector<bool> &vbInliers, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers,nInliers);    
}
*/


extern float _tr6[];


int mapReduce=-1;//3;
int CounttAtLeast=17;
FILE *fpCal;
cv::Mat PnPsolver::iterate(int nIterations, vector<bool> &vbInliers, int &nInliers)//ransac在此 
{

	 vbInliers = vector<bool>(matchesCountt,false);
    nInliers=0;

	 ///
    set_maximum_number_of_correspondences(p03D.size());
	 int subsetCountt=p03D.size();

    if(N<mRansacMinInliers)
    {
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    int nCurrentIterations = -1;
    while(nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

		  ///套上一次的结果看行不行
		  if(nCurrentIterations==0)
		  {
			  CheckInliers();
		  if(mnInliersi>=mRansacMinInliers2)
        {
            // If it is the best solution so far, save it
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_32F,mRi);
                cv::Mat tcw(3,1,CV_32F,mti);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }
				if(Refine())//精修 
				{
					nInliers = mnRefinedInliers;//最终统计的内点的个数 
					//vbInliers = vector<bool>(matchesCountt,false);
					for(int i=0; i<N; i++)
					{
						if(mvbRefinedInliers[i])
						vbInliers[mvKeyPointIndices[i]] = true;//mvKeyPointIndices里面存的是对应的m0的index
					}
printf("\n手气不错!");
//fprintf(fpCal,"\n%d",nCurrentIterations);
					return mRefinedTcw.clone();
				}
        }
		  }

		  ///
		  if(nCurrentIterations<=mapReduce)
		  {
        reset_correspondences();

        for(int j = 0; j < subsetCountt; j++)
				add_correspondence(p03D[j].x,p03D[j].y,p03D[j].z,p12D[j].x,p12D[j].y);//添加一条对应关系 
		  }
		  else
		  {
			reset_correspondences();

        vAvailableIndices = mvAllIndices;
int idxlins[9];
		  srand((int)time(NULL));
        // Get min set of points//mRansacMinSet是为子集的点个数、默认4 
        for(short i = 0; i < mRansacMinSet; ++i)
        {
			   ///在subsetCountt里面挑
            int randi = rand()%MIN(subsetCountt,vAvailableIndices.size());//产生随机数 

            int idx = vAvailableIndices[randi];
idxlins[i]=idx;
//int idx=idxlins[i];
				add_correspondence(p03D[idx].x,p03D[idx].y,p03D[idx].z,p12D[idx].x,p12D[idx].y);//添加一条对应关系 
				ifpDX1[i]=ifpDX0[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();//哦这个工作是为了防止选重复了///////////////////原厂写错了!!!!z这里该是randi 
            vAvailableIndices.pop_back();
				
        }
		  //检查if够了 
		  int CounttpDx=0,CounttpDy=0;
		  for(int i=0;i<number_of_correspondences;i++)
		  {
			  switch(ifpDX1[i])
			  {
			  case DXDY:CounttpDx++,CounttpDy++;
				  break;
			  case pianDX:CounttpDx++;
				  break;
			  case pianDY:CounttpDy++;
				  break;
			  default:CounttpDx++,CounttpDy++;
				  break;
			  }
		  }
		  if(CounttpDx<mRansacMinSet)
		  {
			  //随机序列
			  int rIndex[8887];
			  int ip=MIN(subsetCountt,vAvailableIndices.size());
			  for(int i=0;i<ip;i++)
				  rIndex[i]=vAvailableIndices[i];
			  int rValue[8887];
			  for(int i=0;i<ip;i++)
				  rValue[i]=rand();
			  for(int i=0;i<ip;i++)
				  for(int j=0;j<ip-1;j++)
				  {
					  if(rValue[j]<rValue[j+1])//从大到小 
					  {
						  int tt=rValue[j];
						  rValue[j]=rValue[j+1];
						  rValue[j+1]=tt;
						  tt=rIndex[j];
						  rIndex[j]=rIndex[j+1];
						  rIndex[j+1]=tt;
					  }
				  }
			  for(int i=0;i<ip;i++)
			  {
				  int idx = rIndex[i];
				  if(ifpDX0[idx]==DXDY|| ifpDX0[idx]==pianDX)
				  {
					  add_correspondence(p03D[idx].x,p03D[idx].y,p03D[idx].z,p12D[idx].x,p12D[idx].y);//添加一条对应关系 
				     ifpDX1[number_of_correspondences-1]=ifpDX0[idx];
					  if(ifpDX0[idx]==DXDY)
					  {
						  CounttpDx++,CounttpDy++;
					  }
					  if(ifpDX0[idx]==pianDX)
						  CounttpDx++;
					  if(CounttpDx>=mRansacMinSet)
						  break;
				  }
			  }
		  }
		  if(CounttpDy<mRansacMinSet)
		  {
			  //随机序列
			  int rIndex[8887];
			  int ip=MIN(subsetCountt,vAvailableIndices.size());
			  for(int i=0;i<ip;i++)
				  rIndex[i]=vAvailableIndices[i];
			  int rValue[8887];
			  for(int i=0;i<ip;i++)
				  rValue[i]=rand();
			  for(int i=0;i<ip;i++)
				  for(int j=0;j<ip-1;j++)
				  {
					  if(rValue[j]<rValue[j+1])//从大到小 
					  {
						  int tt=rValue[j];
						  rValue[j]=rValue[j+1];
						  rValue[j+1]=tt;
						  tt=rIndex[j];
						  rIndex[j]=rIndex[j+1];
						  rIndex[j+1]=tt;
					  }
				  }
			  for(int i=0;i<ip;i++)
			  {
				  int idx = rIndex[i];
				  if(ifpDX0[idx]==DXDY|| ifpDX0[idx]==pianDY)
				  {
					  add_correspondence(p03D[idx].x,p03D[idx].y,p03D[idx].z,p12D[idx].x,p12D[idx].y);//添加一条对应关系 
				     ifpDX1[number_of_correspondences-1]=ifpDX0[idx];
					  if(ifpDX0[idx]==DXDY)
					  {
						  CounttpDx++,CounttpDy++;
					  }
					  if(ifpDX0[idx]==pianDY)
						  CounttpDy++;
					  if(CounttpDy>=mRansacMinSet)
						  break;
				  }
			  }
		  }
		  if(CounttpDx<mRansacMinSet|| CounttpDy<mRansacMinSet)//还凑不够 
		  {
//fprintf(fpCal,"\n%d",nCurrentIterations);
			  return cv::Mat();
		  }
		  }

        // Compute camera pose
		  float _tr000[6];float R_000[3][3];float t_000[3];
		  float _tr111[6];float R_111[3][3];float t_111[3];
        compute_pose0(mRi, mti, _tr000, R_000, t_000, _tr111, R_111, t_111);//internal//他只基于correspondence

		  ///
		  if(nCurrentIterations<=mapReduce)
		  {
			  checkandSort(subsetCountt);
			  if(2*subsetCountt/3>=CounttAtLeast)
			  subsetCountt=2*subsetCountt/3;
			  else
				  nCurrentIterations=mapReduce+1;//范围太小了也不行,蹦出来 
		  }
		  else
		  {
        // Check inliers
        CheckInliers();//!!!!!!!compute和check不须要迭代吗
		  }

        if(mnInliersi>=mRansacMinInliers2)
        {
            // If it is the best solution so far, save it
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_32F,mRi);
                cv::Mat tcw(3,1,CV_32F,mti);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }
				if(Refine())//精修 
				{
					nInliers = mnRefinedInliers;//最终统计的内点的个数 
					//vbInliers = vector<bool>(matchesCountt,false);
					for(int i=0; i<N; i++)
					{
						if(mvbRefinedInliers[i])
						vbInliers[mvKeyPointIndices[i]] = true;//mvKeyPointIndices里面存的是对应的m0的index
					}
//fprintf(fpCal,"\n%d",nCurrentIterations);
					return mRefinedTcw.clone();
				}
        }
		  //再试试高斯牛顿法直接求矩阵系数行不行 
		  for(int j=0;j<6;j++)
			  _tr6[j]=_tr000[j];
		  for(int i=0;i<3;i++)
		  {
			  for(int j=0;j<3;j++)
				  mRi[i][j]=R_000[i][j];
			  mti[i]=t_000[i];
		  }
		  CheckInliers();
		  if(mnInliersi>=mRansacMinInliers2)
        {
            // If it is the best solution so far, save it
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_32F,mRi);
                cv::Mat tcw(3,1,CV_32F,mti);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }
				if(Refine())//精修 
				{
					nInliers = mnRefinedInliers;//最终统计的内点的个数 
					//vbInliers = vector<bool>(matchesCountt,false);
					for(int i=0; i<N; i++)
					{
						if(mvbRefinedInliers[i])
						vbInliers[mvKeyPointIndices[i]] = true;//mvKeyPointIndices里面存的是对应的m0的index
					}
//fprintf(fpCal,"\n%d",nCurrentIterations);
					return mRefinedTcw.clone();
				}
        }
		  //再试试高斯牛顿法行不行 
		  for(int j=0;j<6;j++)
			  _tr6[j]=_tr111[j];
		  for(int i=0;i<3;i++)
		  {
			  for(int j=0;j<3;j++)
				  mRi[i][j]=R_111[i][j];
			  mti[i]=t_111[i];
		  }
		  CheckInliers();
		  if(mnInliersi>=mRansacMinInliers2)
        {
            // If it is the best solution so far, save it
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_32F,mRi);
                cv::Mat tcw(3,1,CV_32F,mti);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }
				if(Refine())//精修 
				{
					nInliers = mnRefinedInliers;//最终统计的内点的个数 
					//vbInliers = vector<bool>(matchesCountt,false);
					for(int i=0; i<N; i++)
					{
						if(mvbRefinedInliers[i])
						vbInliers[mvKeyPointIndices[i]] = true;//mvKeyPointIndices里面存的是对应的m0的index
					}
//fprintf(fpCal,"\n%d",nCurrentIterations);
					return mRefinedTcw.clone();
				}
        }
		  //不行再试试原厂的 
		  compute_poseORI(mRi, mti);
		  {
			  //左手坐标系
			  //R[2][0]=sinB;
			  float sinB=mRi[2][0];
			  float b=asin(sinB);
			  float cosB=cos(b);
			  //R[2][1]=-cosB*sinA;
			  float sinA=-mRi[2][1]/cosB;
			  float a=asin(sinA);
			  //R[1][0]=-cosB*sinTh;
			  float sinTh=-mRi[1][0]/cosB;
			  float theta=asin(sinTh);
			  float oX=mti[0];
			  float oY=mti[1];
			  float oZ=mti[2];
			  _tr6[0]=a,_tr6[1]=b,_tr6[2]=theta,_tr6[3]=oX,_tr6[4]=oY,_tr6[5]=oZ;
		  }
		  CheckInliers();
		  if(mnInliersi>=mRansacMinInliers2)
        {
            // If it is the best solution so far, save it
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_32F,mRi);
                cv::Mat tcw(3,1,CV_32F,mti);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }
				if(Refine())//精修 
				{
					nInliers = mnRefinedInliers;//最终统计的内点的个数 
					//vbInliers = vector<bool>(matchesCountt,false);
					for(int i=0; i<N; i++)
					{
						if(mvbRefinedInliers[i])
						vbInliers[mvKeyPointIndices[i]] = true;//mvKeyPointIndices里面存的是对应的m0的index
					}
//fprintf(fpCal,"\n%d",nCurrentIterations);
					return mRefinedTcw.clone();
				}
        }
    }

	 if(mnBestInliers>=mRansacMinInliers2)
	 {
		if(RefineAnyway())//精修 
		{
MessageBox(NULL,_T("勉强"),_T("勉强"),MB_OKCANCEL);
					nInliers = mnRefinedInliers;//最终统计的内点的个数 
					//vbInliers = vector<bool>(matchesCountt,false);
					for(int i=0; i<N; i++)
					{
						if(mvbRefinedInliers[i])
						vbInliers[mvKeyPointIndices[i]] = true;//mvKeyPointIndices里面存的是对应的m0的index
					}
//fprintf(fpCal,"\n%d",nCurrentIterations);
					return mRefinedTcw.clone();
		}
	 }
    return cv::Mat();
}


bool PnPsolver::Refine()
{
    vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());

    for(size_t i=0; i<mvbBestInliers.size(); i++)
    {
        if(mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    set_maximum_number_of_correspondences(vIndices.size());

    reset_correspondences();

    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        add_correspondence(p03D[idx].x,p03D[idx].y,p03D[idx].z,p12D[idx].x,p12D[idx].y);
		  ifpDX1[i]=ifpDX0[idx];
    }

    // Compute camera pose
    compute_pose(mRi, mti);

    // Check inliers
    CheckInliers();

    mnRefinedInliers =mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    if(mnInliersi>mRansacMinInliers)
    {
        cv::Mat Rcw(3,3,CV_32F,mRi);
        cv::Mat tcw(3,1,CV_32F,mti);
        mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));
        return true;
    }

    return false;
}
bool PnPsolver::RefineAnyway()
{
    vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());

    for(size_t i=0; i<mvbBestInliers.size(); i++)
    {
        if(mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    set_maximum_number_of_correspondences(vIndices.size());

    reset_correspondences();

    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        add_correspondence(p03D[idx].x,p03D[idx].y,p03D[idx].z,p12D[idx].x,p12D[idx].y);
		  ifpDX1[i]=ifpDX0[idx];
    }

    // Compute camera pose
    compute_pose(mRi, mti);

    // Check inliers
    CheckInliers();

    mnRefinedInliers =mnInliersi;
    mvbRefinedInliers = mvbInliersi;

	 if(mnInliersi>mRansacMinInliers2)
    {
        cv::Mat Rcw(3,3,CV_32F,mRi);
        cv::Mat tcw(3,1,CV_32F,mti);
        mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));
        return true;
    }

    return false;
}


int errorTh=3*3;//7个像素 
void PnPsolver::CheckInliers()
{
    mnInliersi=0;

    for(int i=0; i<N; i++)
    {
        cv::Point3f p0 = p03D[i];
        cv::Point2f p1 = p12D[i];

        float x1 = mRi[0][0]*p0.x+mRi[0][1]*p0.y+mRi[0][2]*p0.z+mti[0];
        float y1 = mRi[1][0]*p0.x+mRi[1][1]*p0.y+mRi[1][2]*p0.z+mti[1];
        float z1 = mRi[2][0]*p0.x+mRi[2][1]*p0.y+mRi[2][2]*p0.z+mti[2];

		  float u1=uc0+fx*x1/z1;
		  float v1=vc0+fx*y1/z1;

		  float u10 = p1.x, v10 = p1.y;

        float error2;
		  switch(ifpDX0[i])
		  {
		  case DXDY:error2 = (u1-u10)*(u1-u10)+(v1-v10)*(v1-v10);
			  break;
		  case pianDX:error2 = (u1-u10)*(u1-u10);
			  break;
		  case pianDY:error2 = (v1-v10)*(v1-v10);
			  break;
		  default:error2 = (u1-u10)*(u1-u10)+(v1-v10)*(v1-v10);
			  break;
		  }
        if(error2<errorTh)
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[i]=false;
        }
    }
}


void PnPsolver::checkandSort(int subsetCountt)
{
    mnInliersi=0;

	 float dist2[889];
    for(int i=0; i<subsetCountt; i++)
    {
        cv::Point3f p0 = p03D[i];
        cv::Point2f p1 = p12D[i];

        float x1 = mRi[0][0]*p0.x+mRi[0][1]*p0.y+mRi[0][2]*p0.z+mti[0];
        float y1 = mRi[1][0]*p0.x+mRi[1][1]*p0.y+mRi[1][2]*p0.z+mti[1];
        float z1 = mRi[2][0]*p0.x+mRi[2][1]*p0.y+mRi[2][2]*p0.z+mti[2];

		  float u1=uc0+fx*x1/z1;
		  float v1=vc0+fx*y1/z1;

		  float u10 = p1.x, v10 = p1.y;

        float error2 = (u1-u10)*(u1-u10)+(v1-v10)*(v1-v10);
		  dist2[i]=error2;
        if(error2<errorTh)
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[i]=false;
        }
    }

	 //sort 从小到大
	 for(int i=0;i<subsetCountt;i++)
		 for(int j=0;j<subsetCountt-1;j++)
		 {
			 if(dist2[j]>=dist2[j+1])//下面换 
			 {
				 cv::Point3f p3Dlins = p03D[j];
				 p03D[j]=p03D[j+1];
				 p03D[j+1]=p3Dlins;
				 
				 cv::Point2f p2Dlins = p12D[j];
				 p12D[j]=p12D[j+1];
				 p12D[j+1]=p2Dlins;

				 int tt=mvKeyPointIndices[j];//里面存的是对应的m0的index
				 mvKeyPointIndices[j]=mvKeyPointIndices[j+1];
				 mvKeyPointIndices[j+1]=tt;
				 
				 tt=mvAllIndices[j];
				 mvAllIndices[j]=mvAllIndices[j+1];
				 mvAllIndices[j+1]=tt;

				 bool iflins=mvbInliersi[j];
				 mvbInliersi[j]=mvbInliersi[j+1];
				 mvbInliersi[j+1]=iflins;
			 }
		 }
}


void PnPsolver::set_maximum_number_of_correspondences(int n)
{
  if (maximum_number_of_correspondences < n) {
    if (pws != 0) delete [] pws;
    if (us != 0) delete [] us;
    if (alphas != 0) delete [] alphas;
    if (pcs != 0) delete [] pcs;

    maximum_number_of_correspondences = n;
    pws = new float[3 * maximum_number_of_correspondences];//源
    us = new float[2 * maximum_number_of_correspondences];//目标
    alphas = new float[4 * maximum_number_of_correspondences];
    pcs = new float[3 * maximum_number_of_correspondences];
  }
}


void PnPsolver::reset_correspondences(void)
{
  number_of_correspondences = 0;
}


void PnPsolver::add_correspondence(float X0, float Y0, float Z0, float u1, float v1)
{
  pws[3 * number_of_correspondences    ] = X0;
  pws[3 * number_of_correspondences + 1] = Y0;
  pws[3 * number_of_correspondences + 2] = Z0;

  us[2 * number_of_correspondences    ] = u1;
  us[2 * number_of_correspondences + 1] = v1;

  number_of_correspondences++;
}


void PnPsolver::choose_control_points(void)
{
  // Take C0 as the reference points centroid:
  cws[0][0] = cws[0][1] = cws[0][2] = 0;
  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      cws[0][j] += pws[3 * i + j];

  for(int j = 0; j < 3; j++)
    cws[0][j] /= number_of_correspondences;


  // Take C1, C2, and C3 from PCA on the reference points:
  CvMat * PW0 = cvCreateMat(number_of_correspondences, 3, CV_32F);

  float pw0tpw0[3 * 3], dc[3], uct[3 * 3];
  CvMat PW0tPW0 = cvMat(3, 3, CV_32F, pw0tpw0);
  CvMat DC      = cvMat(3, 1, CV_32F, dc);
  CvMat UCt     = cvMat(3, 3, CV_32F, uct);

  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      PW0->data.fl[3 * i + j] = pws[3 * i + j] - cws[0][j];

  cvMulTransposed(PW0, &PW0tPW0, 1);
  cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);//【这里除了0】求奇异值分解 

  cvReleaseMat(&PW0);

  for(int i = 1; i < 4; i++) {
    float k = sqrt(dc[i - 1] / number_of_correspondences);
    for(int j = 0; j < 3; j++)
      cws[i][j] = cws[0][j] + k * uct[3 * (i - 1) + j];
  }
}


void PnPsolver::compute_barycentric_coordinates(void)
{
  float cc[3 * 3], cc_inv[3 * 3];
  CvMat CC     = cvMat(3, 3, CV_32F, cc);
  CvMat CC_inv = cvMat(3, 3, CV_32F, cc_inv);

  for(int i = 0; i < 3; i++)
    for(int j = 1; j < 4; j++)
      cc[3 * i + j - 1] = cws[j][i] - cws[0][i];

  cvInvert(&CC, &CC_inv, CV_SVD);//【这里除了0】这里求了逆 
  float * ci = cc_inv;
  for(int i = 0; i < number_of_correspondences; i++) {
    float * pi = pws + 3 * i;
    float * a = alphas + 4 * i;

    for(int j = 0; j < 3; j++)
      a[1 + j] =
	ci[3 * j    ] * (pi[0] - cws[0][0]) +
	ci[3 * j + 1] * (pi[1] - cws[0][1]) +
	ci[3 * j + 2] * (pi[2] - cws[0][2]);
    a[0] = 1.0f - a[1] - a[2] - a[3];
  }
}


void PnPsolver::fill_M(CvMat * M,
		  const int row, const float * as, const float u, const float v)
{
  float * M1 = M->data.fl + row * 12;
  float * M2 = M1 + 12;

  for(int i = 0; i < 4; i++) {
    M1[3 * i    ] = as[i] * fu;
    M1[3 * i + 1] = 0.0;
    M1[3 * i + 2] = as[i] * (uc - u);

    M2[3 * i    ] = 0.0;
    M2[3 * i + 1] = as[i] * fv;
    M2[3 * i + 2] = as[i] * (vc - v);
  }
}


void PnPsolver::compute_ccs(const float * betas, const float * ut)
{
  for(int i = 0; i < 4; i++)
    ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

  for(int i = 0; i < 4; i++) {
    const float * v = ut + 12 * (11 - i);
    for(int j = 0; j < 4; j++)
      for(int k = 0; k < 3; k++)
	ccs[j][k] += betas[i] * v[3 * j + k];
  }
}


void PnPsolver::compute_pcs(void)
{
  for(int i = 0; i < number_of_correspondences; i++) {
    float * a = alphas + 4 * i;
    float * pc = pcs + 3 * i;

    for(int j = 0; j < 3; j++)
      pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
  }
}


float Beta[3]={0.7,0.03,0.007};
float PnPsolver::compute_pose0(float R[3][3], float t[3], float *_tr000, float R_000[][3], float *t_000, float *_tr111, float R_111[][3], float *t_111)
{
  
  float rep_errors[4];
  float Rs[4][3][3], ts[4][3];

  int j=0;
  {
		//find_betas_approX_1(&L_6x10, &Rho, Betas[1]);
		gauss_newtonZIDINGYI0( Rs[j], ts[j], _tr000, R_000, t_000, _tr111, R_111, t_111);
		rep_errors[j] = errCal7(Rs[j], ts[j]);
  }

  copy_R_and_t(Rs[0], ts[0], R, t);
  return rep_errors[0];
}
float PnPsolver::compute_pose(float R[3][3], float t[3])
{
  
  float rep_errors[4];
  float Rs[4][3][3], ts[4][3];

  int j=0;//for(int j=0;j<3;j++)
  {
		//find_betas_approX_1(&L_6x10, &Rho, Betas[1]);
		gauss_newtonZIDINGYI( Rs[j], ts[j], Beta[j]);
		rep_errors[j] = errCal7(Rs[j], ts[j]);
  }

  /*int N = 0;
  if (rep_errors[1] < rep_errors[0]) N = 1;
  if (rep_errors[2] < rep_errors[N]) N = 2;

  copy_R_and_t(Rs[N], ts[N], R, t);
  */
  copy_R_and_t(Rs[0], ts[0], R, t);

  return rep_errors[0];
}
float PnPsolver::compute_poseORI(float R[3][3], float t[3])
{
  choose_control_points();
  compute_barycentric_coordinates();

  CvMat * M = cvCreateMat(2 * number_of_correspondences, 12, CV_32F);

  for(int i = 0; i < number_of_correspondences; i++)
    fill_M(M, 2 * i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);

  float mtm[12 * 12], d[12], ut[12 * 12];
  CvMat MtM = cvMat(12, 12, CV_32F, mtm);
  CvMat D   = cvMat(12,  1, CV_32F, d);
  CvMat Ut  = cvMat(12, 12, CV_32F, ut);

  cvMulTransposed(M, &MtM, 1);
  cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
  cvReleaseMat(&M);

  float l_6x10[6 * 10], rho[6];
  CvMat L_6x10 = cvMat(6, 10, CV_32F, l_6x10);
  CvMat Rho    = cvMat(6,  1, CV_32F, rho);

  compute_L_6x10(ut, l_6x10);
  compute_rho(rho);

  float Betas[4][4], rep_errors[4];
  float Rs[4][3][3], ts[4][3];

  find_betas_approX_1(&L_6x10, &Rho, Betas[1]);
  gauss_newton(&L_6x10, &Rho, Betas[1]);
  rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);

  find_betas_approX_2(&L_6x10, &Rho, Betas[2]);
  gauss_newton(&L_6x10, &Rho, Betas[2]);
  rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

  find_betas_approX_3(&L_6x10, &Rho, Betas[3]);
  gauss_newton(&L_6x10, &Rho, Betas[3]);
  rep_errors[3] = compute_R_and_t(ut, Betas[3], Rs[3], ts[3]);

  int N = 1;
  if (rep_errors[2] < rep_errors[1]) N = 2;
  if (rep_errors[3] < rep_errors[N]) N = 3;

  copy_R_and_t(Rs[N], ts[N], R, t);

  return rep_errors[N];
}


void PnPsolver::copy_R_and_t( float R_src[3][3], float t_src[3],
			float R_dst[3][3], float t_dst[3])
{
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++)
      R_dst[i][j] = R_src[i][j];
    t_dst[i] = t_src[i];
  }
}


float PnPsolver::errCal7( float R[3][3], float t[3])
{
  float sum2 = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    
	 float * pw = pws + 3 * i;
	 float *ptrF=R[0];
	 float x1 = ptrF[0]*pw[0]+ptrF[1]*pw[1]+ptrF[2]*pw[2] + t[0];
	 ptrF=R[1];
    float y1 = ptrF[0]*pw[0]+ptrF[1]*pw[1]+ptrF[2]*pw[2] + t[1];
	 ptrF=R[2];
    float z1 = ptrF[0]*pw[0]+ptrF[1]*pw[1]+ptrF[2]*pw[2] + t[2];
	 float u1=uc0+fx*x1/z1;
	 float v1=vc0+fx*y1/z1;

    float u10 = us[2 * i], v10 = us[2 * i + 1];

    sum2 += (u1-u10)*(u1-u10)+(v1-v10)*(v1-v10);
  }

  return sum2 / number_of_correspondences;
}


void PnPsolver::print_pose(const float R[3][3], const float t[3])
{
  cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
  cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
  cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}



float dist2(const float * p1, const float * p2)
{
  return
    (p1[0] - p2[0]) * (p1[0] - p2[0]) +
    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
    (p1[2] - p2[2]) * (p1[2] - p2[2]);
}
float dot(const float * v1, const float * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}


void PnPsolver::solve_for_sign(void)
{
  if (pcs[2] < 0.0) {
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
	ccs[i][j] = -ccs[i][j];

    for(int i = 0; i < number_of_correspondences; i++) {
      pcs[3 * i    ] = -pcs[3 * i];
      pcs[3 * i + 1] = -pcs[3 * i + 1];
      pcs[3 * i + 2] = -pcs[3 * i + 2];
    }
  }
}


void PnPsolver::estimate_R_and_t(float R[3][3], float t[3])
{
  float pc0[3], pw0[3];

  pc0[0] = pc0[1] = pc0[2] = 0.0;
  pw0[0] = pw0[1] = pw0[2] = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    const float * pc = pcs + 3 * i;
    const float * pw = pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      pc0[j] += pc[j];
      pw0[j] += pw[j];
    }
  }
  for(int j = 0; j < 3; j++) {
    pc0[j] /= number_of_correspondences;
    pw0[j] /= number_of_correspondences;
  }

  float abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
  CvMat ABt   = cvMat(3, 3, CV_32F, abt);
  CvMat ABt_D = cvMat(3, 1, CV_32F, abt_d);
  CvMat ABt_U = cvMat(3, 3, CV_32F, abt_u);
  CvMat ABt_V = cvMat(3, 3, CV_32F, abt_v);

  cvSetZero(&ABt);
  for(int i = 0; i < number_of_correspondences; i++) {
    float * pc = pcs + 3 * i;
    float * pw = pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
      abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
      abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
    }
  }

  cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
	 {
		 float *ptrF0=abt_u + 3 * i,*ptrF1=abt_v + 3 * j;
		 R[i][j] = ptrF0[0]*ptrF1[0]+ptrF0[1]*ptrF1[1]+ptrF0[2]*ptrF1[2];
	 }

  const float det =
    R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
    R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

  if (det < 0) {
    R[2][0] = -R[2][0];
    R[2][1] = -R[2][1];
    R[2][2] = -R[2][2];
  }

  t[0] = pc0[0] - (R[0][0]*pw0[0]+R[0][1]*pw0[1]+R[0][2]*pw0[2]);
  t[1] = pc0[1] - (R[1][0]*pw0[0]+R[1][1]*pw0[1]+R[1][2]*pw0[2]);
  t[2] = pc0[2] - (R[2][0]*pw0[0]+R[2][1]*pw0[1]+R[2][2]*pw0[2]);
}
float PnPsolver::compute_R_and_t(const float * ut, const float * betas,
			     float R[3][3], float t[3])
{
  compute_ccs(betas, ut);
  compute_pcs();

  solve_for_sign();

  estimate_R_and_t(R, t);

  return errCal7(R, t);
}


// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approX_1 = [B11 B12     B13         B14]

void PnPsolver::find_betas_approX_1(const CvMat * L_6x10, const CvMat * Rho,
			       float * betas)
{
  float l_6x4[6 * 4], b4[4];
  CvMat L_6x4 = cvMat(6, 4, CV_32F, l_6x4);
  CvMat B4    = cvMat(4, 1, CV_32F, b4);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
  }

  cvSolve(&L_6x4, Rho, &B4, CV_SVD);//【这里除了0】

  if (b4[0] < 0) {
    betas[0] = sqrt(-b4[0]);
    betas[1] = -b4[1] / betas[0];
    betas[2] = -b4[2] / betas[0];
    betas[3] = -b4[3] / betas[0];
  } else {
    betas[0] = sqrt(b4[0]);
    betas[1] = b4[1] / betas[0];
    betas[2] = b4[2] / betas[0];
    betas[3] = b4[3] / betas[0];
  }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approX_2 = [B11 B12 B22                            ]

void PnPsolver::find_betas_approX_2(const CvMat * L_6x10, const CvMat * Rho,
			       float * betas)
{
  float l_6x3[6 * 3], b3[3];
  CvMat L_6x3  = cvMat(6, 3, CV_32F, l_6x3);
  CvMat B3     = cvMat(3, 1, CV_32F, b3);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
  }

  cvSolve(&L_6x3, Rho, &B3, CV_SVD);

  if (b3[0] < 0) {
    betas[0] = sqrt(-b3[0]);
    betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
  } else {
    betas[0] = sqrt(b3[0]);
    betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
  }

  if (b3[1] < 0) betas[0] = -betas[0];

  betas[2] = 0.0;
  betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approX_3 = [B11 B12 B22 B13 B23                    ]

void PnPsolver::find_betas_approX_3(const CvMat * L_6x10, const CvMat * Rho,
			       float * betas)
{
  float l_6x5[6 * 5], b5[5];
  CvMat L_6x5 = cvMat(6, 5, CV_32F, l_6x5);
  CvMat B5    = cvMat(5, 1, CV_32F, b5);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
    cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
  }

  cvSolve(&L_6x5, Rho, &B5, CV_SVD);

  if (b5[0] < 0) {
    betas[0] = sqrt(-b5[0]);
    betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
  } else {
    betas[0] = sqrt(b5[0]);
    betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) betas[0] = -betas[0];
  betas[2] = b5[3] / betas[0];
  betas[3] = 0.0;
}


void PnPsolver::compute_L_6x10(const float * ut, float * l_6x10)
{
  const float * v[4];

  v[0] = ut + 12 * 11;
  v[1] = ut + 12 * 10;
  v[2] = ut + 12 *  9;
  v[3] = ut + 12 *  8;

  float dv[4][6][3];

  for(int i = 0; i < 4; i++) {
    int a = 0, b = 1;
    for(int j = 0; j < 6; j++) {
      dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
      dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
      dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

      b++;
      if (b > 3) {
	a++;
	b = a + 1;
      }
    }
  }

  for(int i = 0; i < 6; i++) {
    float * row = l_6x10 + 10 * i;
    row[0] =        dot(dv[0][i], dv[0][i]);
    row[1] = 2.0f * dot(dv[0][i], dv[1][i]);
    row[2] =        dot(dv[1][i], dv[1][i]);
    row[3] = 2.0f * dot(dv[0][i], dv[2][i]);
    row[4] = 2.0f * dot(dv[1][i], dv[2][i]);
    row[5] =        dot(dv[2][i], dv[2][i]);
    row[6] = 2.0f * dot(dv[0][i], dv[3][i]);
    row[7] = 2.0f * dot(dv[1][i], dv[3][i]);
    row[8] = 2.0f * dot(dv[2][i], dv[3][i]);
    row[9] =        dot(dv[3][i], dv[3][i]);
  }
}


void PnPsolver::compute_rho(float * rho)
{
  rho[0] = dist2(cws[0], cws[1]);
  rho[1] = dist2(cws[0], cws[2]);
  rho[2] = dist2(cws[0], cws[3]);
  rho[3] = dist2(cws[1], cws[2]);
  rho[4] = dist2(cws[1], cws[3]);
  rho[5] = dist2(cws[2], cws[3]);
}


void PnPsolver::compute_A_and_b_gauss_newton(const float * l_6x10, const float * rho,
					float betas[4], CvMat * A, CvMat * b)
{
  for(int i = 0; i < 6; i++) {
    const float * rowL = l_6x10 + i * 10;
    float * rowA = A->data.fl + i * 4;

    rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
    rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
    rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
    rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

    cvmSet(b, i, 0, rho[i] -
	   (
	    rowL[0] * betas[0] * betas[0] +
	    rowL[1] * betas[0] * betas[1] +
	    rowL[2] * betas[1] * betas[1] +
	    rowL[3] * betas[0] * betas[2] +
	    rowL[4] * betas[1] * betas[2] +
	    rowL[5] * betas[2] * betas[2] +
	    rowL[6] * betas[0] * betas[3] +
	    rowL[7] * betas[1] * betas[3] +
	    rowL[8] * betas[2] * betas[3] +
	    rowL[9] * betas[3] * betas[3]
	    ));
  }
}


float _tr6[6]={0,0,0,1,0,0};//global
int PnPsolver::gauss_newtonZIDINGYI0( float R[][3], float *t, float *_tr000, float R_000[][3], float *t_000, float *_tr111, float R_111[][3], float *t_111)
{

	//init
  const int iterationsMAX = 6;//87;//4;
  float a=0,b=0,theta=0,oX=0,oY=0,oZ=0;
  float cosA=cos(a),sinA=sin(a),cosB=cos(b),sinB=sin(b),cosTh=cos(theta),sinTh=sin(theta);
  //左手坐标系 
	 R_000[0][0]=cosTh*cosB;
	 R_000[0][1]=cosA*sinTh + cosTh*sinA*sinB;
	 R_000[0][2]=sinTh*sinA - cosTh*cosA*sinB;
	 R_000[1][0]=-cosB*sinTh;
	 R_000[1][1]=cosTh*cosA - sinTh*sinA*sinB;
	 R_000[1][2]=cosTh*sinA + cosA*sinTh*sinB;
	 R_000[2][0]=sinB;
	 R_000[2][1]=-cosB*sinA;
	 R_000[2][2]=cosA*cosB;
    t_000[0]=oX;
	 t_000[1]=oY;
	 t_000[2]=oZ;
  //
  //数据归一化 
  uc0=uc0/28;vc0=vc0/28;fx=fx/28;

  //先先高斯牛顿法直接求矩阵系数 
  float beta=0.7;
  for(int iter = 0; iter < iterationsMAX; iter++) {

	 float dR[3][3];
	 float dt[3];
	 memset( dR, 0, 9*sizeof(float));
	 memset( dt, 0, 3*sizeof(float));
	 Mat d2=Mat(cvSize(12,12),CV_32FC1);//二阶导矩阵、R横着3个跟一个t、横着r3个跟一个t 
	 d2=0;
	 for(int j = 0; j < number_of_correspondences; j++) {
		 
		 float *p0=pws + 3 * j;
		 float x0,y0,z0;
		 x0=p0[0];
		 y0=p0[1];
		 z0=p0[2];
		 float x1,y1,z1;
		 x1= R_000[0][0]*x0+R_000[0][1]*y0+R_000[0][2]*z0 + t_000[0];
		 y1= R_000[1][0]*x0+R_000[1][1]*y0+R_000[1][2]*z0 + t_000[1];
		 z1= R_000[2][0]*x0+R_000[2][1]*y0+R_000[2][2]*z0 + t_000[2];
		 float u1=uc0+fx*x1/z1;
		 float v1=vc0+fx*y1/z1;

		 //e=(u1-u10)的平方+(v1-v10)的平方
		 float *p1=us + 2 * j;
		 float u10=p1[0];
		 float v10=p1[1];
		 ////归一化
		 u10=u10/28;v10=v10/28;

		 float errlins[2];
		 float udR00;
		 float udR01;
		 float udR02;
		 float udt0;
		 float udR20;
		 float udR21;
		 float udR22;
		 float udt2;
		 float vdR10;
		 float vdR11;
		 float vdR12;
		 float vdt1;
		 float vdR20;
		 float vdR21;
		 float vdR22;
		 float vdt2;
		 Mat du1lins,dv1lins;
		 float *ptrF;
switch(ifpDX1[j])
{
case pianDX:
		 errlins[0]=u1-u10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 dR[0][0]=dR[0][0]+2*errlins[0]*fx*x0/z1;
		 dR[0][1]=dR[0][1]+2*errlins[0]*fx*y0/z1;
		 dR[0][2]=dR[0][2]+2*errlins[0]*fx*z0/z1;
		 dt[0]=dt[0]+2*errlins[0]*fx/z1;
		 //dR[1][0]=dR[1][0]+2*errlins[1]*fx*x0/z1;
		 //dR[1][1]=dR[1][1]+2*errlins[1]*fx*y0/z1;
		 //dR[1][2]=dR[1][2]+2*errlins[1]*fx*z0/z1;
		 //dt[1]=dt[1]+2*errlins[1]*fx/z1;
		 dR[2][0]=dR[2][0]+2*errlins[0]*(-fx*x1/(z1*z1))*x0;
		 dR[2][1]=dR[2][1]+2*errlins[0]*(-fx*x1/(z1*z1))*y0;
		 dR[2][2]=dR[2][2]+2*errlins[0]*(-fx*x1/(z1*z1))*z0;
		 dt[2]=dt[2]+2*errlins[0]*(-fx*x1/(z1*z1));
		 //、、、、、、、、、、、、、、、、、、、近似二阶导
		 udR00=fx*x0/z1;
		 udR01=fx*y0/z1;
		 udR02=fx*z0/z1;
		 udt0=fx/z1;
		 udR20=(-fx*x1/(z1*z1))*x0;
		 udR21=(-fx*x1/(z1*z1))*y0;
		 udR22=(-fx*x1/(z1*z1))*z0;
		 udt2=(-fx*x1/(z1*z1));
		 du1lins=Mat(cvSize(1,12),CV_32FC1);
		 du1lins=0;
		 ptrF=(float *)du1lins.ptr(0);
		 ptrF[0]=udR00;
		 ptrF[1]=udR01;
		 ptrF[2]=udR02;
		 ptrF[3]=udt0;
		 ptrF[8]=udR20;
		 ptrF[9]=udR21;
		 ptrF[10]=udR22;
		 ptrF[11]=udt2;
		 d2=d2+2*du1lins*du1lins.t();
	break;
case pianDY:
		 errlins[1]=v1-v10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 //dR[0][0]=dR[0][0]+2*errlins[0]*fx*x0/z1;
		 //dR[0][1]=dR[0][1]+2*errlins[0]*fx*y0/z1;
		 //dR[0][2]=dR[0][2]+2*errlins[0]*fx*z0/z1;
		 //dt[0]=dt[0]+2*errlins[0]*fx/z1;
		 dR[1][0]=dR[1][0]+2*errlins[1]*fx*x0/z1;
		 dR[1][1]=dR[1][1]+2*errlins[1]*fx*y0/z1;
		 dR[1][2]=dR[1][2]+2*errlins[1]*fx*z0/z1;
		 dt[1]=dt[1]+2*errlins[1]*fx/z1;
		 dR[2][0]=dR[2][0]+2*errlins[1]*(-fx*y1/(z1*z1))*x0;
		 dR[2][1]=dR[2][1]+2*errlins[1]*(-fx*y1/(z1*z1))*y0;
		 dR[2][2]=dR[2][2]+2*errlins[1]*(-fx*y1/(z1*z1))*z0;
		 dt[2]=dt[2]+2*errlins[1]*(-fx*y1/(z1*z1));
		 //、、、、、、、、、、、、、、、、、、、近似二阶导
		 vdR10=fx*x0/z1;
		 vdR11=fx*y0/z1;
		 vdR12=fx*z0/z1;
		 vdt1=fx/z1;
		 vdR20=(-fx*y1/(z1*z1))*x0;
		 vdR21=(-fx*y1/(z1*z1))*y0;
		 vdR22=(-fx*y1/(z1*z1))*z0;
		 vdt2=(-fx*y1/(z1*z1));
		 dv1lins=Mat(cvSize(1,12),CV_32FC1);
		 dv1lins=0;
		 ptrF=(float *)dv1lins.ptr(0);
		 ptrF[4]=vdR10;
		 ptrF[5]=vdR11;
		 ptrF[6]=vdR12;
		 ptrF[7]=vdt1;
		 ptrF[8]=vdR20;
		 ptrF[9]=vdR21;
		 ptrF[10]=vdR22;
		 ptrF[11]=vdt2;
		 d2=d2+2*dv1lins*dv1lins.t();
	break;
default:
		 errlins[0]=u1-u10;
		 errlins[1]=v1-v10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 dR[0][0]=dR[0][0]+2*errlins[0]*fx*x0/z1;
		 dR[0][1]=dR[0][1]+2*errlins[0]*fx*y0/z1;
		 dR[0][2]=dR[0][2]+2*errlins[0]*fx*z0/z1;
		 dt[0]=dt[0]+2*errlins[0]*fx/z1;
		 dR[1][0]=dR[1][0]+2*errlins[1]*fx*x0/z1;
		 dR[1][1]=dR[1][1]+2*errlins[1]*fx*y0/z1;
		 dR[1][2]=dR[1][2]+2*errlins[1]*fx*z0/z1;
		 dt[1]=dt[1]+2*errlins[1]*fx/z1;
		 dR[2][0]=dR[2][0]+2*errlins[0]*(-fx*x1/(z1*z1))*x0+2*errlins[1]*(-fx*y1/(z1*z1))*x0;
		 dR[2][1]=dR[2][1]+2*errlins[0]*(-fx*x1/(z1*z1))*y0+2*errlins[1]*(-fx*y1/(z1*z1))*y0;
		 dR[2][2]=dR[2][2]+2*errlins[0]*(-fx*x1/(z1*z1))*z0+2*errlins[1]*(-fx*y1/(z1*z1))*z0;
		 dt[2]=dt[2]+2*errlins[0]*(-fx*x1/(z1*z1))+2*errlins[1]*(-fx*y1/(z1*z1));
		 //、、、、、、、、、、、、、、、、、、、近似二阶导
		 udR00=fx*x0/z1;
		 udR01=fx*y0/z1;
		 udR02=fx*z0/z1;
		 udt0=fx/z1;
		 udR20=(-fx*x1/(z1*z1))*x0;
		 udR21=(-fx*x1/(z1*z1))*y0;
		 udR22=(-fx*x1/(z1*z1))*z0;
		 udt2=(-fx*x1/(z1*z1));
		 vdR10=fx*x0/z1;
		 vdR11=fx*y0/z1;
		 vdR12=fx*z0/z1;
		 vdt1=fx/z1;
		 vdR20=(-fx*y1/(z1*z1))*x0;
		 vdR21=(-fx*y1/(z1*z1))*y0;
		 vdR22=(-fx*y1/(z1*z1))*z0;
		 vdt2=(-fx*y1/(z1*z1));
		 du1lins=Mat(cvSize(1,12),CV_32FC1);
		 du1lins=0;
		 dv1lins=Mat(cvSize(1,12),CV_32FC1);
		 dv1lins=0;
		 ptrF=(float *)du1lins.ptr(0);
		 ptrF[0]=udR00;
		 ptrF[1]=udR01;
		 ptrF[2]=udR02;
		 ptrF[3]=udt0;
		 ptrF[8]=udR20;
		 ptrF[9]=udR21;
		 ptrF[10]=udR22;
		 ptrF[11]=udt2;
		 ptrF=(float *)dv1lins.ptr(0);
		 ptrF[4]=vdR10;
		 ptrF[5]=vdR11;
		 ptrF[6]=vdR12;
		 ptrF[7]=vdt1;
		 ptrF[8]=vdR20;
		 ptrF[9]=vdR21;
		 ptrF[10]=vdR22;
		 ptrF[11]=vdt2;
		 d2=d2+2*du1lins*du1lins.t()+2*dv1lins*dv1lins.t();
	break;
}
	 }
	 for(int m=0;m<3;m++)
	 {
		 for(int n=0;n<3;n++)
			 dR[m][n]=dR[m][n]/number_of_correspondences;
		 dt[m]=dt[m]/number_of_correspondences;
	 }
	 d2=d2/number_of_correspondences;
	 
	 //更新R和t
	 Mat d1=Mat(cvSize(1,12),CV_32FC1);
	 float *ptrF_d1=(float *)d1.ptr(0);
	 ptrF_d1[0]=dR[0][0];
	 ptrF_d1[1]=dR[0][1];
	 ptrF_d1[2]=dR[0][2];
	 ptrF_d1[3]=dt[0];
	 ptrF_d1[4]=dR[1][0];
	 ptrF_d1[5]=dR[1][1];
	 ptrF_d1[6]=dR[1][2];
	 ptrF_d1[7]=dt[1];
	 ptrF_d1[8]=dR[2][0];
	 ptrF_d1[9]=dR[2][1];
	 ptrF_d1[10]=dR[2][2];
	 ptrF_d1[11]=dt[2];
	 Mat invd2;
	 invert(d2,invd2);
	 Mat gai=invd2*d1;
	 float *ptr_gai=(float *)gai.ptr(0);
	 R_000[0][0]=R_000[0][0]-beta*ptr_gai[0];
	 R_000[0][1]=R_000[0][1]-beta*ptr_gai[1];
	 R_000[0][2]=R_000[0][2]-beta*ptr_gai[2];
	 t_000[0]=t_000[0]-beta*ptr_gai[3];
	 R_000[1][0]=R_000[1][0]-beta*ptr_gai[4];
	 R_000[1][1]=R_000[1][1]-beta*ptr_gai[5];
	 R_000[1][2]=R_000[1][2]-beta*ptr_gai[6];
	 t_000[1]=t_000[1]-beta*ptr_gai[7];
	 R_000[2][0]=R_000[2][0]-beta*ptr_gai[8];
	 R_000[2][1]=R_000[2][1]-beta*ptr_gai[9];
	 R_000[2][2]=R_000[2][2]-beta*ptr_gai[10];
	 t_000[2]=t_000[2]-beta*ptr_gai[11];
  }
  //
  float normR=R_000[0][0]*R_000[1][1]*R_000[2][2]+R_000[0][1]*R_000[1][2]*R_000[2][0]+R_000[1][0]*R_000[2][1]*R_000[0][2]-R_000[0][2]*R_000[1][1]*R_000[2][0]-R_000[0][1]*R_000[1][0]*R_000[2][2]-R_000[1][2]*R_000[2][1]*R_000[0][0];
  float normR_3;
  if(normR>=0)
	  normR_3=pow((double)normR,0.3333);
  if(normR<0)
  {
	  normR=-normR;
	  normR_3=-pow((double)normR,0.3333);
  }
  R_000[0][0]=R_000[0][0]/normR_3;
  R_000[0][1]=R_000[0][1]/normR_3;
  R_000[0][2]=R_000[0][2]/normR_3;
  t_000[0]=t_000[0]/normR_3;
  R_000[1][0]=R_000[1][0]/normR_3;
  R_000[1][1]=R_000[1][1]/normR_3;
  R_000[1][2]=R_000[1][2]/normR_3;
  t_000[1]=t_000[1]/normR_3;
  R_000[2][0]=R_000[2][0]/normR_3;
  R_000[2][1]=R_000[2][1]/normR_3;
  R_000[2][2]=R_000[2][2]/normR_3;
  t_000[2]=t_000[2]/normR_3;
  //更新a,b,th,oX/Y/Z
  //R[2][0]=sinB;
  sinB=R_000[2][0];
  b=asin(sinB);
  cosB=cos(b);
  //R[2][1]=-cosB*sinA;
  sinA=-R_000[2][1]/cosB;
  a=asin(sinA);
  //R[1][0]=-cosB*sinTh;
  sinTh=-R_000[1][0]/cosB;
  theta=asin(sinTh);
  oX=t_000[0];
  oY=t_000[1];
  oZ=t_000[2];
  //数据还原、参数尺度也还原 
  uc0=uc0*28;vc0=vc0*28;fx=fx*28;
  //record as prior knowledge 
  _tr000[0]=a,_tr000[1]=b,_tr000[2]=theta,_tr000[3]=oX,_tr000[4]=oY,_tr000[5]=oZ;


  //////////////////////////////////
  //、、高斯牛顿法求6个变量 
  //init
  a=0,b=0,theta=0,oX=0,oY=0,oZ=0;
  cosA=cos(a),sinA=sin(a),cosB=cos(b),sinB=sin(b),cosTh=cos(theta),sinTh=sin(theta);
  //左手坐标系 
	 R_111[0][0]=cosTh*cosB;
	 R_111[0][1]=cosA*sinTh + cosTh*sinA*sinB;
	 R_111[0][2]=sinTh*sinA - cosTh*cosA*sinB;
	 R_111[1][0]=-cosB*sinTh;
	 R_111[1][1]=cosTh*cosA - sinTh*sinA*sinB;
	 R_111[1][2]=cosTh*sinA + cosA*sinTh*sinB;
	 R_111[2][0]=sinB;
	 R_111[2][1]=-cosB*sinA;
	 R_111[2][2]=cosA*cosB;
    t_111[0]=oX;
	 t_111[1]=oY;
	 t_111[2]=oZ;
  //
  //数据归一化 
  uc0=uc0/28;vc0=vc0/28;fx=fx/28;

  beta=0.7;
  for(int iter = 0; iter < iterationsMAX; iter++) {

	 float da=0,db=0,dTh=0,doX=0,doY=0,doZ=0;
	 Mat d2=Mat(cvSize(6,6),CV_32FC1);//二阶导矩阵、R横着3个跟一个t、横着r3个跟一个t 
	 d2=0;
	 for(int j = 0; j < number_of_correspondences; j++) {
		 
		 float *p0=pws + 3 * j;
		 float x0,y0,z0;
		 x0=p0[0];
		 y0=p0[1];
		 z0=p0[2];
		 float x1,y1,z1;
		 x1= R_111[0][0]*x0+R_111[0][1]*y0+R_111[0][2]*z0 + t_111[0];
		 y1= R_111[1][0]*x0+R_111[1][1]*y0+R_111[1][2]*z0 + t_111[1];
		 z1= R_111[2][0]*x0+R_111[2][1]*y0+R_111[2][2]*z0 + t_111[2];
		 float u1=uc0+fx*x1/z1;
		 float v1=vc0+fx*y1/z1;

		 //e=(u1-u10)的平方+(v1-v10)的平方
		 float *p1=us + 2 * j;
		 float u10=p1[0];
		 float v10=p1[1];
		 ////归一化
		 u10=u10/28;v10=v10/28;

		 float errlins[2];
		 float uda;
		 float udb;
		 float udTh;
		 float udoX;
		 float udoY;
		 float udoZ;
		 float vda;
		 float vdb;
		 float vdTh;
		 float vdoX;
		 float vdoY;
		 float vdoZ;
		 Mat du1lins,dv1lins;
		 float *ptrF;
switch(ifpDX1[j])
{
case pianDX:
		 errlins[0]=u1-u10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 uda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1) - (fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1);
		 udb =- (fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1);
		 udTh =(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1);
		 udoX =fx/(z1);
		 udoY =0;
		 udoZ =-(fx*(x1))/(z1*z1);

		 da=da+2*errlins[0]*uda;
		 db=db+2*errlins[0]*udb;
		 dTh=dTh+2*errlins[0]*udTh;
		 doX=doX +2*errlins[0]*udoX;//udoY =0;
		 //doY=doY +2*errlins[1]*vdoY;
		 doZ=doZ +2*errlins[0]*udoZ;
		 //、、、、、、、、、、、、、、、、、、、近似二阶导
		 du1lins=Mat(cvSize(1,6),CV_32FC1);
		 ptrF=(float *)du1lins.ptr(0);
		 ptrF[0]=uda;
		 ptrF[1]=udb;
		 ptrF[2]=udTh;
		 ptrF[3]=udoX;
		 ptrF[4]=udoY;
		 ptrF[5]=udoZ;

		 d2=d2+2*du1lins*du1lins.t();
	break;
case pianDY:
		 errlins[1]=v1-v10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 vda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1) - (fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1);
		 vdb =(fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1);
		 vdTh =-(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1);
		 vdoX=0;
		 vdoY =fx/(z1);
		 vdoZ =-(fx*(y1))/(z1*z1);

		 da=da+2*errlins[1]*vda;
		 db=db+2*errlins[1]*vdb;
		 dTh=dTh+2*errlins[1]*vdTh;
		 //doX=doX +2*errlins[0]*udoX;//udoY =0;
		 doY=doY +2*errlins[1]*vdoY;
		 doZ=doZ +2*errlins[1]*vdoZ;
		 //、、、、、、、、、、、、、、、、、、、近似二阶导
		 dv1lins=Mat(cvSize(1,6),CV_32FC1);
		 ptrF=(float *)dv1lins.ptr(0);
		 ptrF[0]=vda;
		 ptrF[1]=vdb;
		 ptrF[2]=vdTh;
		 ptrF[3]=vdoX;
		 ptrF[4]=vdoY;
		 ptrF[5]=vdoZ;

		 d2=d2+2*dv1lins*dv1lins.t();
	break;
default:
		 errlins[0]=u1-u10;
		 errlins[1]=v1-v10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 uda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1) - (fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1);
		 udb =- (fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1);
		 udTh =(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1);
		 udoX =fx/(z1);
		 udoY =0;
		 udoZ =-(fx*(x1))/(z1*z1);
		 vda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1) - (fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1);
		 vdb =(fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1);
		 vdTh =-(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1);
		 vdoX=0;
		 vdoY =fx/(z1);
		 vdoZ =-(fx*(y1))/(z1*z1);

		 da=da+2*errlins[0]*uda+2*errlins[1]*vda;
		 db=db+2*errlins[0]*udb+2*errlins[1]*vdb;
		 dTh=dTh+2*errlins[0]*udTh+2*errlins[1]*vdTh;
		 doX=doX +2*errlins[0]*udoX;//udoY =0;
		 doY=doY +2*errlins[1]*vdoY;
		 doZ=doZ +2*errlins[0]*udoZ+2*errlins[1]*vdoZ;
		 //、、、、、、、、、、、、、、、、、、、近似二阶导
		 du1lins=Mat(cvSize(1,6),CV_32FC1);
		 dv1lins=Mat(cvSize(1,6),CV_32FC1);
		 ptrF=(float *)du1lins.ptr(0);
		 ptrF[0]=uda;
		 ptrF[1]=udb;
		 ptrF[2]=udTh;
		 ptrF[3]=udoX;
		 ptrF[4]=udoY;
		 ptrF[5]=udoZ;
		 ptrF=(float *)dv1lins.ptr(0);
		 ptrF[0]=vda;
		 ptrF[1]=vdb;
		 ptrF[2]=vdTh;
		 ptrF[3]=vdoX;
		 ptrF[4]=vdoY;
		 ptrF[5]=vdoZ;

		 d2=d2+2*du1lins*du1lins.t()+2*dv1lins*dv1lins.t();
	break;
}
	 }
	 da=da/number_of_correspondences;
	 db=db/number_of_correspondences;
	 dTh=dTh/number_of_correspondences;
	 doX=doX/number_of_correspondences;
	 doY=doY/number_of_correspondences;
	 doZ=doZ/number_of_correspondences;
	 d2=d2/number_of_correspondences;
	 
	 Mat d1=Mat(cvSize(1,6),CV_32FC1);
	 float *ptrF_d1=(float *)d1.ptr(0);
	 ptrF_d1[0]=da;
	 ptrF_d1[1]=db;
	 ptrF_d1[2]=dTh;
	 ptrF_d1[3]=doX;
	 ptrF_d1[4]=doY;
	 ptrF_d1[5]=doZ;

	 Mat invd2;
	 invert(d2,invd2);
	 Mat gai=invd2*d1;
	 float *ptr_gai=(float *)gai.ptr(0);
	 a=a-beta*ptr_gai[0];
	 b=b-beta*ptr_gai[1];
	 theta=theta-beta*ptr_gai[2];
	 oX=oX-beta*ptr_gai[3];
	 oY=oY-beta*ptr_gai[4];
	 oZ=oZ-beta*ptr_gai[5];
	 //更新R和t
	 cosA=cos(a),sinA=sin(a),cosB=cos(b),sinB=sin(b),cosTh=cos(theta),sinTh=sin(theta);
	 R_111[0][0]=cosTh*cosB;
	 R_111[0][1]=cosA*sinTh + cosTh*sinA*sinB;
	 R_111[0][2]=sinTh*sinA - cosTh*cosA*sinB;
	 R_111[1][0]=-cosB*sinTh;
	 R_111[1][1]=cosTh*cosA - sinTh*sinA*sinB;
	 R_111[1][2]=cosTh*sinA + cosA*sinTh*sinB;
	 R_111[2][0]=sinB;
	 R_111[2][1]=-cosB*sinA;
	 R_111[2][2]=cosA*cosB;
    t_111[0]=oX;
	 t_111[1]=oY;
	 t_111[2]=oZ;
  }
  //数据还原、参数尺度也还原 
  uc0=uc0*28;vc0=vc0*28;fx=fx*28;
  //record as prior knowledge 
  _tr111[0]=a,_tr111[1]=b,_tr111[2]=theta,_tr111[3]=oX,_tr111[4]=oY,_tr111[5]=oZ;


  //init
  //先先最小二乘求解 
  /*float A[3][3]={3,2,1,
  2,2,2,
  4,-2,-2};
  Mat mA=Mat(cvSize(3,3),CV_32FC1,A);
  float b[3]={6,4,2};
  Mat mb=Mat(cvSize(1,3),CV_32FC1,b);//Mat mx;
  //solve(mA,mb,mx,CV_SVD);
  */
  float A[12][12];
  memset( A, 0, 12*12*sizeof(float));
  int uEqCountt=0,vEqCountt=0;
  for(int j = 0; j < number_of_correspondences;j++)//6; j++)
  {
	  float *p0=pws + 3 * j;
		 float x0,y0,z0;
		 x0=p0[0];
		 y0=p0[1];
		 z0=p0[2];
	  float *p1=us + 2 * j;
		 float u10=p1[0];
		 float v10=p1[1];
	  //A[j][0]=fx*x0,A[j][1]=fx*y0,A[j][2]=fx*z0,A[j][3]=fx,A[j][8]=-(u10-uc0)*x0,A[j][9]=-(u10-uc0)*y0,A[j][10]=-(u10-uc0)*z0,A[j][11]=-(u10-uc0);
	  //A[6+j][4]=fx*x0,A[6+j][5]=fx*y0,A[6+j][6]=fx*z0,A[6+j][7]=fx,A[6+j][8]=-(v10-vc0)*x0,A[6+j][9]=-(v10-vc0)*y0,A[6+j][10]=-(v10-vc0)*z0,A[6+j][11]=-(v10-vc0);
switch(ifpDX1[j])
{
case DXDY:
		 if(uEqCountt<6)
		 {
		 A[uEqCountt][0]=fx*x0,A[uEqCountt][1]=fx*y0,A[uEqCountt][2]=fx*z0,A[uEqCountt][3]=fx,A[uEqCountt][8]=-(u10-uc0)*x0,A[uEqCountt][9]=-(u10-uc0)*y0,A[uEqCountt][10]=-(u10-uc0)*z0,A[uEqCountt][11]=-(u10-uc0);
		 uEqCountt++;//一共6个式子就行,甭管谁的 
		 }
		 if(vEqCountt<6)
		 {
		 A[6+vEqCountt][4]=fx*x0,A[6+vEqCountt][5]=fx*y0,A[6+vEqCountt][6]=fx*z0,A[6+vEqCountt][7]=fx,A[6+vEqCountt][8]=-(v10-vc0)*x0,A[6+vEqCountt][9]=-(v10-vc0)*y0,A[6+vEqCountt][10]=-(v10-vc0)*z0,A[6+vEqCountt][11]=-(v10-vc0);
		 vEqCountt++;
		 }
	break;
case pianDX:
		 if(uEqCountt<6)
		 {
		 A[uEqCountt][0]=fx*x0,A[uEqCountt][1]=fx*y0,A[uEqCountt][2]=fx*z0,A[uEqCountt][3]=fx,A[uEqCountt][8]=-(u10-uc0)*x0,A[uEqCountt][9]=-(u10-uc0)*y0,A[uEqCountt][10]=-(u10-uc0)*z0,A[uEqCountt][11]=-(u10-uc0);
		 uEqCountt++;
		 }
	break;
case pianDY:
		 if(vEqCountt<6)
		 {
		 A[6+vEqCountt][4]=fx*x0,A[6+vEqCountt][5]=fx*y0,A[6+vEqCountt][6]=fx*z0,A[6+vEqCountt][7]=fx,A[6+vEqCountt][8]=-(v10-vc0)*x0,A[6+vEqCountt][9]=-(v10-vc0)*y0,A[6+vEqCountt][10]=-(v10-vc0)*z0,A[6+vEqCountt][11]=-(v10-vc0);
		 vEqCountt++;
		 }
	break;
default:
		 if(uEqCountt<6)
		 {
		 A[uEqCountt][0]=fx*x0,A[uEqCountt][1]=fx*y0,A[uEqCountt][2]=fx*z0,A[uEqCountt][3]=fx,A[uEqCountt][8]=-(u10-uc0)*x0,A[uEqCountt][9]=-(u10-uc0)*y0,A[uEqCountt][10]=-(u10-uc0)*z0,A[uEqCountt][11]=-(u10-uc0);
		 uEqCountt++;//一共6个式子就行,甭管谁的 
		 }
		 if(vEqCountt<6)
		 {
		 A[6+vEqCountt][4]=fx*x0,A[6+vEqCountt][5]=fx*y0,A[6+vEqCountt][6]=fx*z0,A[6+vEqCountt][7]=fx,A[6+vEqCountt][8]=-(v10-vc0)*x0,A[6+vEqCountt][9]=-(v10-vc0)*y0,A[6+vEqCountt][10]=-(v10-vc0)*z0,A[6+vEqCountt][11]=-(v10-vc0);
		 vEqCountt++;
		 }
	break;
}
  }
  Mat mA=Mat(cvSize(12,12),CV_32FC1,A);
  Mat w,u,vt;
  SVD::compute(mA,w,u,vt);
  int jieIdx=-1;
  float e2MIN=87;
  float normRjie_3;
  float *ptrW=(float *)w.ptr(0);
  float *ptrVt;
  int x=11;//for(int x=0;x<w.rows;x++)
  //{
if(x>=11)
{
	int clins=-1;
	clins++;
}
	  float e2=ptrW[x]*ptrW[x];
	  //误差归一化
	  ptrVt=(float *)vt.ptr(x);
	  normR=ptrVt[0]*ptrVt[5]*ptrVt[10]+ptrVt[1]*ptrVt[6]*ptrVt[8]+ptrVt[4]*ptrVt[9]*ptrVt[2]-ptrVt[2]*ptrVt[5]*ptrVt[8]-ptrVt[1]*ptrVt[4]*ptrVt[10]-ptrVt[6]*ptrVt[9]*ptrVt[0];
	  e2=e2/(normR*normR);
	  //if(e2<e2MIN)
	  {
		  e2MIN=e2;
		  jieIdx=x;
		  if(normR>=0)
		  normRjie_3=pow((double)normR,0.3333);
		  if(normR<0)
		  {
			  normR=-normR;
			  normRjie_3=-pow((double)normR,0.3333);
		  }
	  }
  //}
if(jieIdx==-1)
{
	//解不对 
	return -1;
}
  ptrVt=(float *)vt.ptr(jieIdx);
  R[0][0]=ptrVt[0]/normRjie_3;
  R[0][1]=ptrVt[1]/normRjie_3;
  R[0][2]=ptrVt[2]/normRjie_3;
  t[0]=ptrVt[3]/normRjie_3;
  R[1][0]=ptrVt[4]/normRjie_3;
  R[1][1]=ptrVt[5]/normRjie_3;
  R[1][2]=ptrVt[6]/normRjie_3;
  t[1]=ptrVt[7]/normRjie_3;
  R[2][0]=ptrVt[8]/normRjie_3;
  R[2][1]=ptrVt[9]/normRjie_3;
  R[2][2]=ptrVt[10]/normRjie_3;
  t[2]=ptrVt[11]/normRjie_3;
  //左手坐标系 
  //R[2][0]=sinB;
  sinB=R[2][0];
  b=asin(sinB);
  cosB=cos(b);
  //R[2][1]=-cosB*sinA;
  sinA=-R[2][1]/cosB;
  a=asin(sinA);
  //R[1][0]=-cosB*sinTh;
  sinTh=-R[1][0]/cosB;
  theta=asin(sinTh);
  oX=t[0];
  oY=t[1];
  oZ=t[2];
  
  //record as prior knowledge 
  _tr6[0]=a,_tr6[1]=b,_tr6[2]=theta,_tr6[3]=oX,_tr6[4]=oY,_tr6[5]=oZ;
  
  return -1;
}
int PnPsolver::gauss_newtonZIDINGYI( float R[][3], float *t, float beta)
{

  //init
  const int GradientDiterationsMAX = -1;//87;
  const int iterationsMAX = 9;//6;//87;//4;
  float a=_tr6[0],b=_tr6[1],theta=_tr6[2],oX=_tr6[3],oY=_tr6[4],oZ=_tr6[5];
  float cosA=cos(a),sinA=sin(a),cosB=cos(b),sinB=sin(b),cosTh=cos(theta),sinTh=sin(theta);
  //左手坐标系 
	 R[0][0]=cosTh*cosB;
	 R[0][1]=cosA*sinTh + cosTh*sinA*sinB;
	 R[0][2]=sinTh*sinA - cosTh*cosA*sinB;
	 R[1][0]=-cosB*sinTh;
	 R[1][1]=cosTh*cosA - sinTh*sinA*sinB;
	 R[1][2]=cosTh*sinA + cosA*sinTh*sinB;
	 R[2][0]=sinB;
	 R[2][1]=-cosB*sinA;
	 R[2][2]=cosA*cosB;
    t[0]=oX;
	 t[1]=oY;
	 t[2]=oZ;
  //
  //数据归一化 
  for(int j = 0; j < number_of_correspondences; j++) {
		 float * ptrPws = pws + 3 * j;
		 ptrPws[0]=ptrPws[0];//889;//限x米 
		 ptrPws[1]=ptrPws[1];//889;
		 ptrPws[2]=ptrPws[2];//889;
  }
  uc0=uc0/28;vc0=vc0/28;fx=fx/28;

  //先批量梯度下降
  beta=0.03;
  for(int iter = 0; iter < GradientDiterationsMAX; iter++) {

	 float da=0,db=0,dTh=0,doX=0,doY=0,doZ=0;	 
	 for(int j = 0; j < number_of_correspondences; j++) {
		 
		 float *p0=pws + 3 * j;
		 float x0,y0,z0;
		 x0=p0[0];
		 y0=p0[1];
		 z0=p0[2];
		 float x1,y1,z1;
		 x1= R[0][0]*x0+R[0][1]*y0+R[0][2]*z0 + t[0];
		 y1= R[1][0]*x0+R[1][1]*y0+R[1][2]*z0 + t[1];
		 z1= R[2][0]*x0+R[2][1]*y0+R[2][2]*z0 + t[2];
		 float u1=uc0+fx*x1/z1;
		 float v1=vc0+fx*y1/z1;

		 //e=(u1-u10)的平方+(v1-v10)的平方
		 float *p1=us + 2 * j;
		 float u10=p1[0];
		 float v10=p1[1];
		 ////归一化
		 u10=u10/28;v10=v10/28;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 float errlins[2];
		 errlins[0]=u1-u10;
		 errlins[1]=v1-v10;

		 float uda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1) - (fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1);
		 float udb =- (fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1);
		 float udTh =(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1);
		 float udoX =fx/(z1);
		 float udoY =0;
		 float udoZ =-(fx*(x1))/(z1*z1);
		 float vda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1) - (fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1);
		 float vdb =(fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1);
		 float vdTh =-(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1);
		 float vdoY =fx/(z1);
		 float vdoZ =-(fx*(y1))/(z1*z1);

		 da=da+2*errlins[0]*uda+2*errlins[1]*vda;
		 db=db+2*errlins[0]*udb+2*errlins[1]*vdb;
		 dTh=dTh+2*errlins[0]*udTh+2*errlins[1]*vdTh;
		 doX=doX +2*errlins[0]*udoX;//udoY =0;
		 doY=doY +2*errlins[1]*vdoY;
		 doZ=doZ +2*errlins[0]*udoZ+2*errlins[1]*vdoZ;

	 }
	 da=da/number_of_correspondences;
	 db=db/number_of_correspondences;
	 dTh=dTh/number_of_correspondences;
	 doX=doX/number_of_correspondences;
	 doY=doY/number_of_correspondences;
	 doZ=doZ/number_of_correspondences;

	 //梯度下降法迭代 
	 a=a-beta*da;
	 b=b-beta*db;
	 theta=theta-beta*dTh;
	 oX=oX-beta*doX;
	 oY=oY-beta*doY;
	 oZ=oZ-beta*doZ;
	 
	 //更新R和t
	 cosA=cos(a),sinA=sin(a),cosB=cos(b),sinB=sin(b),cosTh=cos(theta),sinTh=sin(theta);
	 R[0][0]=cosTh*cosB;
	 R[0][1]=cosA*sinTh + cosTh*sinA*sinB;
	 R[0][2]=sinTh*sinA - cosTh*cosA*sinB;
	 R[1][0]=-cosB*sinTh;
	 R[1][1]=cosTh*cosA - sinTh*sinA*sinB;
	 R[1][2]=cosTh*sinA + cosA*sinTh*sinB;
	 R[2][0]=sinB;
	 R[2][1]=-cosB*sinA;
	 R[2][2]=cosA*cosB;
    t[0]=oX;
	 t[1]=oY;
	 t[2]=oZ;
  }

  //再牛顿法 
  beta=0.7;
  for(int iter = 0; iter < iterationsMAX; iter++) {

    /*float err=errCal7(R,t);
	 if(err<errTh)
			break;
			*/
	 /*P1 =
oX + x*r00+y*r01+z*r02
 oY + x*r10+y*r11+z*r12
oZ + x*r20+y*r21+z*r22
1
	 */
	 float da=0,db=0,dTh=0,doX=0,doY=0,doZ=0;
	 //float dt[3];
	 //memset( dt, 0, 3*sizeof(float));
	 float dda=0,da_b=0,da_Th=0,da_oX=0,da_oY=0,da_oZ=0;
	 float db_a=0,ddb=0,db_Th=0,db_oX=0,db_oY=0,db_oZ=0;
	 float dTh_a=0,dTh_b=0,ddTh=0,dTh_oX=0,dTh_oY=0,dTh_oZ=0;
	 float doX_a=0,doX_b=0,doX_Th=0,ddoX=0,doX_oY=0,doX_oZ=0;
	 float doY_a=0,doY_b=0,doY_Th=0,doY_oX=0,ddoY=0,doY_oZ=0;
	 float doZ_a=0,doZ_b=0,doZ_Th=0,doZ_oX=0,doZ_oY=0,ddoZ=0;
	 float ddRt[6][6];
	 memset( ddRt, 0, 6*6*sizeof(float));

	 for(int j = 0; j < number_of_correspondences; j++) {
		 
		 float *p0=pws + 3 * j;
		 float x0,y0,z0;
		 x0=p0[0];
		 y0=p0[1];
		 z0=p0[2];
		 float x1,y1,z1;
		 x1= R[0][0]*x0+R[0][1]*y0+R[0][2]*z0 + t[0];
		 y1= R[1][0]*x0+R[1][1]*y0+R[1][2]*z0 + t[1];
		 z1= R[2][0]*x0+R[2][1]*y0+R[2][2]*z0 + t[2];
		 float u1=uc0+fx*x1/z1;
		 float v1=vc0+fx*y1/z1;

		 //e=(u1-u10)的平方+(v1-v10)的平方
		 float *p1=us + 2 * j;
		 float u10=p1[0];
		 float v10=p1[1];
		 ////归一化
		 u10=u10/28;v10=v10/28;
		 
		 float errlins[2];
		 float uda;
		 float udb;
		 float udTh;
		 float udoX;
		 float udoY;
		 float udoZ;
		 float vda;
		 float vdb;
		 float vdTh;
		 float vdoY;
		 float vdoZ;
		 float udda;
		 float vdda;
		 float uda_b;
		 float vda_b;
		 float uda_Th;
		 float vda_Th;
		 float uda_oX;
		 float vda_oY;
		 float uda_oZ;
		 float vda_oZ;
		 float uddb;
		 float vddb;
		 float udb_Th;
		 float vdb_Th;
		 float udb_oX;
		 float vdb_oY;
		 float udb_oZ;
		 float vdb_oZ;
		 float uddTh;
		 float vddTh;
		 float udTh_oX;
		 float vdTh_oY;
		 float udTh_oZ;
		 float vdTh_oZ;
		 float uddoX;
		 float udoX_oZ;
		 float vdoY_oZ;
		 float uddoZ;
		 float vddoZ;
switch(ifpDX1[j])
{
case pianDX://只x是准的 
		 errlins[0]=u1-u10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 uda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1) - (fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1);
		 udb =- (fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1);
		 udTh =(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1);
		 udoX =fx/(z1);
		 udoY =0;
		 udoZ =-(fx*(x1))/(z1*z1);

		 da=da+2*errlins[0]*uda;
		 db=db+2*errlins[0]*udb;
		 dTh=dTh+2*errlins[0]*udTh;
		 doX=doX +2*errlins[0]*udoX;//udoY =0;
		 //doY=doY +2*errlins[1]*vdoY;
		 doZ=doZ +2*errlins[0]*udoZ;

		 //、、、、、、、、、、、、、、、、、、、求二阶导 
		 udda =(fx*(z0*cosA*cosB - y0*cosB*sinA)*(x1))/(z1*z1) - (fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB)))/(z1) - (2*fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB))*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1) + (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1*z1);
		 uda_b =(fx*(y0*cosTh*cosA*cosB + z0*cosTh*cosB*sinA))/(z1) + (fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB))*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) - (fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1*z1) - (fx*(y0*cosA*sinB + z0*sinA*sinB)*(x1))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1*z1);
		 uda_Th =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1*z1) - (fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1);
		 uda_oX =(fx*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1);
		 uda_oZ =(fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1*z1);
		 uddb =(fx*(x0*sinB + z0*cosA*cosB - y0*cosB*sinA)*(x1))/(z1*z1) - (fx*(x0*cosTh*cosB - z0*cosTh*cosA*sinB + y0*cosTh*sinA*sinB))/(z1) + (2*fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) + (2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1*z1);
		 udb_Th =(fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1) - (fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 udb_oX =-(fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 udb_oZ =(fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1*z1) + (2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1*z1);
		 uddTh =-(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1);
		 udTh_oX =0;
		 udTh_oZ =-(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1*z1);
		 uddoX =0;
		 udoX_oZ =-fx/(z1*z1);
		 uddoZ =(2*fx*(x1))/(z1*z1*z1);
		 
		 dda=2*(uda*uda+errlins[0]*udda);
		 da_b=2*(udb*uda+errlins[0]*uda_b);
		 da_Th=2*(udTh*uda+errlins[0]*uda_Th);
		 da_oX=2*(udoX*uda+errlins[0]*uda_oX);
		 da_oY=0;
		 da_oZ=2*(udoZ*uda+errlins[0]*uda_oZ);
		 //db_a和da_b重复了 
		 ddb=2*(udb*udb+errlins[0]*uddb);
		 db_Th=2*(udTh*udb+errlins[0]*udb_Th);
		 db_oX=2*(udoX*udb+errlins[0]*udb_oX);
		 db_oY=0;
		 db_oZ=2*(udoZ*udb+errlins[0]*udb_oZ);
		 ddTh=2*(udTh*udTh+errlins[0]*uddTh);
		 dTh_oX=2*udoX*udTh;//udTh_oX=0
		 dTh_oY=0;
		 dTh_oZ=2*(udoZ*udTh+errlins[0]*udTh_oZ);
		 ddoX=2*udoX*udoX;//uddoX=0;
		 doX_oY=0;
		 doX_oZ=2*(udoZ*udoX+errlins[0]*udoX_oZ);
		 ddoY=0;
		 doY_oZ=0;
		 ddoZ=2*(udoZ*udoZ+errlins[0]*uddoZ);
		 
		 ddRt[0][0]=ddRt[0][0]+dda;
		 ddRt[0][1]=ddRt[0][1]+da_b;
		 ddRt[0][2]=ddRt[0][2]+da_Th;
		 ddRt[0][3]=ddRt[0][3]+da_oX;
		 ddRt[0][4]=ddRt[0][4]+da_oY;
		 ddRt[0][5]=ddRt[0][5]+da_oZ;
		 ddRt[1][1]=ddRt[1][1]+ddb;
		 ddRt[1][2]=ddRt[1][2]+db_Th;
		 ddRt[1][3]=ddRt[1][3]+db_oX;
		 ddRt[1][4]=ddRt[1][4]+db_oY;
		 ddRt[1][5]=ddRt[1][5]+db_oZ;
		 ddRt[2][2]=ddRt[2][2]+ddTh;
		 ddRt[2][3]=ddRt[2][3]+dTh_oX;
		 ddRt[2][4]=ddRt[2][4]+dTh_oY;
		 ddRt[2][5]=ddRt[2][5]+dTh_oZ;
		 ddRt[3][3]=ddRt[3][3]+ddoX;
		 ddRt[3][4]=0;//doX_oY=0;
		 ddRt[3][5]=ddRt[3][5]+doX_oZ;
		 ddRt[4][4]=ddRt[4][4]+ddoY;
		 ddRt[4][5]=ddRt[4][5]+doY_oZ;
		 ddRt[5][5]=ddRt[5][5]+ddoZ;
	break;
case pianDY:
		 errlins[1]=v1-v10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 vda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1) - (fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1);
		 vdb =(fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1);
		 vdTh =-(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1);
		 vdoY =fx/(z1);
		 vdoZ =-(fx*(y1))/(z1*z1);

		 da=da+2*errlins[1]*vda;
		 db=db+2*errlins[1]*vdb;
		 dTh=dTh+2*errlins[1]*vdTh;
		 //doX=doX +2*errlins[0]*udoX;//udoY =0;
		 doY=doY +2*errlins[1]*vdoY;
		 doZ=doZ +2*errlins[1]*vdoZ;

		 //、、、、、、、、、、、、、、、、、、、求二阶导 
		 vdda =(2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1*z1) - (fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB)))/(z1) + (fx*(z0*cosA*cosB - y0*cosB*sinA)*(y1))/(z1*z1) - (2*fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB))*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1);
		 vda_b =(fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB))*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) - (fx*(y0*cosA*cosB*sinTh + z0*cosB*sinTh*sinA))/(z1) - (fx*(y0*cosA*sinB + z0*sinA*sinB)*(y1))/(z1*z1) + (fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1*z1);
		 vda_Th =(fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1) - (fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1*z1);
		 vda_oY =(fx*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1);
		 vda_oZ =(fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1*z1);
		 vddb =(fx*(x0*cosB*sinTh - z0*cosA*sinTh*sinB + y0*sinTh*sinA*sinB))/(z1) + (fx*(x0*sinB + z0*cosA*cosB - y0*cosB*sinA)*(y1))/(z1*z1) - (2*fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) + (2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1*z1);
		 vdb_Th =(fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1) + (fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 vdb_oY =-(fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 vdb_oZ =(2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1*z1) - (fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1*z1);
		 vddTh =-(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1);
		 vdTh_oY =0;
		 vdTh_oZ =(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1*z1);
		 vdoY_oZ =-fx/(z1*z1);
		 vddoZ =(2*fx*(y1))/(z1*z1*z1);

		 dda=2*(vda*vda+errlins[1]*vdda);
		 da_b=2*(vdb*vda+errlins[1]*vda_b);
		 da_Th=2*(vdTh*vda+errlins[1]*vda_Th);
		 da_oX=0;
		 da_oY=2*(vdoY*vda+errlins[1]*vda_oY);
		 da_oZ=2*(vdoZ*vda+errlins[1]*vda_oZ);
		 //db_a和da_b重复了 
		 ddb=2*(vdb*vdb+errlins[1]*vddb);
		 db_Th=2*(vdTh*vdb+errlins[1]*vdb_Th);
		 db_oX=0;
		 db_oY=2*(vdoY*vdb+errlins[1]*vdb_oY);
		 db_oZ=2*(vdoZ*vdb+errlins[1]*vdb_oZ);
		 ddTh=2*(vdTh*vdTh+errlins[1]*vddTh);
		 dTh_oX=0;
		 dTh_oY=2*vdoY*vdTh;//vdTh_oY=0
		 dTh_oZ=2*(vdoZ*vdTh+errlins[1]*vdTh_oZ);
		 ddoX=0;
		 doX_oY=0;
		 doX_oZ=0;
		 ddoY=2*vdoY*vdoY;
		 doY_oZ=2*(vdoZ*vdoY+errlins[1]*vdoY_oZ);
		 ddoZ=2*(vdoZ*vdoZ+errlins[1]*vddoZ);
		 
		 ddRt[0][0]=ddRt[0][0]+dda;
		 ddRt[0][1]=ddRt[0][1]+da_b;
		 ddRt[0][2]=ddRt[0][2]+da_Th;
		 ddRt[0][3]=ddRt[0][3]+da_oX;
		 ddRt[0][4]=ddRt[0][4]+da_oY;
		 ddRt[0][5]=ddRt[0][5]+da_oZ;
		 ddRt[1][1]=ddRt[1][1]+ddb;
		 ddRt[1][2]=ddRt[1][2]+db_Th;
		 ddRt[1][3]=ddRt[1][3]+db_oX;
		 ddRt[1][4]=ddRt[1][4]+db_oY;
		 ddRt[1][5]=ddRt[1][5]+db_oZ;
		 ddRt[2][2]=ddRt[2][2]+ddTh;
		 ddRt[2][3]=ddRt[2][3]+dTh_oX;
		 ddRt[2][4]=ddRt[2][4]+dTh_oY;
		 ddRt[2][5]=ddRt[2][5]+dTh_oZ;
		 ddRt[3][3]=ddRt[3][3]+ddoX;
		 ddRt[3][4]=0;//doX_oY=0;
		 ddRt[3][5]=ddRt[3][5]+doX_oZ;
		 ddRt[4][4]=ddRt[4][4]+ddoY;
		 ddRt[4][5]=ddRt[4][5]+doY_oZ;
		 ddRt[5][5]=ddRt[5][5]+ddoZ;
	break;
default:
		 errlins[0]=u1-u10;
		 errlins[1]=v1-v10;
		 //、、、、、、、、、、、、、、、、、、、求一阶导
		 uda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1) - (fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1);
		 udb =- (fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1);
		 udTh =(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1);
		 udoX =fx/(z1);
		 udoY =0;
		 udoZ =-(fx*(x1))/(z1*z1);
		 vda =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1) - (fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1);
		 vdb =(fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1) - (fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1);
		 vdTh =-(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1);
		 vdoY =fx/(z1);
		 vdoZ =-(fx*(y1))/(z1*z1);

		 da=da+2*errlins[0]*uda+2*errlins[1]*vda;
		 db=db+2*errlins[0]*udb+2*errlins[1]*vdb;
		 dTh=dTh+2*errlins[0]*udTh+2*errlins[1]*vdTh;
		 doX=doX +2*errlins[0]*udoX;//udoY =0;
		 doY=doY +2*errlins[1]*vdoY;
		 doZ=doZ +2*errlins[0]*udoZ+2*errlins[1]*vdoZ;

		 //、、、、、、、、、、、、、、、、、、、求二阶导 
		 udda =(fx*(z0*cosA*cosB - y0*cosB*sinA)*(x1))/(z1*z1) - (fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB)))/(z1) - (2*fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB))*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1) + (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1*z1);
		 vdda =(2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1*z1) - (fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB)))/(z1) + (fx*(z0*cosA*cosB - y0*cosB*sinA)*(y1))/(z1*z1) - (2*fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB))*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1);
		 uda_b =(fx*(y0*cosTh*cosA*cosB + z0*cosTh*cosB*sinA))/(z1) + (fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB))*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) - (fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1*z1) - (fx*(y0*cosA*sinB + z0*sinA*sinB)*(x1))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1*z1);
		 vda_b =(fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB))*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) - (fx*(y0*cosA*cosB*sinTh + z0*cosB*sinTh*sinA))/(z1) - (fx*(y0*cosA*sinB + z0*sinA*sinB)*(y1))/(z1*z1) + (fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1*z1);
		 uda_Th =(fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1*z1) - (fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1);
		 vda_Th =(fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1) - (fx*(y0*cosA*cosB + z0*cosB*sinA)*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1*z1);
		 uda_oX =(fx*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1);
		 vda_oY =(fx*(y0*cosA*cosB + z0*cosB*sinA))/(z1*z1);
		 uda_oZ =(fx*(y0*(sinTh*sinA - cosTh*cosA*sinB) - z0*(cosA*sinTh + cosTh*sinA*sinB)))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(x1))/(z1*z1*z1);
		 vda_oZ =(fx*(y0*(cosTh*sinA + cosA*sinTh*sinB) - z0*(cosTh*cosA - sinTh*sinA*sinB)))/(z1*z1) - (2*fx*(y0*cosA*cosB + z0*cosB*sinA)*(y1))/(z1*z1*z1);
		 uddb =(fx*(x0*sinB + z0*cosA*cosB - y0*cosB*sinA)*(x1))/(z1*z1) - (fx*(x0*cosTh*cosB - z0*cosTh*cosA*sinB + y0*cosTh*sinA*sinB))/(z1) + (2*fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) + (2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1*z1);
		 vddb =(fx*(x0*cosB*sinTh - z0*cosA*sinTh*sinB + y0*sinTh*sinA*sinB))/(z1) + (fx*(x0*sinB + z0*cosA*cosB - y0*cosB*sinA)*(y1))/(z1*z1) - (2*fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1) + (2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1*z1);
		 udb_Th =(fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1) - (fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 vdb_Th =(fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1) + (fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB)*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 udb_oX =-(fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 vdb_oY =-(fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB))/(z1*z1);
		 udb_oZ =(fx*(x0*cosTh*sinB + z0*cosTh*cosA*cosB - y0*cosTh*cosB*sinA))/(z1*z1) + (2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(x1))/(z1*z1*z1);
		 vdb_oZ =(2*fx*(x0*cosB - z0*cosA*sinB + y0*sinA*sinB)*(y1))/(z1*z1*z1) - (fx*(x0*sinTh*sinB + z0*cosA*cosB*sinTh - y0*cosB*sinTh*sinA))/(z1*z1);
		 uddTh =-(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1);
		 vddTh =-(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1);
		 udTh_oX =0;
		 vdTh_oY =0;
		 udTh_oZ =-(fx*(y0*(cosTh*cosA - sinTh*sinA*sinB) + z0*(cosTh*sinA + cosA*sinTh*sinB) - x0*cosB*sinTh))/(z1*z1);
		 vdTh_oZ =(fx*(y0*(cosA*sinTh + cosTh*sinA*sinB) + z0*(sinTh*sinA - cosTh*cosA*sinB) + x0*cosTh*cosB))/(z1*z1);
		 uddoX =0;
		 udoX_oZ =-fx/(z1*z1);
		 vdoY_oZ =-fx/(z1*z1);
		 uddoZ =(2*fx*(x1))/(z1*z1*z1);
		 vddoZ =(2*fx*(y1))/(z1*z1*z1);

		 dda=2*(uda*uda+errlins[0]*udda) +2*(vda*vda+errlins[1]*vdda);
		 da_b=2*(udb*uda+errlins[0]*uda_b) +2*(vdb*vda+errlins[1]*vda_b);
		 da_Th=2*(udTh*uda+errlins[0]*uda_Th) +2*(vdTh*vda+errlins[1]*vda_Th);
		 da_oX=2*(udoX*uda+errlins[0]*uda_oX);
		 da_oY=2*(vdoY*vda+errlins[1]*vda_oY);
		 da_oZ=2*(udoZ*uda+errlins[0]*uda_oZ) +2*(vdoZ*vda+errlins[1]*vda_oZ);
		 //db_a和da_b重复了 
		 ddb=2*(udb*udb+errlins[0]*uddb) +2*(vdb*vdb+errlins[1]*vddb);
		 db_Th=2*(udTh*udb+errlins[0]*udb_Th) +2*(vdTh*vdb+errlins[1]*vdb_Th);
		 db_oX=2*(udoX*udb+errlins[0]*udb_oX);
		 db_oY=2*(vdoY*vdb+errlins[1]*vdb_oY);
		 db_oZ=2*(udoZ*udb+errlins[0]*udb_oZ) +2*(vdoZ*vdb+errlins[1]*vdb_oZ);
		 ddTh=2*(udTh*udTh+errlins[0]*uddTh) +2*(vdTh*vdTh+errlins[1]*vddTh);
		 dTh_oX=2*udoX*udTh;//udTh_oX=0
		 dTh_oY=2*vdoY*vdTh;//vdTh_oY=0
		 dTh_oZ=2*(udoZ*udTh+errlins[0]*udTh_oZ) +2*(vdoZ*vdTh+errlins[1]*vdTh_oZ);
		 ddoX=2*udoX*udoX;//uddoX=0;
		 doX_oY=0;
		 doX_oZ=2*(udoZ*udoX+errlins[0]*udoX_oZ);
		 ddoY=2*vdoY*vdoY;
		 doY_oZ=2*(vdoZ*vdoY+errlins[1]*vdoY_oZ);
		 ddoZ=2*(udoZ*udoZ+errlins[0]*uddoZ)+2*(vdoZ*vdoZ+errlins[1]*vddoZ);
		 
		 ddRt[0][0]=ddRt[0][0]+dda;
		 ddRt[0][1]=ddRt[0][1]+da_b;
		 ddRt[0][2]=ddRt[0][2]+da_Th;
		 ddRt[0][3]=ddRt[0][3]+da_oX;
		 ddRt[0][4]=ddRt[0][4]+da_oY;
		 ddRt[0][5]=ddRt[0][5]+da_oZ;
		 ddRt[1][1]=ddRt[1][1]+ddb;
		 ddRt[1][2]=ddRt[1][2]+db_Th;
		 ddRt[1][3]=ddRt[1][3]+db_oX;
		 ddRt[1][4]=ddRt[1][4]+db_oY;
		 ddRt[1][5]=ddRt[1][5]+db_oZ;
		 ddRt[2][2]=ddRt[2][2]+ddTh;
		 ddRt[2][3]=ddRt[2][3]+dTh_oX;
		 ddRt[2][4]=ddRt[2][4]+dTh_oY;
		 ddRt[2][5]=ddRt[2][5]+dTh_oZ;
		 ddRt[3][3]=ddRt[3][3]+ddoX;
		 ddRt[3][4]=0;//doX_oY=0;
		 ddRt[3][5]=ddRt[3][5]+doX_oZ;
		 ddRt[4][4]=ddRt[4][4]+ddoY;
		 ddRt[4][5]=ddRt[4][5]+doY_oZ;
		 ddRt[5][5]=ddRt[5][5]+ddoZ;
	break;
}
	 }
	 da=da/number_of_correspondences;
	 db=db/number_of_correspondences;
	 dTh=dTh/number_of_correspondences;
	 doX=doX/number_of_correspondences;
	 doY=doY/number_of_correspondences;
	 doZ=doZ/number_of_correspondences;

	 //梯度下降法迭代 
	 /*a=a-beta*da;
	 b=b-beta*db;
	 theta=theta-beta*dTh;
	 oX=oX-beta*doX;
	 oY=oY-beta*doY;
	 oZ=oZ-beta*doZ;
	 */

	 //阻尼牛顿法迭代
	  for(int m=0;m<6;m++)
			 for(int n=0;n<m;n++)
				 ddRt[m][n]=ddRt[n][m];//利用对称性 
	 for(int m=0;m<6;m++)
		for(int n=0;n<6;n++)
			ddRt[m][n]=ddRt[m][n]/number_of_correspondences;
	/*for(int m=0;m<12;m++)
		ddRt[m][m]=ddRt[m][m]+0.003;//防止除0 、、、、、、、、、、、、、、、、、、、、
	 */
	 float h00=ddRt[0][0];
	 float h01=ddRt[0][1];
	 float h02=ddRt[0][2];
	 float h03=ddRt[0][3];
	 float h04=ddRt[0][4];
	 float h05=ddRt[0][5];
	 float h10=ddRt[1][0];
	 float h11=ddRt[1][1];
	 float h12=ddRt[1][2];
	 float h13=ddRt[1][3];
	 float h14=ddRt[1][4];
	 float h15=ddRt[1][5];
	 float h20=ddRt[2][0];
	 float h21=ddRt[2][1];
	 float h22=ddRt[2][2];
	 float h23=ddRt[2][3];
	 float h24=ddRt[2][4];
	 float h25=ddRt[2][5];
	 float h30=ddRt[3][0];
	 float h31=ddRt[3][1];
	 float h32=ddRt[3][2];
	 float h33=ddRt[3][3];
	 float h34=ddRt[3][4];
	 float h35=ddRt[3][5];
	 float h40=ddRt[4][0];
	 float h41=ddRt[4][1];
	 float h42=ddRt[4][2];
	 float h43=ddRt[4][3];
	 float h44=ddRt[4][4];
	 float h45=ddRt[4][5];
	 float h50=ddRt[5][0];
	 float h51=ddRt[5][1];
	 float h52=ddRt[5][2];
	 float h53=ddRt[5][3];
	 float h54=ddRt[5][4];
	 float h55=ddRt[5][5];
	 float invH[6][6];
	 float divider=- h01*h01*h23*h23*h45*h45 + h44*h55*h01*h01*h23*h23 + 2*h01*h01*h23*h24*h35*h45 - 2*h44*h01*h01*h23*h25*h35 - h01*h01*h24*h24*h35*h35 + h33*h55*h01*h01*h24*h24 - 2*h33*h01*h01*h24*h25*h45 + h33*h44*h01*h01*h25*h25 + h22*h44*h01*h01*h35*h35 + h22*h33*h01*h01*h45*h45 - h22*h33*h44*h55*h01*h01 - 2*h44*h01*h02*h12*h35*h35 - 2*h33*h01*h02*h12*h45*h45 + 2*h33*h44*h55*h01*h02*h12 + 2*h01*h02*h13*h23*h45*h45 - 2*h44*h55*h01*h02*h13*h23 - 2*h01*h02*h13*h24*h35*h45 + 2*h44*h01*h02*h13*h25*h35 - 2*h01*h02*h14*h23*h35*h45 + 2*h01*h02*h14*h24*h35*h35 - 2*h33*h55*h01*h02*h14*h24 + 2*h33*h01*h02*h14*h25*h45 + 2*h44*h01*h02*h15*h23*h35 + 2*h33*h01*h02*h15*h24*h45 - 2*h33*h44*h01*h02*h15*h25 + 2*h01*h03*h12*h23*h45*h45 - 2*h44*h55*h01*h03*h12*h23 - 2*h01*h03*h12*h24*h35*h45 + 2*h44*h01*h03*h12*h25*h35 - 2*h55*h01*h03*h13*h24*h24 + 4*h01*h03*h13*h24*h25*h45 - 2*h44*h01*h03*h13*h25*h25 - 2*h22*h01*h03*h13*h45*h45 + 2*h22*h44*h55*h01*h03*h13 + 2*h55*h01*h03*h14*h23*h24 - 2*h01*h03*h14*h23*h25*h45 - 2*h01*h03*h14*h24*h25*h35 + 2*h22*h01*h03*h14*h35*h45 - 2*h01*h03*h15*h23*h24*h45 + 2*h44*h01*h03*h15*h23*h25 + 2*h01*h03*h15*h24*h24*h35 - 2*h22*h44*h01*h03*h15*h35 - 2*h01*h04*h12*h23*h35*h45 + 2*h01*h04*h12*h24*h35*h35 - 2*h33*h55*h01*h04*h12*h24 + 2*h33*h01*h04*h12*h25*h45 + 2*h55*h01*h04*h13*h23*h24 - 2*h01*h04*h13*h23*h25*h45 - 2*h01*h04*h13*h24*h25*h35 + 2*h22*h01*h04*h13*h35*h45 - 2*h55*h01*h04*h14*h23*h23 + 4*h01*h04*h14*h23*h25*h35 - 2*h33*h01*h04*h14*h25*h25 - 2*h22*h01*h04*h14*h35*h35 + 2*h22*h33*h55*h01*h04*h14 + 2*h01*h04*h15*h23*h23*h45 - 2*h01*h04*h15*h23*h24*h35 + 2*h33*h01*h04*h15*h24*h25 - 2*h22*h33*h01*h04*h15*h45 + 2*h44*h01*h05*h12*h23*h35 + 2*h33*h01*h05*h12*h24*h45 - 2*h33*h44*h01*h05*h12*h25 - 2*h01*h05*h13*h23*h24*h45 + 2*h44*h01*h05*h13*h23*h25 + 2*h01*h05*h13*h24*h24*h35 - 2*h22*h44*h01*h05*h13*h35 + 2*h01*h05*h14*h23*h23*h45 - 2*h01*h05*h14*h23*h24*h35 + 2*h33*h01*h05*h14*h24*h25 - 2*h22*h33*h01*h05*h14*h45 - 2*h44*h01*h05*h15*h23*h23 - 2*h33*h01*h05*h15*h24*h24 + 2*h22*h33*h44*h01*h05*h15 - h02*h02*h13*h13*h45*h45 + h44*h55*h02*h02*h13*h13 + 2*h02*h02*h13*h14*h35*h45 - 2*h44*h02*h02*h13*h15*h35 - h02*h02*h14*h14*h35*h35 + h33*h55*h02*h02*h14*h14 - 2*h33*h02*h02*h14*h15*h45 + h33*h44*h02*h02*h15*h15 + h11*h44*h02*h02*h35*h35 + h11*h33*h02*h02*h45*h45 - h11*h33*h44*h55*h02*h02 + 2*h02*h03*h12*h13*h45*h45 - 2*h44*h55*h02*h03*h12*h13 - 2*h02*h03*h12*h14*h35*h45 + 2*h44*h02*h03*h12*h15*h35 + 2*h55*h02*h03*h13*h14*h24 - 2*h02*h03*h13*h14*h25*h45 - 2*h02*h03*h13*h15*h24*h45 + 2*h44*h02*h03*h13*h15*h25 - 2*h55*h02*h03*h14*h14*h23 + 2*h02*h03*h14*h14*h25*h35 + 4*h02*h03*h14*h15*h23*h45 - 2*h02*h03*h14*h15*h24*h35 - 2*h44*h02*h03*h15*h15*h23 - 2*h11*h02*h03*h23*h45*h45 + 2*h11*h44*h55*h02*h03*h23 + 2*h11*h02*h03*h24*h35*h45 - 2*h11*h44*h02*h03*h25*h35 - 2*h02*h04*h12*h13*h35*h45 + 2*h02*h04*h12*h14*h35*h35 - 2*h33*h55*h02*h04*h12*h14 + 2*h33*h02*h04*h12*h15*h45 - 2*h55*h02*h04*h13*h13*h24 + 2*h02*h04*h13*h13*h25*h45 + 2*h55*h02*h04*h13*h14*h23 - 2*h02*h04*h13*h14*h25*h35 - 2*h02*h04*h13*h15*h23*h45 + 4*h02*h04*h13*h15*h24*h35 - 2*h02*h04*h14*h15*h23*h35 + 2*h33*h02*h04*h14*h15*h25 - 2*h33*h02*h04*h15*h15*h24 + 2*h11*h02*h04*h23*h35*h45 - 2*h11*h02*h04*h24*h35*h35 + 2*h11*h33*h55*h02*h04*h24 - 2*h11*h33*h02*h04*h25*h45 + 2*h44*h02*h05*h12*h13*h35 + 2*h33*h02*h05*h12*h14*h45 - 2*h33*h44*h02*h05*h12*h15 + 2*h02*h05*h13*h13*h24*h45 - 2*h44*h02*h05*h13*h13*h25 - 2*h02*h05*h13*h14*h23*h45 - 2*h02*h05*h13*h14*h24*h35 + 2*h44*h02*h05*h13*h15*h23 + 2*h02*h05*h14*h14*h23*h35 - 2*h33*h02*h05*h14*h14*h25 + 2*h33*h02*h05*h14*h15*h24 - 2*h11*h44*h02*h05*h23*h35 - 2*h11*h33*h02*h05*h24*h45 + 2*h11*h33*h44*h02*h05*h25 - h03*h03*h12*h12*h45*h45 + h44*h55*h03*h03*h12*h12 - 2*h55*h03*h03*h12*h14*h24 + 2*h03*h03*h12*h14*h25*h45 + 2*h03*h03*h12*h15*h24*h45 - 2*h44*h03*h03*h12*h15*h25 - h03*h03*h14*h14*h25*h25 + h22*h55*h03*h03*h14*h14 + 2*h03*h03*h14*h15*h24*h25 - 2*h22*h03*h03*h14*h15*h45 - h03*h03*h15*h15*h24*h24 + h22*h44*h03*h03*h15*h15 + h11*h55*h03*h03*h24*h24 - 2*h11*h03*h03*h24*h25*h45 + h11*h44*h03*h03*h25*h25 + h11*h22*h03*h03*h45*h45 - h11*h22*h44*h55*h03*h03 + 2*h03*h04*h12*h12*h35*h45 + 2*h55*h03*h04*h12*h13*h24 - 2*h03*h04*h12*h13*h25*h45 + 2*h55*h03*h04*h12*h14*h23 - 2*h03*h04*h12*h14*h25*h35 - 2*h03*h04*h12*h15*h23*h45 - 2*h03*h04*h12*h15*h24*h35 + 2*h03*h04*h13*h14*h25*h25 - 2*h22*h55*h03*h04*h13*h14 - 2*h03*h04*h13*h15*h24*h25 + 2*h22*h03*h04*h13*h15*h45 - 2*h03*h04*h14*h15*h23*h25 + 2*h22*h03*h04*h14*h15*h35 + 2*h03*h04*h15*h15*h23*h24 - 2*h11*h55*h03*h04*h23*h24 + 2*h11*h03*h04*h23*h25*h45 + 2*h11*h03*h04*h24*h25*h35 - 2*h11*h22*h03*h04*h35*h45 - 2*h44*h03*h05*h12*h12*h35 - 2*h03*h05*h12*h13*h24*h45 + 2*h44*h03*h05*h12*h13*h25 - 2*h03*h05*h12*h14*h23*h45 + 4*h03*h05*h12*h14*h24*h35 + 2*h44*h03*h05*h12*h15*h23 - 2*h03*h05*h13*h14*h24*h25 + 2*h22*h03*h05*h13*h14*h45 + 2*h03*h05*h13*h15*h24*h24 - 2*h22*h44*h03*h05*h13*h15 + 2*h03*h05*h14*h14*h23*h25 - 2*h22*h03*h05*h14*h14*h35 - 2*h03*h05*h14*h15*h23*h24 + 2*h11*h03*h05*h23*h24*h45 - 2*h11*h44*h03*h05*h23*h25 - 2*h11*h03*h05*h24*h24*h35 + 2*h11*h22*h44*h03*h05*h35 - h04*h04*h12*h12*h35*h35 + h33*h55*h04*h04*h12*h12 - 2*h55*h04*h04*h12*h13*h23 + 2*h04*h04*h12*h13*h25*h35 + 2*h04*h04*h12*h15*h23*h35 - 2*h33*h04*h04*h12*h15*h25 - h04*h04*h13*h13*h25*h25 + h22*h55*h04*h04*h13*h13 + 2*h04*h04*h13*h15*h23*h25 - 2*h22*h04*h04*h13*h15*h35 - h04*h04*h15*h15*h23*h23 + h22*h33*h04*h04*h15*h15 + h11*h55*h04*h04*h23*h23 - 2*h11*h04*h04*h23*h25*h35 + h11*h33*h04*h04*h25*h25 + h11*h22*h04*h04*h35*h35 - h11*h22*h33*h55*h04*h04 - 2*h33*h04*h05*h12*h12*h45 + 4*h04*h05*h12*h13*h23*h45 - 2*h04*h05*h12*h13*h24*h35 - 2*h04*h05*h12*h14*h23*h35 + 2*h33*h04*h05*h12*h14*h25 + 2*h33*h04*h05*h12*h15*h24 + 2*h04*h05*h13*h13*h24*h25 - 2*h22*h04*h05*h13*h13*h45 - 2*h04*h05*h13*h14*h23*h25 + 2*h22*h04*h05*h13*h14*h35 - 2*h04*h05*h13*h15*h23*h24 + 2*h04*h05*h14*h15*h23*h23 - 2*h22*h33*h04*h05*h14*h15 - 2*h11*h04*h05*h23*h23*h45 + 2*h11*h04*h05*h23*h24*h35 - 2*h11*h33*h04*h05*h24*h25 + 2*h11*h22*h33*h04*h05*h45 + h33*h44*h05*h05*h12*h12 - 2*h44*h05*h05*h12*h13*h23 - 2*h33*h05*h05*h12*h14*h24 - h05*h05*h13*h13*h24*h24 + h22*h44*h05*h05*h13*h13 + 2*h05*h05*h13*h14*h23*h24 - h05*h05*h14*h14*h23*h23 + h22*h33*h05*h05*h14*h14 + h11*h44*h05*h05*h23*h23 + h11*h33*h05*h05*h24*h24 - h11*h22*h33*h44*h05*h05 + h00*h44*h12*h12*h35*h35 + h00*h33*h12*h12*h45*h45 - h00*h33*h44*h55*h12*h12 - 2*h00*h12*h13*h23*h45*h45 + 2*h00*h44*h55*h12*h13*h23 + 2*h00*h12*h13*h24*h35*h45 - 2*h00*h44*h12*h13*h25*h35 + 2*h00*h12*h14*h23*h35*h45 - 2*h00*h12*h14*h24*h35*h35 + 2*h00*h33*h55*h12*h14*h24 - 2*h00*h33*h12*h14*h25*h45 - 2*h00*h44*h12*h15*h23*h35 - 2*h00*h33*h12*h15*h24*h45 + 2*h00*h33*h44*h12*h15*h25 + h00*h55*h13*h13*h24*h24 - 2*h00*h13*h13*h24*h25*h45 + h00*h44*h13*h13*h25*h25 + h00*h22*h13*h13*h45*h45 - h00*h22*h44*h55*h13*h13 - 2*h00*h55*h13*h14*h23*h24 + 2*h00*h13*h14*h23*h25*h45 + 2*h00*h13*h14*h24*h25*h35 - 2*h00*h22*h13*h14*h35*h45 + 2*h00*h13*h15*h23*h24*h45 - 2*h00*h44*h13*h15*h23*h25 - 2*h00*h13*h15*h24*h24*h35 + 2*h00*h22*h44*h13*h15*h35 + h00*h55*h14*h14*h23*h23 - 2*h00*h14*h14*h23*h25*h35 + h00*h33*h14*h14*h25*h25 + h00*h22*h14*h14*h35*h35 - h00*h22*h33*h55*h14*h14 - 2*h00*h14*h15*h23*h23*h45 + 2*h00*h14*h15*h23*h24*h35 - 2*h00*h33*h14*h15*h24*h25 + 2*h00*h22*h33*h14*h15*h45 + h00*h44*h15*h15*h23*h23 + h00*h33*h15*h15*h24*h24 - h00*h22*h33*h44*h15*h15 + h00*h11*h23*h23*h45*h45 - h00*h11*h44*h55*h23*h23 - 2*h00*h11*h23*h24*h35*h45 + 2*h00*h11*h44*h23*h25*h35 + h00*h11*h24*h24*h35*h35 - h00*h11*h33*h55*h24*h24 + 2*h00*h11*h33*h24*h25*h45 - h00*h11*h33*h44*h25*h25 - h00*h11*h22*h44*h35*h35 - h00*h11*h22*h33*h45*h45 + h00*h11*h22*h33*h44*h55;
	 invH[0][0]=(h44*h12*h12*h35*h35 + h33*h12*h12*h45*h45 - h33*h44*h55*h12*h12 - 2*h12*h13*h23*h45*h45 + 2*h44*h55*h12*h13*h23 + 2*h12*h13*h24*h35*h45 - 2*h44*h12*h13*h25*h35 + 2*h12*h14*h23*h35*h45 - 2*h12*h14*h24*h35*h35 + 2*h33*h55*h12*h14*h24 - 2*h33*h12*h14*h25*h45 - 2*h44*h12*h15*h23*h35 - 2*h33*h12*h15*h24*h45 + 2*h33*h44*h12*h15*h25 + h55*h13*h13*h24*h24 - 2*h13*h13*h24*h25*h45 + h44*h13*h13*h25*h25 + h22*h13*h13*h45*h45 - h22*h44*h55*h13*h13 - 2*h55*h13*h14*h23*h24 + 2*h13*h14*h23*h25*h45 + 2*h13*h14*h24*h25*h35 - 2*h22*h13*h14*h35*h45 + 2*h13*h15*h23*h24*h45 - 2*h44*h13*h15*h23*h25 - 2*h13*h15*h24*h24*h35 + 2*h22*h44*h13*h15*h35 + h55*h14*h14*h23*h23 - 2*h14*h14*h23*h25*h35 + h33*h14*h14*h25*h25 + h22*h14*h14*h35*h35 - h22*h33*h55*h14*h14 - 2*h14*h15*h23*h23*h45 + 2*h14*h15*h23*h24*h35 - 2*h33*h14*h15*h24*h25 + 2*h22*h33*h14*h15*h45 + h44*h15*h15*h23*h23 + h33*h15*h15*h24*h24 - h22*h33*h44*h15*h15 + h11*h23*h23*h45*h45 - h11*h44*h55*h23*h23 - 2*h11*h23*h24*h35*h45 + 2*h11*h44*h23*h25*h35 + h11*h24*h24*h35*h35 - h11*h33*h55*h24*h24 + 2*h11*h33*h24*h25*h45 - h11*h33*h44*h25*h25 - h11*h22*h44*h35*h35 - h11*h22*h33*h45*h45 + h11*h22*h33*h44*h55)/divider;
	 invH[0][1]=-(h01*h24*h24*h35*h35 + h01*h23*h23*h45*h45 - h02*h14*h24*h35*h35 - h04*h12*h24*h35*h35 + h04*h14*h22*h35*h35 + h04*h14*h25*h25*h33 - h03*h15*h24*h24*h35 - h05*h13*h24*h24*h35 + h05*h15*h24*h24*h33 - h02*h13*h23*h45*h45 - h03*h12*h23*h45*h45 + h03*h13*h22*h45*h45 + h03*h13*h25*h25*h44 - h04*h15*h23*h23*h45 - h05*h14*h23*h23*h45 + h05*h15*h23*h23*h44 + h02*h12*h33*h45*h45 + h02*h12*h35*h35*h44 + h03*h13*h24*h24*h55 + h04*h14*h23*h23*h55 - h01*h22*h33*h45*h45 - h01*h22*h35*h35*h44 - h01*h25*h25*h33*h44 - h01*h24*h24*h33*h55 - h01*h23*h23*h44*h55 + h03*h14*h24*h25*h35 + h04*h13*h24*h25*h35 - 2*h04*h14*h23*h25*h35 + h04*h15*h23*h24*h35 - h04*h15*h24*h25*h33 + h05*h14*h23*h24*h35 - h05*h14*h24*h25*h33 - 2*h03*h13*h24*h25*h45 + h03*h14*h23*h25*h45 + h03*h15*h23*h24*h45 - h03*h15*h23*h25*h44 + h04*h13*h23*h25*h45 + h05*h13*h23*h24*h45 - h05*h13*h23*h25*h44 + h02*h13*h24*h35*h45 - h02*h13*h25*h35*h44 + h02*h14*h23*h35*h45 - h02*h14*h25*h33*h45 - h02*h15*h23*h35*h44 - h02*h15*h24*h33*h45 + h02*h15*h25*h33*h44 + h03*h12*h24*h35*h45 - h03*h12*h25*h35*h44 - h03*h14*h22*h35*h45 - h03*h14*h23*h24*h55 + h03*h15*h22*h35*h44 + h04*h12*h23*h35*h45 - h04*h12*h25*h33*h45 - h04*h13*h22*h35*h45 - h04*h13*h23*h24*h55 + h04*h15*h22*h33*h45 - h05*h12*h23*h35*h44 - h05*h12*h24*h33*h45 + h05*h12*h25*h33*h44 + h05*h13*h22*h35*h44 + h05*h14*h22*h33*h45 - h05*h15*h22*h33*h44 - 2*h01*h23*h24*h35*h45 + 2*h01*h23*h25*h35*h44 + 2*h01*h24*h25*h33*h45 + h02*h14*h24*h33*h55 + h04*h12*h24*h33*h55 - h04*h14*h22*h33*h55 + h02*h13*h23*h44*h55 + h03*h12*h23*h44*h55 - h03*h13*h22*h44*h55 - h02*h12*h33*h44*h55 + h01*h22*h33*h44*h55)/divider;
	 invH[0][2]=-(h02*h14*h14*h35*h35 + h02*h13*h13*h45*h45 - h04*h12*h14*h35*h35 - h03*h12*h13*h45*h45 - h01*h14*h24*h35*h35 + h04*h11*h24*h35*h35 + h04*h15*h15*h24*h33 - h03*h14*h14*h25*h35 - h05*h14*h14*h23*h35 + h05*h14*h14*h25*h33 - h01*h13*h23*h45*h45 + h03*h11*h23*h45*h45 + h03*h15*h15*h23*h44 - h04*h13*h13*h25*h45 - h05*h13*h13*h24*h45 + h05*h13*h13*h25*h44 + h01*h12*h33*h45*h45 - h02*h11*h33*h45*h45 + h01*h12*h35*h35*h44 - h02*h11*h35*h35*h44 - h02*h15*h15*h33*h44 + h03*h14*h14*h23*h55 + h04*h13*h13*h24*h55 - h02*h14*h14*h33*h55 - h02*h13*h13*h44*h55 + h03*h14*h15*h24*h35 + h04*h13*h14*h25*h35 - 2*h04*h13*h15*h24*h35 + h04*h14*h15*h23*h35 - h04*h14*h15*h25*h33 + h05*h13*h14*h24*h35 - h05*h14*h15*h24*h33 + h03*h13*h14*h25*h45 + h03*h13*h15*h24*h45 - h03*h13*h15*h25*h44 - 2*h03*h14*h15*h23*h45 + h04*h13*h15*h23*h45 + h05*h13*h14*h23*h45 - h05*h13*h15*h23*h44 - 2*h02*h13*h14*h35*h45 + 2*h02*h13*h15*h35*h44 + 2*h02*h14*h15*h33*h45 + h03*h12*h14*h35*h45 - h03*h12*h15*h35*h44 - h03*h13*h14*h24*h55 + h04*h12*h13*h35*h45 - h04*h12*h15*h33*h45 - h04*h13*h14*h23*h55 - h05*h12*h13*h35*h44 - h05*h12*h14*h33*h45 + h05*h12*h15*h33*h44 + h01*h13*h24*h35*h45 - h01*h13*h25*h35*h44 + h01*h14*h23*h35*h45 - h01*h14*h25*h33*h45 - h01*h15*h23*h35*h44 - h01*h15*h24*h33*h45 + h01*h15*h25*h33*h44 - h03*h11*h24*h35*h45 + h03*h11*h25*h35*h44 - h04*h11*h23*h35*h45 + h04*h11*h25*h33*h45 + h04*h12*h14*h33*h55 + h05*h11*h23*h35*h44 + h05*h11*h24*h33*h45 - h05*h11*h25*h33*h44 + h01*h14*h24*h33*h55 + h03*h12*h13*h44*h55 - h04*h11*h24*h33*h55 + h01*h13*h23*h44*h55 - h03*h11*h23*h44*h55 - h01*h12*h33*h44*h55 + h02*h11*h33*h44*h55)/divider;
	 invH[0][3]=-(h03*h14*h14*h25*h25 + h03*h15*h15*h24*h24 + h03*h12*h12*h45*h45 - h04*h13*h14*h25*h25 - h05*h13*h15*h24*h24 - h04*h15*h15*h23*h24 - h05*h14*h14*h23*h25 - h02*h12*h13*h45*h45 - h01*h15*h24*h24*h35 + h05*h11*h24*h24*h35 - h02*h14*h14*h25*h35 + h05*h14*h14*h22*h35 - h01*h12*h23*h45*h45 + h01*h13*h22*h45*h45 + h02*h11*h23*h45*h45 - h03*h11*h22*h45*h45 + h01*h13*h25*h25*h44 - h03*h11*h25*h25*h44 + h02*h15*h15*h23*h44 - h03*h15*h15*h22*h44 + h01*h13*h24*h24*h55 - h03*h11*h24*h24*h55 + h02*h14*h14*h23*h55 - h03*h14*h14*h22*h55 - h04*h12*h12*h35*h45 + h05*h12*h12*h35*h44 - h03*h12*h12*h44*h55 - 2*h03*h14*h15*h24*h25 + h04*h13*h15*h24*h25 + h04*h14*h15*h23*h25 + h05*h13*h14*h24*h25 + h05*h14*h15*h23*h24 + h02*h14*h15*h24*h35 + h04*h12*h14*h25*h35 + h04*h12*h15*h24*h35 - h04*h14*h15*h22*h35 - 2*h05*h12*h14*h24*h35 + h01*h14*h24*h25*h35 + h02*h13*h14*h25*h45 + h02*h13*h15*h24*h45 - h02*h13*h15*h25*h44 - 2*h02*h14*h15*h23*h45 - 2*h03*h12*h14*h25*h45 - 2*h03*h12*h15*h24*h45 + 2*h03*h12*h15*h25*h44 + 2*h03*h14*h15*h22*h45 - h04*h11*h24*h25*h35 + h04*h12*h13*h25*h45 + h04*h12*h15*h23*h45 - h04*h13*h15*h22*h45 + h05*h12*h13*h24*h45 - h05*h12*h13*h25*h44 + h05*h12*h14*h23*h45 - h05*h12*h15*h23*h44 - h05*h13*h14*h22*h45 + h05*h13*h15*h22*h44 - 2*h01*h13*h24*h25*h45 + h01*h14*h23*h25*h45 + h01*h15*h23*h24*h45 - h01*h15*h23*h25*h44 + h02*h12*h14*h35*h45 - h02*h12*h15*h35*h44 - h02*h13*h14*h24*h55 + 2*h03*h11*h24*h25*h45 + 2*h03*h12*h14*h24*h55 - h04*h11*h23*h25*h45 - h04*h12*h13*h24*h55 - h04*h12*h14*h23*h55 + h04*h13*h14*h22*h55 - h05*h11*h23*h24*h45 + h05*h11*h23*h25*h44 + h01*h12*h24*h35*h45 - h01*h12*h25*h35*h44 - h01*h14*h22*h35*h45 - h01*h14*h23*h24*h55 + h01*h15*h22*h35*h44 - h02*h11*h24*h35*h45 + h02*h11*h25*h35*h44 + h04*h11*h22*h35*h45 + h04*h11*h23*h24*h55 - h05*h11*h22*h35*h44 + h02*h12*h13*h44*h55 + h01*h12*h23*h44*h55 - h01*h13*h22*h44*h55 - h02*h11*h23*h44*h55 + h03*h11*h22*h44*h55)/divider;
	 invH[0][4]=-(h04*h13*h13*h25*h25 + h04*h15*h15*h23*h23 + h04*h12*h12*h35*h35 - h03*h13*h14*h25*h25 - h05*h14*h15*h23*h23 - h02*h12*h14*h35*h35 - h03*h15*h15*h23*h24 - h05*h13*h13*h24*h25 - h01*h12*h24*h35*h35 + h01*h14*h22*h35*h35 + h02*h11*h24*h35*h35 - h04*h11*h22*h35*h35 + h01*h14*h25*h25*h33 - h04*h11*h25*h25*h33 + h02*h15*h15*h24*h33 - h04*h15*h15*h22*h33 - h01*h15*h23*h23*h45 + h05*h11*h23*h23*h45 - h02*h13*h13*h25*h45 + h05*h13*h13*h22*h45 + h01*h14*h23*h23*h55 - h04*h11*h23*h23*h55 + h02*h13*h13*h24*h55 - h04*h13*h13*h22*h55 - h03*h12*h12*h35*h45 + h05*h12*h12*h33*h45 - h04*h12*h12*h33*h55 + h03*h13*h15*h24*h25 + h03*h14*h15*h23*h25 - 2*h04*h13*h15*h23*h25 + h05*h13*h14*h23*h25 + h05*h13*h15*h23*h24 + h02*h13*h14*h25*h35 - 2*h02*h13*h15*h24*h35 + h02*h14*h15*h23*h35 - h02*h14*h15*h25*h33 + h03*h12*h14*h25*h35 + h03*h12*h15*h24*h35 - h03*h14*h15*h22*h35 - 2*h04*h12*h13*h25*h35 - 2*h04*h12*h15*h23*h35 + 2*h04*h12*h15*h25*h33 + 2*h04*h13*h15*h22*h35 + h05*h12*h13*h24*h35 + h05*h12*h14*h23*h35 - h05*h12*h14*h25*h33 - h05*h12*h15*h24*h33 - h05*h13*h14*h22*h35 + h05*h14*h15*h22*h33 + h01*h13*h24*h25*h35 - 2*h01*h14*h23*h25*h35 + h01*h15*h23*h24*h35 - h01*h15*h24*h25*h33 + h02*h13*h15*h23*h45 - h03*h11*h24*h25*h35 + h03*h12*h13*h25*h45 + h03*h12*h15*h23*h45 - h03*h13*h15*h22*h45 + 2*h04*h11*h23*h25*h35 - h05*h11*h23*h24*h35 + h05*h11*h24*h25*h33 - 2*h05*h12*h13*h23*h45 + h01*h13*h23*h25*h45 + h02*h12*h13*h35*h45 - h02*h12*h15*h33*h45 - h02*h13*h14*h23*h55 - h03*h11*h23*h25*h45 - h03*h12*h13*h24*h55 - h03*h12*h14*h23*h55 + h03*h13*h14*h22*h55 + 2*h04*h12*h13*h23*h55 + h01*h12*h23*h35*h45 - h01*h12*h25*h33*h45 - h01*h13*h22*h35*h45 - h01*h13*h23*h24*h55 + h01*h15*h22*h33*h45 - h02*h11*h23*h35*h45 + h02*h11*h25*h33*h45 + h02*h12*h14*h33*h55 + h03*h11*h22*h35*h45 + h03*h11*h23*h24*h55 - h05*h11*h22*h33*h45 + h01*h12*h24*h33*h55 - h01*h14*h22*h33*h55 - h02*h11*h24*h33*h55 + h04*h11*h22*h33*h55)/divider;
	 invH[0][5]=-(h05*h13*h13*h24*h24 + h05*h14*h14*h23*h23 - h03*h13*h15*h24*h24 - h04*h14*h15*h23*h23 - h03*h14*h14*h23*h25 - h04*h13*h13*h24*h25 - h01*h13*h24*h24*h35 + h01*h15*h24*h24*h33 + h03*h11*h24*h24*h35 - h05*h11*h24*h24*h33 - h02*h14*h14*h23*h35 + h02*h14*h14*h25*h33 + h03*h14*h14*h22*h35 - h05*h14*h14*h22*h33 - h01*h14*h23*h23*h45 + h01*h15*h23*h23*h44 + h04*h11*h23*h23*h45 - h05*h11*h23*h23*h44 - h02*h13*h13*h24*h45 + h02*h13*h13*h25*h44 + h04*h13*h13*h22*h45 - h05*h13*h13*h22*h44 + h03*h12*h12*h35*h44 + h04*h12*h12*h33*h45 - h05*h12*h12*h33*h44 + h03*h13*h14*h24*h25 + h03*h14*h15*h23*h24 + h04*h13*h14*h23*h25 + h04*h13*h15*h23*h24 - 2*h05*h13*h14*h23*h24 + h02*h13*h14*h24*h35 - h02*h14*h15*h24*h33 - 2*h03*h12*h14*h24*h35 + h04*h12*h13*h24*h35 + h04*h12*h14*h23*h35 - h04*h12*h14*h25*h33 - h04*h12*h15*h24*h33 - h04*h13*h14*h22*h35 + h04*h14*h15*h22*h33 + 2*h05*h12*h14*h24*h33 + h01*h14*h23*h24*h35 - h01*h14*h24*h25*h33 + h02*h13*h14*h23*h45 - h02*h13*h15*h23*h44 + h03*h12*h13*h24*h45 - h03*h12*h13*h25*h44 + h03*h12*h14*h23*h45 - h03*h12*h15*h23*h44 - h03*h13*h14*h22*h45 + h03*h13*h15*h22*h44 - h04*h11*h23*h24*h35 + h04*h11*h24*h25*h33 - 2*h04*h12*h13*h23*h45 + 2*h05*h12*h13*h23*h44 + h01*h13*h23*h24*h45 - h01*h13*h23*h25*h44 - h02*h12*h13*h35*h44 - h02*h12*h14*h33*h45 + h02*h12*h15*h33*h44 - h03*h11*h23*h24*h45 + h03*h11*h23*h25*h44 - h01*h12*h23*h35*h44 - h01*h12*h24*h33*h45 + h01*h12*h25*h33*h44 + h01*h13*h22*h35*h44 + h01*h14*h22*h33*h45 - h01*h15*h22*h33*h44 + h02*h11*h23*h35*h44 + h02*h11*h24*h33*h45 - h02*h11*h25*h33*h44 - h03*h11*h22*h35*h44 - h04*h11*h22*h33*h45 + h05*h11*h22*h33*h44)/divider;
	 invH[1][1]=(h44*h02*h02*h35*h35 + h33*h02*h02*h45*h45 - h33*h44*h55*h02*h02 - 2*h02*h03*h23*h45*h45 + 2*h44*h55*h02*h03*h23 + 2*h02*h03*h24*h35*h45 - 2*h44*h02*h03*h25*h35 + 2*h02*h04*h23*h35*h45 - 2*h02*h04*h24*h35*h35 + 2*h33*h55*h02*h04*h24 - 2*h33*h02*h04*h25*h45 - 2*h44*h02*h05*h23*h35 - 2*h33*h02*h05*h24*h45 + 2*h33*h44*h02*h05*h25 + h55*h03*h03*h24*h24 - 2*h03*h03*h24*h25*h45 + h44*h03*h03*h25*h25 + h22*h03*h03*h45*h45 - h22*h44*h55*h03*h03 - 2*h55*h03*h04*h23*h24 + 2*h03*h04*h23*h25*h45 + 2*h03*h04*h24*h25*h35 - 2*h22*h03*h04*h35*h45 + 2*h03*h05*h23*h24*h45 - 2*h44*h03*h05*h23*h25 - 2*h03*h05*h24*h24*h35 + 2*h22*h44*h03*h05*h35 + h55*h04*h04*h23*h23 - 2*h04*h04*h23*h25*h35 + h33*h04*h04*h25*h25 + h22*h04*h04*h35*h35 - h22*h33*h55*h04*h04 - 2*h04*h05*h23*h23*h45 + 2*h04*h05*h23*h24*h35 - 2*h33*h04*h05*h24*h25 + 2*h22*h33*h04*h05*h45 + h44*h05*h05*h23*h23 + h33*h05*h05*h24*h24 - h22*h33*h44*h05*h05 + h00*h23*h23*h45*h45 - h00*h44*h55*h23*h23 - 2*h00*h23*h24*h35*h45 + 2*h00*h44*h23*h25*h35 + h00*h24*h24*h35*h35 - h00*h33*h55*h24*h24 + 2*h00*h33*h24*h25*h45 - h00*h33*h44*h25*h25 - h00*h22*h44*h35*h35 - h00*h22*h33*h45*h45 + h00*h22*h33*h44*h55)/divider;
	 invH[1][2]=-(h04*h04*h12*h35*h35 + h03*h03*h12*h45*h45 - h02*h04*h14*h35*h35 - h02*h03*h13*h45*h45 - h01*h04*h24*h35*h35 - h01*h03*h23*h45*h45 + h00*h14*h24*h35*h35 + h05*h05*h14*h24*h33 - h04*h04*h13*h25*h35 - h04*h04*h15*h23*h35 + h04*h04*h15*h25*h33 + h00*h13*h23*h45*h45 + h01*h02*h33*h45*h45 + h01*h02*h35*h35*h44 + h05*h05*h13*h23*h44 - h03*h03*h14*h25*h45 - h03*h03*h15*h24*h45 + h03*h03*h15*h25*h44 - h00*h12*h33*h45*h45 - h00*h12*h35*h35*h44 - h05*h05*h12*h33*h44 + h04*h04*h13*h23*h55 + h03*h03*h14*h24*h55 - h04*h04*h12*h33*h55 - h03*h03*h12*h44*h55 + h03*h04*h14*h25*h35 + h03*h04*h15*h24*h35 - 2*h03*h05*h14*h24*h35 + h04*h05*h13*h24*h35 + h04*h05*h14*h23*h35 - h04*h05*h14*h25*h33 - h04*h05*h15*h24*h33 + h03*h04*h13*h25*h45 + h03*h04*h15*h23*h45 + h03*h05*h13*h24*h45 - h03*h05*h13*h25*h44 + h03*h05*h14*h23*h45 - h03*h05*h15*h23*h44 - 2*h04*h05*h13*h23*h45 + h02*h03*h14*h35*h45 - h02*h03*h15*h35*h44 + h02*h04*h13*h35*h45 - h02*h04*h15*h33*h45 - h02*h05*h13*h35*h44 - h02*h05*h14*h33*h45 + h02*h05*h15*h33*h44 - 2*h03*h04*h12*h35*h45 - h03*h04*h13*h24*h55 - h03*h04*h14*h23*h55 + 2*h03*h05*h12*h35*h44 + 2*h04*h05*h12*h33*h45 + h01*h03*h24*h35*h45 - h01*h03*h25*h35*h44 + h01*h04*h23*h35*h45 - h01*h04*h25*h33*h45 - h01*h05*h23*h35*h44 - h01*h05*h24*h33*h45 + h01*h05*h25*h33*h44 + h02*h04*h14*h33*h55 - h00*h13*h24*h35*h45 + h00*h13*h25*h35*h44 - h00*h14*h23*h35*h45 + h00*h14*h25*h33*h45 + h00*h15*h23*h35*h44 + h00*h15*h24*h33*h45 - h00*h15*h25*h33*h44 + h01*h04*h24*h33*h55 + h02*h03*h13*h44*h55 - h00*h14*h24*h33*h55 + h01*h03*h23*h44*h55 - h00*h13*h23*h44*h55 - h01*h02*h33*h44*h55 + h00*h12*h33*h44*h55)/divider;
	 invH[1][3]=-(h04*h04*h13*h25*h25 + h05*h05*h13*h24*h24 + h02*h02*h13*h45*h45 - h03*h04*h14*h25*h25 - h03*h05*h15*h24*h24 - h02*h03*h12*h45*h45 - h01*h05*h24*h24*h35 - h05*h05*h14*h23*h24 - h04*h04*h15*h23*h25 - h01*h02*h23*h45*h45 + h01*h03*h22*h45*h45 + h01*h03*h25*h25*h44 + h00*h15*h24*h24*h35 - h04*h04*h12*h25*h35 + h04*h04*h15*h22*h35 + h00*h12*h23*h45*h45 - h00*h13*h22*h45*h45 - h00*h13*h25*h25*h44 + h01*h03*h24*h24*h55 + h05*h05*h12*h23*h44 - h05*h05*h13*h22*h44 - h00*h13*h24*h24*h55 + h04*h04*h12*h23*h55 - h04*h04*h13*h22*h55 - h02*h02*h14*h35*h45 + h02*h02*h15*h35*h44 - h02*h02*h13*h44*h55 + h03*h04*h15*h24*h25 + h03*h05*h14*h24*h25 - 2*h04*h05*h13*h24*h25 + h04*h05*h14*h23*h25 + h04*h05*h15*h23*h24 + h02*h04*h14*h25*h35 - 2*h02*h04*h15*h24*h35 + h02*h05*h14*h24*h35 + h04*h05*h12*h24*h35 - h04*h05*h14*h22*h35 + h01*h04*h24*h25*h35 + h02*h03*h14*h25*h45 + h02*h03*h15*h24*h45 - h02*h03*h15*h25*h44 - 2*h02*h04*h13*h25*h45 + h02*h04*h15*h23*h45 - 2*h02*h05*h13*h24*h45 + 2*h02*h05*h13*h25*h44 + h02*h05*h14*h23*h45 - h02*h05*h15*h23*h44 + h03*h04*h12*h25*h45 - h03*h04*h15*h22*h45 + h03*h05*h12*h24*h45 - h03*h05*h12*h25*h44 - h03*h05*h14*h22*h45 + h03*h05*h15*h22*h44 - 2*h04*h05*h12*h23*h45 + 2*h04*h05*h13*h22*h45 - h00*h14*h24*h25*h35 - 2*h01*h03*h24*h25*h45 + h01*h04*h23*h25*h45 + h01*h05*h23*h24*h45 - h01*h05*h23*h25*h44 - h02*h03*h14*h24*h55 + h02*h04*h12*h35*h45 + 2*h02*h04*h13*h24*h55 - h02*h04*h14*h23*h55 - h02*h05*h12*h35*h44 - h03*h04*h12*h24*h55 + h03*h04*h14*h22*h55 + 2*h00*h13*h24*h25*h45 - h00*h14*h23*h25*h45 - h00*h15*h23*h24*h45 + h00*h15*h23*h25*h44 + h01*h02*h24*h35*h45 - h01*h02*h25*h35*h44 - h01*h04*h22*h35*h45 - h01*h04*h23*h24*h55 + h01*h05*h22*h35*h44 - h00*h12*h24*h35*h45 + h00*h12*h25*h35*h44 + h00*h14*h22*h35*h45 + h00*h14*h23*h24*h55 - h00*h15*h22*h35*h44 + h02*h03*h12*h44*h55 + h01*h02*h23*h44*h55 - h01*h03*h22*h44*h55 - h00*h12*h23*h44*h55 + h00*h13*h22*h44*h55)/divider;
	 invH[1][4]=-(h03*h03*h14*h25*h25 + h05*h05*h14*h23*h23 + h02*h02*h14*h35*h35 - h03*h04*h13*h25*h25 - h04*h05*h15*h23*h23 - h02*h04*h12*h35*h35 - h01*h02*h24*h35*h35 + h01*h04*h22*h35*h35 + h01*h04*h25*h25*h33 - h05*h05*h13*h23*h24 - h03*h03*h15*h24*h25 + h00*h12*h24*h35*h35 - h00*h14*h22*h35*h35 - h00*h14*h25*h25*h33 - h01*h05*h23*h23*h45 + h05*h05*h12*h24*h33 - h05*h05*h14*h22*h33 + h00*h15*h23*h23*h45 + h01*h04*h23*h23*h55 - h03*h03*h12*h25*h45 + h03*h03*h15*h22*h45 - h00*h14*h23*h23*h55 + h03*h03*h12*h24*h55 - h03*h03*h14*h22*h55 - h02*h02*h13*h35*h45 + h02*h02*h15*h33*h45 - h02*h02*h14*h33*h55 + h03*h04*h15*h23*h25 + h03*h05*h13*h24*h25 - 2*h03*h05*h14*h23*h25 + h03*h05*h15*h23*h24 + h04*h05*h13*h23*h25 - 2*h02*h03*h14*h25*h35 + h02*h03*h15*h24*h35 + h02*h04*h13*h25*h35 + h02*h04*h15*h23*h35 - h02*h04*h15*h25*h33 + h02*h05*h13*h24*h35 - 2*h02*h05*h14*h23*h35 + 2*h02*h05*h14*h25*h33 - h02*h05*h15*h24*h33 + h03*h04*h12*h25*h35 - h03*h04*h15*h22*h35 - 2*h03*h05*h12*h24*h35 + 2*h03*h05*h14*h22*h35 + h04*h05*h12*h23*h35 - h04*h05*h12*h25*h33 - h04*h05*h13*h22*h35 + h04*h05*h15*h22*h33 + h01*h03*h24*h25*h35 - 2*h01*h04*h23*h25*h35 + h01*h05*h23*h24*h35 - h01*h05*h24*h25*h33 + h02*h03*h13*h25*h45 - 2*h02*h03*h15*h23*h45 + h02*h05*h13*h23*h45 + h03*h05*h12*h23*h45 - h03*h05*h13*h22*h45 - h00*h13*h24*h25*h35 + 2*h00*h14*h23*h25*h35 - h00*h15*h23*h24*h35 + h00*h15*h24*h25*h33 + h01*h03*h23*h25*h45 + h02*h03*h12*h35*h45 - h02*h03*h13*h24*h55 + 2*h02*h03*h14*h23*h55 - h02*h04*h13*h23*h55 - h02*h05*h12*h33*h45 - h03*h04*h12*h23*h55 + h03*h04*h13*h22*h55 - h00*h13*h23*h25*h45 + h01*h02*h23*h35*h45 - h01*h02*h25*h33*h45 - h01*h03*h22*h35*h45 - h01*h03*h23*h24*h55 + h01*h05*h22*h33*h45 + h02*h04*h12*h33*h55 - h00*h12*h23*h35*h45 + h00*h12*h25*h33*h45 + h00*h13*h22*h35*h45 + h00*h13*h23*h24*h55 - h00*h15*h22*h33*h45 + h01*h02*h24*h33*h55 - h01*h04*h22*h33*h55 - h00*h12*h24*h33*h55 + h00*h14*h22*h33*h55)/divider;
	 invH[1][5]=-(h03*h03*h15*h24*h24 + h04*h04*h15*h23*h23 - h03*h05*h13*h24*h24 - h04*h05*h14*h23*h23 - h01*h03*h24*h24*h35 + h01*h05*h24*h24*h33 - h04*h04*h13*h23*h25 - h03*h03*h14*h24*h25 + h00*h13*h24*h24*h35 - h00*h15*h24*h24*h33 - h01*h04*h23*h23*h45 + h01*h05*h23*h23*h44 - h04*h04*h12*h23*h35 + h04*h04*h12*h25*h33 + h04*h04*h13*h22*h35 - h04*h04*h15*h22*h33 + h00*h14*h23*h23*h45 - h00*h15*h23*h23*h44 - h03*h03*h12*h24*h45 + h03*h03*h12*h25*h44 + h03*h03*h14*h22*h45 - h03*h03*h15*h22*h44 + h02*h02*h13*h35*h44 + h02*h02*h14*h33*h45 - h02*h02*h15*h33*h44 + h03*h04*h13*h24*h25 + h03*h04*h14*h23*h25 - 2*h03*h04*h15*h23*h24 + h03*h05*h14*h23*h24 + h04*h05*h13*h23*h24 + h02*h03*h14*h24*h35 - 2*h02*h04*h13*h24*h35 + h02*h04*h14*h23*h35 - h02*h04*h14*h25*h33 + 2*h02*h04*h15*h24*h33 - h02*h05*h14*h24*h33 + h03*h04*h12*h24*h35 - h03*h04*h14*h22*h35 - h04*h05*h12*h24*h33 + h04*h05*h14*h22*h33 + h01*h04*h23*h24*h35 - h01*h04*h24*h25*h33 + h02*h03*h13*h24*h45 - h02*h03*h13*h25*h44 - 2*h02*h03*h14*h23*h45 + 2*h02*h03*h15*h23*h44 + h02*h04*h13*h23*h45 - h02*h05*h13*h23*h44 + h03*h04*h12*h23*h45 - h03*h04*h13*h22*h45 - h03*h05*h12*h23*h44 + h03*h05*h13*h22*h44 - h00*h14*h23*h24*h35 + h00*h14*h24*h25*h33 + h01*h03*h23*h24*h45 - h01*h03*h23*h25*h44 - h02*h03*h12*h35*h44 - h02*h04*h12*h33*h45 + h02*h05*h12*h33*h44 - h00*h13*h23*h24*h45 + h00*h13*h23*h25*h44 - h01*h02*h23*h35*h44 - h01*h02*h24*h33*h45 + h01*h02*h25*h33*h44 + h01*h03*h22*h35*h44 + h01*h04*h22*h33*h45 - h01*h05*h22*h33*h44 + h00*h12*h23*h35*h44 + h00*h12*h24*h33*h45 - h00*h12*h25*h33*h44 - h00*h13*h22*h35*h44 - h00*h14*h22*h33*h45 + h00*h15*h22*h33*h44)/divider;
	 invH[2][2]=(h44*h01*h01*h35*h35 + h33*h01*h01*h45*h45 - h33*h44*h55*h01*h01 - 2*h01*h03*h13*h45*h45 + 2*h44*h55*h01*h03*h13 + 2*h01*h03*h14*h35*h45 - 2*h44*h01*h03*h15*h35 + 2*h01*h04*h13*h35*h45 - 2*h01*h04*h14*h35*h35 + 2*h33*h55*h01*h04*h14 - 2*h33*h01*h04*h15*h45 - 2*h44*h01*h05*h13*h35 - 2*h33*h01*h05*h14*h45 + 2*h33*h44*h01*h05*h15 + h55*h03*h03*h14*h14 - 2*h03*h03*h14*h15*h45 + h44*h03*h03*h15*h15 + h11*h03*h03*h45*h45 - h11*h44*h55*h03*h03 - 2*h55*h03*h04*h13*h14 + 2*h03*h04*h13*h15*h45 + 2*h03*h04*h14*h15*h35 - 2*h11*h03*h04*h35*h45 + 2*h03*h05*h13*h14*h45 - 2*h44*h03*h05*h13*h15 - 2*h03*h05*h14*h14*h35 + 2*h11*h44*h03*h05*h35 + h55*h04*h04*h13*h13 - 2*h04*h04*h13*h15*h35 + h33*h04*h04*h15*h15 + h11*h04*h04*h35*h35 - h11*h33*h55*h04*h04 - 2*h04*h05*h13*h13*h45 + 2*h04*h05*h13*h14*h35 - 2*h33*h04*h05*h14*h15 + 2*h11*h33*h04*h05*h45 + h44*h05*h05*h13*h13 + h33*h05*h05*h14*h14 - h11*h33*h44*h05*h05 + h00*h13*h13*h45*h45 - h00*h44*h55*h13*h13 - 2*h00*h13*h14*h35*h45 + 2*h00*h44*h13*h15*h35 + h00*h14*h14*h35*h35 - h00*h33*h55*h14*h14 + 2*h00*h33*h14*h15*h45 - h00*h33*h44*h15*h15 - h00*h11*h44*h35*h35 - h00*h11*h33*h45*h45 + h00*h11*h33*h44*h55)/divider;
	 invH[2][3]=-(h04*h04*h15*h15*h23 + h05*h05*h14*h14*h23 + h01*h01*h23*h45*h45 - h03*h04*h15*h15*h24 - h03*h05*h14*h14*h25 - h02*h05*h14*h14*h35 - h05*h05*h13*h14*h24 - h04*h04*h13*h15*h25 - h01*h02*h13*h45*h45 - h01*h03*h12*h45*h45 + h02*h03*h11*h45*h45 + h02*h03*h15*h15*h44 - h04*h04*h12*h15*h35 + h00*h12*h13*h45*h45 + h00*h14*h14*h25*h35 + h02*h03*h14*h14*h55 + h05*h05*h12*h13*h44 + h04*h04*h11*h25*h35 - h00*h11*h23*h45*h45 - h00*h15*h15*h23*h44 - h05*h05*h11*h23*h44 + h04*h04*h12*h13*h55 - h00*h14*h14*h23*h55 - h04*h04*h11*h23*h55 - h01*h01*h24*h35*h45 + h01*h01*h25*h35*h44 - h01*h01*h23*h44*h55 + h03*h04*h14*h15*h25 + h03*h05*h14*h15*h24 + h04*h05*h13*h14*h25 + h04*h05*h13*h15*h24 - 2*h04*h05*h14*h15*h23 + h02*h04*h14*h15*h35 + h04*h05*h12*h14*h35 - 2*h01*h04*h14*h25*h35 + h01*h04*h15*h24*h35 + h01*h05*h14*h24*h35 - 2*h02*h03*h14*h15*h45 + h02*h04*h13*h15*h45 + h02*h05*h13*h14*h45 - h02*h05*h13*h15*h44 + h03*h04*h12*h15*h45 + h03*h05*h12*h14*h45 - h03*h05*h12*h15*h44 - h04*h05*h11*h24*h35 - 2*h04*h05*h12*h13*h45 - h00*h14*h15*h24*h35 + h01*h03*h14*h25*h45 + h01*h03*h15*h24*h45 - h01*h03*h15*h25*h44 + h01*h04*h13*h25*h45 - 2*h01*h04*h15*h23*h45 + h01*h05*h13*h24*h45 - h01*h05*h13*h25*h44 - 2*h01*h05*h14*h23*h45 + 2*h01*h05*h15*h23*h44 - h02*h04*h13*h14*h55 - h03*h04*h11*h25*h45 - h03*h04*h12*h14*h55 - h03*h05*h11*h24*h45 + h03*h05*h11*h25*h44 + 2*h04*h05*h11*h23*h45 - h00*h13*h14*h25*h45 - h00*h13*h15*h24*h45 + h00*h13*h15*h25*h44 + 2*h00*h14*h15*h23*h45 + h01*h02*h14*h35*h45 - h01*h02*h15*h35*h44 - h01*h03*h14*h24*h55 + h01*h04*h12*h35*h45 - h01*h04*h13*h24*h55 + 2*h01*h04*h14*h23*h55 - h01*h05*h12*h35*h44 - h02*h04*h11*h35*h45 + h02*h05*h11*h35*h44 + h03*h04*h11*h24*h55 - h00*h12*h14*h35*h45 + h00*h12*h15*h35*h44 + h00*h13*h14*h24*h55 + h00*h11*h24*h35*h45 - h00*h11*h25*h35*h44 + h01*h02*h13*h44*h55 + h01*h03*h12*h44*h55 - h02*h03*h11*h44*h55 - h00*h12*h13*h44*h55 + h00*h11*h23*h44*h55)/divider;
	 invH[2][4]=-(h03*h03*h15*h15*h24 + h05*h05*h13*h13*h24 + h01*h01*h24*h35*h35 - h03*h04*h15*h15*h23 - h04*h05*h13*h13*h25 - h01*h02*h14*h35*h35 - h01*h04*h12*h35*h35 + h02*h04*h11*h35*h35 + h02*h04*h15*h15*h33 - h05*h05*h13*h14*h23 - h03*h03*h14*h15*h25 + h00*h12*h14*h35*h35 + h05*h05*h12*h14*h33 - h02*h05*h13*h13*h45 - h00*h11*h24*h35*h35 - h00*h15*h15*h24*h33 - h05*h05*h11*h24*h33 + h02*h04*h13*h13*h55 - h03*h03*h12*h15*h45 + h00*h13*h13*h25*h45 + h03*h03*h11*h25*h45 + h03*h03*h12*h14*h55 - h00*h13*h13*h24*h55 - h03*h03*h11*h24*h55 - h01*h01*h23*h35*h45 + h01*h01*h25*h33*h45 - h01*h01*h24*h33*h55 + h03*h04*h13*h15*h25 + h03*h05*h13*h14*h25 - 2*h03*h05*h13*h15*h24 + h03*h05*h14*h15*h23 + h04*h05*h13*h15*h23 + h02*h03*h14*h15*h35 - 2*h02*h04*h13*h15*h35 + h02*h05*h13*h14*h35 - h02*h05*h14*h15*h33 + h03*h04*h12*h15*h35 - 2*h03*h05*h12*h14*h35 + h04*h05*h12*h13*h35 - h04*h05*h12*h15*h33 + h01*h03*h14*h25*h35 - 2*h01*h03*h15*h24*h35 + h01*h04*h13*h25*h35 + h01*h04*h15*h23*h35 - h01*h04*h15*h25*h33 - 2*h01*h05*h13*h24*h35 + h01*h05*h14*h23*h35 - h01*h05*h14*h25*h33 + 2*h01*h05*h15*h24*h33 + h02*h03*h13*h15*h45 - h03*h04*h11*h25*h35 + 2*h03*h05*h11*h24*h35 + h03*h05*h12*h13*h45 - h04*h05*h11*h23*h35 + h04*h05*h11*h25*h33 - h00*h13*h14*h25*h35 + 2*h00*h13*h15*h24*h35 - h00*h14*h15*h23*h35 + h00*h14*h15*h25*h33 - 2*h01*h03*h13*h25*h45 + h01*h03*h15*h23*h45 + h01*h05*h13*h23*h45 - h02*h03*h13*h14*h55 - h03*h04*h12*h13*h55 - h03*h05*h11*h23*h45 - h00*h13*h15*h23*h45 + h01*h02*h13*h35*h45 - h01*h02*h15*h33*h45 + h01*h03*h12*h35*h45 + 2*h01*h03*h13*h24*h55 - h01*h03*h14*h23*h55 - h01*h04*h13*h23*h55 - h01*h05*h12*h33*h45 - h02*h03*h11*h35*h45 + h02*h05*h11*h33*h45 + h03*h04*h11*h23*h55 - h00*h12*h13*h35*h45 + h00*h12*h15*h33*h45 + h00*h13*h14*h23*h55 + h01*h02*h14*h33*h55 + h01*h04*h12*h33*h55 - h02*h04*h11*h33*h55 + h00*h11*h23*h35*h45 - h00*h11*h25*h33*h45 - h00*h12*h14*h33*h55 + h00*h11*h24*h33*h55)/divider;
	 invH[2][5]=-(h03*h03*h14*h14*h25 + h04*h04*h13*h13*h25 - h03*h05*h14*h14*h23 - h04*h05*h13*h13*h24 - h02*h03*h14*h14*h35 + h02*h05*h14*h14*h33 - h04*h04*h13*h15*h23 - h03*h03*h14*h15*h24 - h02*h04*h13*h13*h45 + h02*h05*h13*h13*h44 - h04*h04*h12*h13*h35 + h04*h04*h12*h15*h33 + h00*h14*h14*h23*h35 - h00*h14*h14*h25*h33 + h04*h04*h11*h23*h35 - h04*h04*h11*h25*h33 - h03*h03*h12*h14*h45 + h03*h03*h12*h15*h44 + h00*h13*h13*h24*h45 - h00*h13*h13*h25*h44 + h03*h03*h11*h24*h45 - h03*h03*h11*h25*h44 + h01*h01*h23*h35*h44 + h01*h01*h24*h33*h45 - h01*h01*h25*h33*h44 - 2*h03*h04*h13*h14*h25 + h03*h04*h13*h15*h24 + h03*h04*h14*h15*h23 + h03*h05*h13*h14*h24 + h04*h05*h13*h14*h23 + h02*h04*h13*h14*h35 - h02*h04*h14*h15*h33 + h03*h04*h12*h14*h35 - h04*h05*h12*h14*h33 + h01*h03*h14*h24*h35 + h01*h04*h13*h24*h35 - 2*h01*h04*h14*h23*h35 + 2*h01*h04*h14*h25*h33 - h01*h04*h15*h24*h33 - h01*h05*h14*h24*h33 + h02*h03*h13*h14*h45 - h02*h03*h13*h15*h44 - h03*h04*h11*h24*h35 + h03*h04*h12*h13*h45 - h03*h05*h12*h13*h44 + h04*h05*h11*h24*h33 - h00*h13*h14*h24*h35 + h00*h14*h15*h24*h33 - 2*h01*h03*h13*h24*h45 + 2*h01*h03*h13*h25*h44 + h01*h03*h14*h23*h45 - h01*h03*h15*h23*h44 + h01*h04*h13*h23*h45 - h01*h05*h13*h23*h44 - h03*h04*h11*h23*h45 + h03*h05*h11*h23*h44 - h00*h13*h14*h23*h45 + h00*h13*h15*h23*h44 - h01*h02*h13*h35*h44 - h01*h02*h14*h33*h45 + h01*h02*h15*h33*h44 - h01*h03*h12*h35*h44 - h01*h04*h12*h33*h45 + h01*h05*h12*h33*h44 + h02*h03*h11*h35*h44 + h02*h04*h11*h33*h45 - h02*h05*h11*h33*h44 + h00*h12*h13*h35*h44 + h00*h12*h14*h33*h45 - h00*h12*h15*h33*h44 - h00*h11*h23*h35*h44 - h00*h11*h24*h33*h45 + h00*h11*h25*h33*h44)/divider;
	 invH[3][3]=(h55*h01*h01*h24*h24 - 2*h01*h01*h24*h25*h45 + h44*h01*h01*h25*h25 + h22*h01*h01*h45*h45 - h22*h44*h55*h01*h01 - 2*h01*h02*h12*h45*h45 + 2*h44*h55*h01*h02*h12 - 2*h55*h01*h02*h14*h24 + 2*h01*h02*h14*h25*h45 + 2*h01*h02*h15*h24*h45 - 2*h44*h01*h02*h15*h25 - 2*h55*h01*h04*h12*h24 + 2*h01*h04*h12*h25*h45 - 2*h01*h04*h14*h25*h25 + 2*h22*h55*h01*h04*h14 + 2*h01*h04*h15*h24*h25 - 2*h22*h01*h04*h15*h45 + 2*h01*h05*h12*h24*h45 - 2*h44*h01*h05*h12*h25 + 2*h01*h05*h14*h24*h25 - 2*h22*h01*h05*h14*h45 - 2*h01*h05*h15*h24*h24 + 2*h22*h44*h01*h05*h15 + h55*h02*h02*h14*h14 - 2*h02*h02*h14*h15*h45 + h44*h02*h02*h15*h15 + h11*h02*h02*h45*h45 - h11*h44*h55*h02*h02 - 2*h55*h02*h04*h12*h14 + 2*h02*h04*h12*h15*h45 + 2*h02*h04*h14*h15*h25 - 2*h02*h04*h15*h15*h24 + 2*h11*h55*h02*h04*h24 - 2*h11*h02*h04*h25*h45 + 2*h02*h05*h12*h14*h45 - 2*h44*h02*h05*h12*h15 - 2*h02*h05*h14*h14*h25 + 2*h02*h05*h14*h15*h24 - 2*h11*h02*h05*h24*h45 + 2*h11*h44*h02*h05*h25 + h55*h04*h04*h12*h12 - 2*h04*h04*h12*h15*h25 + h22*h04*h04*h15*h15 + h11*h04*h04*h25*h25 - h11*h22*h55*h04*h04 - 2*h04*h05*h12*h12*h45 + 2*h04*h05*h12*h14*h25 + 2*h04*h05*h12*h15*h24 - 2*h22*h04*h05*h14*h15 - 2*h11*h04*h05*h24*h25 + 2*h11*h22*h04*h05*h45 + h44*h05*h05*h12*h12 - 2*h05*h05*h12*h14*h24 + h22*h05*h05*h14*h14 + h11*h05*h05*h24*h24 - h11*h22*h44*h05*h05 + h00*h12*h12*h45*h45 - h00*h44*h55*h12*h12 + 2*h00*h55*h12*h14*h24 - 2*h00*h12*h14*h25*h45 - 2*h00*h12*h15*h24*h45 + 2*h00*h44*h12*h15*h25 + h00*h14*h14*h25*h25 - h00*h22*h55*h14*h14 - 2*h00*h14*h15*h24*h25 + 2*h00*h22*h14*h15*h45 + h00*h15*h15*h24*h24 - h00*h22*h44*h15*h15 - h00*h11*h55*h24*h24 + 2*h00*h11*h24*h25*h45 - h00*h11*h44*h25*h25 - h00*h11*h22*h45*h45 + h00*h11*h22*h44*h55)/divider;
	 invH[3][4]=-(h03*h04*h11*h25*h25 - h01*h04*h13*h25*h25 - h01*h03*h14*h25*h25 - h02*h03*h15*h15*h24 - h02*h04*h15*h15*h23 + h03*h04*h15*h15*h22 + h00*h13*h14*h25*h25 - h05*h05*h12*h13*h24 - h05*h05*h12*h14*h23 + h05*h05*h13*h14*h22 - h04*h05*h12*h12*h35 + h00*h15*h15*h23*h24 + h05*h05*h11*h23*h24 - h03*h05*h12*h12*h45 - h02*h02*h14*h15*h35 + h03*h04*h12*h12*h55 - h02*h02*h13*h15*h45 + h02*h02*h13*h14*h55 - h01*h01*h24*h25*h35 + h00*h12*h12*h35*h45 + h02*h02*h11*h35*h45 - h01*h01*h23*h25*h45 + h01*h01*h22*h35*h45 + h01*h01*h23*h24*h55 + h02*h03*h14*h15*h25 + h02*h04*h13*h15*h25 - 2*h02*h05*h13*h14*h25 + h02*h05*h13*h15*h24 + h02*h05*h14*h15*h23 - 2*h03*h04*h12*h15*h25 + h03*h05*h12*h14*h25 + h03*h05*h12*h15*h24 - h03*h05*h14*h15*h22 + h04*h05*h12*h13*h25 + h04*h05*h12*h15*h23 - h04*h05*h13*h15*h22 + h01*h03*h15*h24*h25 + h01*h04*h15*h23*h25 + h01*h05*h13*h24*h25 + h01*h05*h14*h23*h25 - 2*h01*h05*h15*h23*h24 + h02*h04*h12*h15*h35 + h02*h05*h12*h14*h35 - h03*h05*h11*h24*h25 - h04*h05*h11*h23*h25 - h00*h13*h15*h24*h25 - h00*h14*h15*h23*h25 + h01*h02*h14*h25*h35 + h01*h02*h15*h24*h35 + h01*h04*h12*h25*h35 - h01*h04*h15*h22*h35 + h01*h05*h12*h24*h35 - h01*h05*h14*h22*h35 + h02*h03*h12*h15*h45 - h02*h04*h11*h25*h35 - h02*h05*h11*h24*h35 + h02*h05*h12*h13*h45 + h04*h05*h11*h22*h35 - h00*h12*h14*h25*h35 - h00*h12*h15*h24*h35 + h00*h14*h15*h22*h35 + h01*h02*h13*h25*h45 + h01*h02*h15*h23*h45 + h01*h03*h12*h25*h45 - h01*h03*h15*h22*h45 + h01*h05*h12*h23*h45 - h01*h05*h13*h22*h45 - h02*h03*h11*h25*h45 - h02*h03*h12*h14*h55 - h02*h04*h12*h13*h55 - h02*h05*h11*h23*h45 + h03*h05*h11*h22*h45 + h00*h11*h24*h25*h35 - h00*h12*h13*h25*h45 - h00*h12*h15*h23*h45 + h00*h13*h15*h22*h45 - 2*h01*h02*h12*h35*h45 - h01*h02*h13*h24*h55 - h01*h02*h14*h23*h55 - h01*h03*h12*h24*h55 + h01*h03*h14*h22*h55 - h01*h04*h12*h23*h55 + h01*h04*h13*h22*h55 + h02*h03*h11*h24*h55 + h02*h04*h11*h23*h55 - h03*h04*h11*h22*h55 + h00*h11*h23*h25*h45 + h00*h12*h13*h24*h55 + h00*h12*h14*h23*h55 - h00*h13*h14*h22*h55 - h00*h11*h22*h35*h45 - h00*h11*h23*h24*h55)/divider;
	 invH[3][5]=-(h02*h02*h14*h14*h35 + h04*h04*h12*h12*h35 + h01*h01*h24*h24*h35 - h01*h03*h15*h24*h24 - h01*h05*h13*h24*h24 + h03*h05*h11*h24*h24 - h02*h03*h14*h14*h25 - h02*h05*h14*h14*h23 + h03*h05*h14*h14*h22 + h00*h13*h15*h24*h24 - h04*h04*h12*h13*h25 - h04*h04*h12*h15*h23 + h04*h04*h13*h15*h22 + h00*h14*h14*h23*h25 + h04*h04*h11*h23*h25 - h03*h04*h12*h12*h45 + h03*h05*h12*h12*h44 - h00*h11*h24*h24*h35 - h00*h14*h14*h22*h35 - h04*h04*h11*h22*h35 - h02*h02*h13*h14*h45 + h02*h02*h13*h15*h44 - h00*h12*h12*h35*h44 - h02*h02*h11*h35*h44 - h01*h01*h23*h24*h45 + h01*h01*h23*h25*h44 - h01*h01*h22*h35*h44 + h02*h03*h14*h15*h24 + h02*h04*h13*h14*h25 - 2*h02*h04*h13*h15*h24 + h02*h04*h14*h15*h23 + h02*h05*h13*h14*h24 + h03*h04*h12*h14*h25 + h03*h04*h12*h15*h24 - h03*h04*h14*h15*h22 - 2*h03*h05*h12*h14*h24 + h04*h05*h12*h13*h24 + h04*h05*h12*h14*h23 - h04*h05*h13*h14*h22 + h01*h03*h14*h24*h25 + h01*h04*h13*h24*h25 - 2*h01*h04*h14*h23*h25 + h01*h04*h15*h23*h24 + h01*h05*h14*h23*h24 - 2*h02*h04*h12*h14*h35 - h03*h04*h11*h24*h25 - h04*h05*h11*h23*h24 - h00*h13*h14*h24*h25 - h00*h14*h15*h23*h24 - 2*h01*h02*h14*h24*h35 - 2*h01*h04*h12*h24*h35 + 2*h01*h04*h14*h22*h35 + h02*h03*h12*h14*h45 - h02*h03*h12*h15*h44 + 2*h02*h04*h11*h24*h35 + h02*h04*h12*h13*h45 - h02*h05*h12*h13*h44 + 2*h00*h12*h14*h24*h35 + h01*h02*h13*h24*h45 - h01*h02*h13*h25*h44 + h01*h02*h14*h23*h45 - h01*h02*h15*h23*h44 + h01*h03*h12*h24*h45 - h01*h03*h12*h25*h44 - h01*h03*h14*h22*h45 + h01*h03*h15*h22*h44 + h01*h04*h12*h23*h45 - h01*h04*h13*h22*h45 - h01*h05*h12*h23*h44 + h01*h05*h13*h22*h44 - h02*h03*h11*h24*h45 + h02*h03*h11*h25*h44 - h02*h04*h11*h23*h45 + h02*h05*h11*h23*h44 + h03*h04*h11*h22*h45 - h03*h05*h11*h22*h44 - h00*h12*h13*h24*h45 + h00*h12*h13*h25*h44 - h00*h12*h14*h23*h45 + h00*h12*h15*h23*h44 + h00*h13*h14*h22*h45 - h00*h13*h15*h22*h44 + 2*h01*h02*h12*h35*h44 + h00*h11*h23*h24*h45 - h00*h11*h23*h25*h44 + h00*h11*h22*h35*h44)/divider;
	 invH[4][4]=(h55*h01*h01*h23*h23 - 2*h01*h01*h23*h25*h35 + h33*h01*h01*h25*h25 + h22*h01*h01*h35*h35 - h22*h33*h55*h01*h01 - 2*h01*h02*h12*h35*h35 + 2*h33*h55*h01*h02*h12 - 2*h55*h01*h02*h13*h23 + 2*h01*h02*h13*h25*h35 + 2*h01*h02*h15*h23*h35 - 2*h33*h01*h02*h15*h25 - 2*h55*h01*h03*h12*h23 + 2*h01*h03*h12*h25*h35 - 2*h01*h03*h13*h25*h25 + 2*h22*h55*h01*h03*h13 + 2*h01*h03*h15*h23*h25 - 2*h22*h01*h03*h15*h35 + 2*h01*h05*h12*h23*h35 - 2*h33*h01*h05*h12*h25 + 2*h01*h05*h13*h23*h25 - 2*h22*h01*h05*h13*h35 - 2*h01*h05*h15*h23*h23 + 2*h22*h33*h01*h05*h15 + h55*h02*h02*h13*h13 - 2*h02*h02*h13*h15*h35 + h33*h02*h02*h15*h15 + h11*h02*h02*h35*h35 - h11*h33*h55*h02*h02 - 2*h55*h02*h03*h12*h13 + 2*h02*h03*h12*h15*h35 + 2*h02*h03*h13*h15*h25 - 2*h02*h03*h15*h15*h23 + 2*h11*h55*h02*h03*h23 - 2*h11*h02*h03*h25*h35 + 2*h02*h05*h12*h13*h35 - 2*h33*h02*h05*h12*h15 - 2*h02*h05*h13*h13*h25 + 2*h02*h05*h13*h15*h23 - 2*h11*h02*h05*h23*h35 + 2*h11*h33*h02*h05*h25 + h55*h03*h03*h12*h12 - 2*h03*h03*h12*h15*h25 + h22*h03*h03*h15*h15 + h11*h03*h03*h25*h25 - h11*h22*h55*h03*h03 - 2*h03*h05*h12*h12*h35 + 2*h03*h05*h12*h13*h25 + 2*h03*h05*h12*h15*h23 - 2*h22*h03*h05*h13*h15 - 2*h11*h03*h05*h23*h25 + 2*h11*h22*h03*h05*h35 + h33*h05*h05*h12*h12 - 2*h05*h05*h12*h13*h23 + h22*h05*h05*h13*h13 + h11*h05*h05*h23*h23 - h11*h22*h33*h05*h05 + h00*h12*h12*h35*h35 - h00*h33*h55*h12*h12 + 2*h00*h55*h12*h13*h23 - 2*h00*h12*h13*h25*h35 - 2*h00*h12*h15*h23*h35 + 2*h00*h33*h12*h15*h25 + h00*h13*h13*h25*h25 - h00*h22*h55*h13*h13 - 2*h00*h13*h15*h23*h25 + 2*h00*h22*h13*h15*h35 + h00*h15*h15*h23*h23 - h00*h22*h33*h15*h15 - h00*h11*h55*h23*h23 + 2*h00*h11*h23*h25*h35 - h00*h11*h33*h25*h25 - h00*h11*h22*h35*h35 + h00*h11*h22*h33*h55)/divider;
	 invH[4][5]=-(h02*h02*h13*h13*h45 + h03*h03*h12*h12*h45 + h01*h01*h23*h23*h45 - h01*h04*h15*h23*h23 - h01*h05*h14*h23*h23 + h04*h05*h11*h23*h23 - h02*h04*h13*h13*h25 - h02*h05*h13*h13*h24 + h04*h05*h13*h13*h22 + h00*h14*h15*h23*h23 - h03*h04*h12*h12*h35 + h04*h05*h12*h12*h33 - h03*h03*h12*h14*h25 - h03*h03*h12*h15*h24 + h03*h03*h14*h15*h22 + h00*h13*h13*h24*h25 + h03*h03*h11*h24*h25 - h02*h02*h13*h14*h35 + h02*h02*h14*h15*h33 - h00*h11*h23*h23*h45 - h00*h13*h13*h22*h45 - h03*h03*h11*h22*h45 - h01*h01*h23*h24*h35 + h01*h01*h24*h25*h33 - h00*h12*h12*h33*h45 - h02*h02*h11*h33*h45 - h01*h01*h22*h33*h45 + h02*h03*h13*h14*h25 + h02*h03*h13*h15*h24 - 2*h02*h03*h14*h15*h23 + h02*h04*h13*h15*h23 + h02*h05*h13*h14*h23 + h03*h04*h12*h13*h25 + h03*h04*h12*h15*h23 - h03*h04*h13*h15*h22 + h03*h05*h12*h13*h24 + h03*h05*h12*h14*h23 - h03*h05*h13*h14*h22 - 2*h04*h05*h12*h13*h23 - 2*h01*h03*h13*h24*h25 + h01*h03*h14*h23*h25 + h01*h03*h15*h23*h24 + h01*h04*h13*h23*h25 + h01*h05*h13*h23*h24 + h02*h03*h12*h14*h35 + h02*h04*h12*h13*h35 - h02*h04*h12*h15*h33 - h02*h05*h12*h14*h33 - h03*h04*h11*h23*h25 - h03*h05*h11*h23*h24 - h00*h13*h14*h23*h25 - h00*h13*h15*h23*h24 + h01*h02*h13*h24*h35 + h01*h02*h14*h23*h35 - h01*h02*h14*h25*h33 - h01*h02*h15*h24*h33 + h01*h03*h12*h24*h35 - h01*h03*h14*h22*h35 + h01*h04*h12*h23*h35 - h01*h04*h12*h25*h33 - h01*h04*h13*h22*h35 + h01*h04*h15*h22*h33 - h01*h05*h12*h24*h33 + h01*h05*h14*h22*h33 - h02*h03*h11*h24*h35 - 2*h02*h03*h12*h13*h45 - h02*h04*h11*h23*h35 + h02*h04*h11*h25*h33 + h02*h05*h11*h24*h33 + h03*h04*h11*h22*h35 - h04*h05*h11*h22*h33 - h00*h12*h13*h24*h35 - h00*h12*h14*h23*h35 + h00*h12*h14*h25*h33 + h00*h12*h15*h24*h33 + h00*h13*h14*h22*h35 - h00*h14*h15*h22*h33 - 2*h01*h02*h13*h23*h45 - 2*h01*h03*h12*h23*h45 + 2*h01*h03*h13*h22*h45 + 2*h02*h03*h11*h23*h45 + h00*h11*h23*h24*h35 - h00*h11*h24*h25*h33 + 2*h00*h12*h13*h23*h45 + 2*h01*h02*h12*h33*h45 + h00*h11*h22*h33*h45)/divider;
	 invH[5][5]=(h44*h01*h01*h23*h23 + h33*h01*h01*h24*h24 - h22*h33*h44*h01*h01 + 2*h33*h44*h01*h02*h12 - 2*h44*h01*h02*h13*h23 - 2*h33*h01*h02*h14*h24 - 2*h44*h01*h03*h12*h23 - 2*h01*h03*h13*h24*h24 + 2*h22*h44*h01*h03*h13 + 2*h01*h03*h14*h23*h24 - 2*h33*h01*h04*h12*h24 + 2*h01*h04*h13*h23*h24 - 2*h01*h04*h14*h23*h23 + 2*h22*h33*h01*h04*h14 + h44*h02*h02*h13*h13 + h33*h02*h02*h14*h14 - h11*h33*h44*h02*h02 - 2*h44*h02*h03*h12*h13 + 2*h02*h03*h13*h14*h24 - 2*h02*h03*h14*h14*h23 + 2*h11*h44*h02*h03*h23 - 2*h33*h02*h04*h12*h14 - 2*h02*h04*h13*h13*h24 + 2*h02*h04*h13*h14*h23 + 2*h11*h33*h02*h04*h24 + h44*h03*h03*h12*h12 - 2*h03*h03*h12*h14*h24 + h22*h03*h03*h14*h14 + h11*h03*h03*h24*h24 - h11*h22*h44*h03*h03 + 2*h03*h04*h12*h13*h24 + 2*h03*h04*h12*h14*h23 - 2*h22*h03*h04*h13*h14 - 2*h11*h03*h04*h23*h24 + h33*h04*h04*h12*h12 - 2*h04*h04*h12*h13*h23 + h22*h04*h04*h13*h13 + h11*h04*h04*h23*h23 - h11*h22*h33*h04*h04 - h00*h33*h44*h12*h12 + 2*h00*h44*h12*h13*h23 + 2*h00*h33*h12*h14*h24 + h00*h13*h13*h24*h24 - h00*h22*h44*h13*h13 - 2*h00*h13*h14*h23*h24 + h00*h14*h14*h23*h23 - h00*h22*h33*h14*h14 - h00*h11*h44*h23*h23 - h00*h11*h33*h24*h24 + h00*h11*h22*h33*h44)/divider;
	 for(int m=0;m<6;m++)
		for(int n=0;n<m;n++)
			invH[m][n]=invH[n][m];//利用对称性 

	 a=a-beta*(invH[0][0]*da+invH[0][1]*db+invH[0][2]*dTh+invH[0][3]*doX+invH[0][4]*doY+invH[0][5]*doZ);
	 b=b-beta*(invH[1][0]*da+invH[1][1]*db+invH[1][2]*dTh+invH[1][3]*doX+invH[1][4]*doY+invH[1][5]*doZ);
	 theta=theta-beta*(invH[2][0]*da+invH[2][1]*db+invH[2][2]*dTh+invH[2][3]*doX+invH[2][4]*doY+invH[2][5]*doZ);
	 oX=oX-beta*(invH[3][0]*da+invH[3][1]*db+invH[3][2]*dTh+invH[3][3]*doX+invH[3][4]*doY+invH[3][5]*doZ);
	 oY=oY-beta*(invH[4][0]*da+invH[4][1]*db+invH[4][2]*dTh+invH[4][3]*doX+invH[4][4]*doY+invH[4][5]*doZ);
	 oZ=oZ-beta*(invH[5][0]*da+invH[5][1]*db+invH[5][2]*dTh+invH[5][3]*doX+invH[5][4]*doY+invH[5][5]*doZ);
	 //更新R和t
	 cosA=cos(a),sinA=sin(a),cosB=cos(b),sinB=sin(b),cosTh=cos(theta),sinTh=sin(theta);
	 R[0][0]=cosTh*cosB;
	 R[0][1]=cosA*sinTh + cosTh*sinA*sinB;
	 R[0][2]=sinTh*sinA - cosTh*cosA*sinB;
	 R[1][0]=-cosB*sinTh;
	 R[1][1]=cosTh*cosA - sinTh*sinA*sinB;
	 R[1][2]=cosTh*sinA + cosA*sinTh*sinB;
	 R[2][0]=sinB;
	 R[2][1]=-cosB*sinA;
	 R[2][2]=cosA*cosB;
    t[0]=oX;
	 t[1]=oY;
	 t[2]=oZ;
  }

  //数据还原、参数尺度也还原 
  for(int j = 0; j < number_of_correspondences; j++) {
		 float * ptrPws = pws + 3 * j;
		 ptrPws[0]=ptrPws[0];//*889;//限x米 
		 ptrPws[1]=ptrPws[1];//*889;
		 ptrPws[2]=ptrPws[2];//*889;
  }
  t[0]=t[0];//*889;
  t[1]=t[1];//*889;
  t[2]=t[2];//*889;
  uc0=uc0*28;vc0=vc0*28;fx=fx*28;

  //record as prior knowledge 
  _tr6[0]=a,_tr6[1]=b,_tr6[2]=theta,_tr6[3]=oX,_tr6[4]=oY,_tr6[5]=oZ;
  
  return -1;
}


void PnPsolver::gauss_newton(const CvMat * L_6x10, const CvMat * Rho,
			float betas[4])
{
  const int iterations_number = 6;//5;

  float a[6*4], b[6], x[4];
  CvMat A = cvMat(6, 4, CV_32F, a);
  CvMat B = cvMat(6, 1, CV_32F, b);
  CvMat X = cvMat(4, 1, CV_32F, x);

  for(int k = 0; k < iterations_number; k++) {
    compute_A_and_b_gauss_newton(L_6x10->data.fl, Rho->data.fl,
				 betas, &A, &B);
    qr_solve(&A, &B, &X);

    for(int i = 0; i < 4; i++)
      betas[i] += x[i];
  }
}


void PnPsolver::qr_solve(CvMat * A, CvMat * b, CvMat * X)
{
  static int max_nr = 0;
  static float * A1, * A2;

  const int nr = A->rows;
  const int nc = A->cols;

  if (max_nr != 0 && max_nr < nr) {
    delete [] A1;
    delete [] A2;
  }
  if (max_nr < nr) {
    max_nr = nr;
    A1 = new float[nr];
    A2 = new float[nr];
  }

  float * pA = A->data.fl, * ppAkk = pA;
  for(int k = 0; k < nc; k++) {
    float * ppAik = ppAkk, eta = fabs(*ppAik);
    for(int i = k + 1; i < nr; i++) {
      float elt = fabs(*ppAik);
      if (eta < elt) eta = elt;
      ppAik += nc;
    }

    if (eta == 0) {
      A1[k] = A2[k] = 0.0;
      cerr << "God damnit, A is singular, this shouldn't happen." << endl;
      return;
    } else {
      float * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
      for(int i = k; i < nr; i++) {
	*ppAik *= inv_eta;
	sum += *ppAik * *ppAik;
	ppAik += nc;
      }
      float sigma = sqrt(sum);
      if (*ppAkk < 0)
	sigma = -sigma;
      *ppAkk += sigma;
      A1[k] = sigma * *ppAkk;
      A2[k] = -eta * sigma;
      for(int j = k + 1; j < nc; j++) {
	float * ppAik = ppAkk, sum = 0;
	for(int i = k; i < nr; i++) {
	  sum += *ppAik * ppAik[j - k];
	  ppAik += nc;
	}
	float tau = sum / A1[k];//【这里除了0】
	ppAik = ppAkk;
	for(int i = k; i < nr; i++) {
	  ppAik[j - k] -= tau * *ppAik;
	  ppAik += nc;
	}
      }
    }
    ppAkk += nc + 1;
  }

  // b <- Qt b
  float * ppAjj = pA, * pb = b->data.fl;
  for(int j = 0; j < nc; j++) {
    float * ppAij = ppAjj, tau = 0;
    for(int i = j; i < nr; i++)	{
      tau += *ppAij * pb[i];
      ppAij += nc;
    }
    tau /= A1[j];
    ppAij = ppAjj;
    for(int i = j; i < nr; i++) {
      pb[i] -= tau * *ppAij;
      ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  float * pX = X->data.fl;
  pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
  for(int i = nc - 2; i >= 0; i--) {
    float * ppAij = pA + i * nc + (i + 1), sum = 0;

    for(int j = i + 1; j < nc; j++) {
      sum += *ppAij * pX[j];
      ppAij++;
    }
    pX[i] = (pb[i] - sum) / A2[i];
  }
}


void PnPsolver::relative_error(float & rot_err, float & transl_err,
			  const float Rtrue[3][3], const float ttrue[3],
			  const float Rest[3][3],  const float test[3])
{
  float qtrue[4], qest[4];

  mat_to_quat(Rtrue, qtrue);
  mat_to_quat(Rest, qest);

  float rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
			 (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
			 (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
			 (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  float rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
			 (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
			 (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
			 (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  rot_err = min(rot_err1, rot_err2);

  transl_err =
    sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
	 (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
	 (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
    sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}


void PnPsolver::mat_to_quat(const float R[3][3], float q[4])
{
  float tr = R[0][0] + R[1][1] + R[2][2];
  float n4;

  if (tr > 0.0f) {
    q[0] = R[1][2] - R[2][1];
    q[1] = R[2][0] - R[0][2];
    q[2] = R[0][1] - R[1][0];
    q[3] = tr + 1.0f;
    n4 = q[3];
  } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
    q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
    q[1] = R[1][0] + R[0][1];
    q[2] = R[2][0] + R[0][2];
    q[3] = R[1][2] - R[2][1];
    n4 = q[0];
  } else if (R[1][1] > R[2][2]) {
    q[0] = R[1][0] + R[0][1];
    q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
    q[2] = R[2][1] + R[1][2];
    q[3] = R[2][0] - R[0][2];
    n4 = q[1];
  } else {
    q[0] = R[2][0] + R[0][2];
    q[1] = R[2][1] + R[1][2];
    q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
    q[3] = R[0][1] - R[1][0];
    n4 = q[2];
  }
  float scale = 0.5f / float(sqrt(n4));

  q[0] *= scale;
  q[1] *= scale;
  q[2] *= scale;
  q[3] *= scale;
}


//self deduce 
/*u=fx*(x/z)+u0
d=fx*b/z*/
//其中fx是归一化了的焦距x 
/*float Q[][4]={ 1, 0, 0, -cx, //脑残的矩阵形式 
0, 1 ,0, -cy,
0, 0, 0, fx,
0, 0, -1/b, x};
*/
//当俩相机对眼时，x是为一定值 
float fx=500/2;
float B=0.1197;//m 
float uc0=322/2;
float vc0=235.5/2;

int w7=3;
float DUAN=0.5;
int _26=3;
//反投影 
PP3D cvReprojectTo3D(PP p7, int _disparity, 
                              float Q[][4])
{

    PP3D p3D;
//直接算了 
if(_disparity>=0)
{
	p3D.z=fx*B/_disparity;//z
	p3D.x=(p7.c-uc0)*B/_disparity;//x//以左图为准，图中心相对于大地坐标系中心的偏移不影响 
	p3D.y=(p7.r-vc0)*B/_disparity;//y
}
else
	p3D.z=-1;
return p3D;
//
}
//} //namespace ORB_SLAM