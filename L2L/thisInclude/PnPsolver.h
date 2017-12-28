/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Ra¨²l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include <cv.h>
#include <cxcore.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
//using namespace cv;
#include "MapPoint.h"

using namespace std;

//namespace ORB_SLAM2
//{

class PnPsolver {
 public:
  PnPsolver( Match *m0, int matchesCountt, featureP *Pseries0, featureP *Pseries1, int *d0);
  PnPsolver( cv::Mat im0L, cv::Mat im1L, Match *m0, int matchesCountt, featureP *Pseries0, featureP *Pseries1, featureP *Pseries0X, featureP *Pseries1X, featureP *Pseries0Y, featureP *Pseries1Y, int *d0, int *d0X, int *d0Y);
  PnPsolver( vector<cv::DMatch> m0, vector<cv::KeyPoint> keypoints0, vector<cv::KeyPoint> keypoints1, int *d0);

  ~PnPsolver();

  void SetRansacParameters(float probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991, cv::Mat TcwLast=cv::Mat());

  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  cv::Mat iterate(int nIterations, vector<bool> &vbInliers, int &nInliers);

 private:

  void CheckInliers();
  void checkandSort(int subsetCountt);
  bool Refine();
  bool RefineAnyway();

  // Functions from the original EPnP code
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(float X0, float Y0, float Z0, float u1, float v1);

  float compute_pose0(float R[3][3], float t[3], float *_tr000, float R_000[][3], float *t_000, float *_tr111, float R_111[][3], float *t_111);
  float compute_pose(float R[3][3], float t[3]);
  float compute_poseORI(float R[3][3], float t[3]);

  void relative_error(float & rot_err, float & transl_err,
              const float Rtrue[3][3], const float ttrue[3],
              const float Rest[3][3],  const float test[3]);

  void print_pose(const float R[3][3], const float t[3]);
  float errCal7( float R[3][3], float t[3]);

  void choose_control_points(void);
  void compute_barycentric_coordinates(void);
  void fill_M(CvMat * M, const int row, const float * alphas, const float u, const float v);
  void compute_ccs(const float * betas, const float * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approX_1(const CvMat * L_6x10, const CvMat * Rho, float * betas);
  void find_betas_approX_2(const CvMat * L_6x10, const CvMat * Rho, float * betas);
  void find_betas_approX_3(const CvMat * L_6x10, const CvMat * Rho, float * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  void compute_rho(float * rho);
  void compute_L_6x10(const float * ut, float * l_6x10);

  int gauss_newtonZIDINGYI0( float R[][3], float *t, float *_tr000, float R_000[][3], float *t_000, float *_tr111, float R_111[][3], float *t_111);
  int gauss_newtonZIDINGYI( float R[][3], float *t, float beta);
  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, float current_betas[4]);
  void compute_A_and_b_gauss_newton(const float * l_6x10, const float * rho,
				    float cb[4], CvMat * A, CvMat * b);

  void estimate_R_and_t(float R[3][3], float t[3]);
  float compute_R_and_t(const float * ut, const float * betas,
			 float R[3][3], float t[3]);

  void copy_R_and_t( float R_src[3][3], float t_src[3],
			float R_dst[3][3], float t_dst[3]);

  void mat_to_quat(const float R[3][3], float q[4]);

  float uc, vc, fu, fv;

  float * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  float cws[4][3], ccs[4][3];
  float cws_determinant;

  //vector<MapPoint*> mvpMapPointMatches;
  int matchesCountt;

  // 2D Points
  vector<cv::Point2f> p12D;
  vector<float> mvSigma2;

  // 3D Points
  vector<cv::Point3f> p03D;

  // Index in Frame
  vector<size_t> mvKeyPointIndices;

  // Current Estimation
  float mRi[3][3];
  float mti[3];
  cv::Mat mTcwi;
  vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestTcw;

  // Refined
  cv::Mat mRefinedTcw;
  vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // Number of Correspondences
  int N;

  // Indices for random selection [0 .. N-1]
  vector<size_t> mvAllIndices;

  // RANSAC probability
  float mRansacProb;

  // RANSAC min inliers
  int mRansacMinInliers;
  int mRansacMinInliers2;

  // RANSAC max iterations
  int mRansacMaxIts;

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;

  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet;

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;

};


extern float fx;
extern float B;
extern float uc0;
extern float vc0;
PP3D cvReprojectTo3D(PP p7, int _disparity, 
                              float Q[][4]);
//} //namespace ORB_SLAM

extern FILE *fpCal;
#endif //PNPSOLVER_H
