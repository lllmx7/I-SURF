
#include "MapPoint.h"

#include <cv.h>
#include <cxcore.h>

extern cv::Mat im0R;
cv::Mat L2L( cv::Mat im0, cv::Mat im0R, cv::Mat im1, int if1stTime, cv::Mat Tcw_es);//返回CV_32FC1数据类型 
cv::Mat L2L000( cv::Mat im0L, cv::Mat im0R, cv::Mat im1L, cv::Mat Tcw_es);
cv::Mat L2L111( cv::Mat im0L, cv::Mat im0R, cv::Mat im1L, cv::Mat Tcw_es);
int ocv_zhaod( int *d1, cv::Mat im1L, cv::Mat im1R, featureP *Pseries1, int CounttPs1, int _19, int dMAX);
int ocv_zhaod222( int *d0, cv::Mat im0L, cv::Mat im0R, cv::Mat dMat, featureP *Pseries0, int CounttPs0);
cv::Mat ocv_bm(cv::Mat left0, cv::Mat right0);
cv::Mat ocv_sgbm(cv::Mat left, cv::Mat right);
