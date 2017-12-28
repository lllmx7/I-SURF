
#include "MapPoint.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace cv;

int featureExtractingSURF( vector<KeyPoint> *keypoints0, Mat *descriptors0, Mat im0);
int featureExtractingORB( vector<KeyPoint> *keypoints0, Mat *descriptors0, Mat im0);
extern int dmax;
extern int _19;
extern int _7;
extern double cornerThLUT[];
extern double uniquenessRatio;
extern double thABS;
int PPcluster(featureP *_Pseries0, featureP *Pseries0, int CounttPs0, int WindowSize, Mat im0);//一小块只取一个点 
int getSubset8( featureP *Pselected, int *IndexofPselected, featureP *Pseries, int CounttPs, int PselectedCountt);
int findCorrespondence( Match *m0, featureP *Pseries0, int CounttPs0, featureP *Pseries1, int CounttPs1, Mat img0, Mat img0_int, Mat img1, Mat img1_int);//p0去p1中找对应 
int matchesDrawing( Match *m0, int matchesCountt, featureP *Pseries0, featureP *Pseries1, Mat im0, Mat im1);
int pDrawing( featureP *Pseries0, int pCountt0, Mat im0);

extern int numberofCao;
Mat histIntegral(Mat img0);//return Mat_int32
int sumYikuai( int *hist, Mat img_int, int row, int col, int rows, int cols); 
Mat Integral0(Mat img);
int sumYikuai0( Mat img_int, int row, int col, int rows, int cols);

#ifndef ABS
#  define ABS(a)  ( (a)>=0 ? (a) : -(a))
#endif