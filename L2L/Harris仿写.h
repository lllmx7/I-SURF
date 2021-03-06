// 仿写Harris.cpp : 定义控制台应用程序的入口点。
//

#include <cv.h>
#include <cxcore.h>

extern int featureExtractingHarrisDx( featureP *Pseries0, cv::Mat *descriptors0, cv::Mat im0, cv::Mat im0_int);
extern int featureExtractingHarrisDy( featureP *Pseries0, cv::Mat *descriptors0, cv::Mat im0, cv::Mat im0_int);
extern float *dx1( cv::Mat im0, cv::Mat im0_int, int w, int pitch);
extern int multiDx( float **D, float **chaD, int *scale, int octaves, int intervals, int _19, cv::Mat im0, cv::Mat im0_int);//int nOctaves=4, int nOctaveLayers=2
extern float *dy1( cv::Mat im0, cv::Mat im0_int, int w, int pitch);
extern int multiDy( float **D, float **chaD, int *scale, int scaleN, int _19, cv::Mat im0, cv::Mat im0_int);
extern double thdx;
extern double thdy;
extern int DetectX( featureP *Pseries0, float **multiscaleDx, float **multiscaleDy, float **chaDx, int *scale, int scaleN, int flag, int th, cv::Mat im0);
extern int isExtremumX(int r, int c, float *b, float *b_cha, float *s_cha, float *t_cha,  float th, int scalei, int scalei_1, int scaleijia1, cv::Mat im0);
extern int DetectY( featureP *Pseries0, float **multiscaleDx, float **multiscaleDy, float **chaDy, int *scale, int scaleN, int flag, int th, cv::Mat im0);
extern int isExtremumY(int r, int c, float *b, float *b_cha, float *s_cha, float *t_cha,  float th, int scalei, int scalei_1, int scaleijia1, cv::Mat im0);
extern cv::Mat Integral(cv::Mat im0);
extern int sumYikuai(cv::Mat im0_int, int row, int col, int rows, int cols); 

#ifndef ABS
#  define ABS(a)  ( (a)>=0 ? (a) : -(a))
#endif