// 仿写Harris.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "MapPoint.h"
#include "Harris仿写.h"
#include "sad.h"

#include <cv.h>
#include <cxcore.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
using namespace cv;



//float k = 0.04;
//float Threshold = 50000;
//float sigma = 1;
//float halfwid = sigma * 3;

//-------------------------------------------------------
int scale[17];
int scaleN;
int bSize=26;
float **multiscaleDxGlobal;
float **multiscaleDyGlobal;
float **chaDyGlobal;
int featureExtractingHarrisDx( featureP *Pseries0, Mat *descriptors0, Mat im0, Mat im0_int)
{
	SYSTEMTIME st;
GetLocalTime(&st);
int time0=st.wSecond*1000+st.wMilliseconds;
	/// Build the response&findP
	// Build the response
	float **multiscaleDx=new float*[17];
	float **chaDx=new float*[17];
	//int scale[17];
	scaleN=multiDx( multiscaleDx, chaDx, scale, 4, 3, 2*bSize/3, im0, im0_int);//19的2/3 
	float **multiscaleDy=new float*[17];
	float **chaDy=new float*[17];
	multiDy( multiscaleDy, chaDy, scale, scaleN, 2*bSize/3, im0, im0_int);//19的2/3 
GetLocalTime(&st);
int time1=st.wSecond*1000+st.wMilliseconds;
char timeji000[87];
sprintf(timeji000,"\n_time consuming_%d",time1-time0);
	//detectP
	int pCountt0=DetectX( Pseries0, multiscaleDx, multiscaleDy, chaDx, scale, scaleN, pianDX, thdx, im0);
GetLocalTime(&st);
time1=st.wSecond*1000+st.wMilliseconds;
char timeji111[87];
sprintf(timeji111,"\n_time consuming_%d",time1-time0);
//pDrawing( Pseries0, pCountt0, im0);
	/*int idx=pCountt0;
	for(int i=0;i<pCountt0aaa;i++)
	{
		Pseries0[idx]=Pseries0aaa[i];
		idx++;
	}
	pCountt0=pCountt0+pCountt0aaa;
	*/
	 // computing descriptors
    SurfDescriptorExtractor extractor;
	 vector<KeyPoint> keypoints0;
	 for(int i=0;i<pCountt0;i++)
		 keypoints0.push_back(Pseries0[i].kpt);
    extractor.compute(im0, keypoints0, *descriptors0);
GetLocalTime(&st);
time1=st.wSecond*1000+st.wMilliseconds;
char timeji222[87];
sprintf(timeji222,"\n_time consuming_%d",time1-time0);
size_t len = strlen(timeji000) + 1;
size_t converted = 0;
wchar_t *WStr;
WStr=(wchar_t*)malloc(len*sizeof(wchar_t));
mbstowcs_s(&converted, WStr, len, timeji000, _TRUNCATE);//转宽字符 
MessageBox(NULL,WStr,_T("ddd"),MB_OKCANCEL);
len = strlen(timeji111) + 1;
converted = 0;
WStr=(wchar_t*)malloc(len*sizeof(wchar_t));
mbstowcs_s(&converted, WStr, len, timeji111, _TRUNCATE);//转宽字符 
MessageBox(NULL,WStr,_T("ddd"),MB_OKCANCEL);
len = strlen(timeji222) + 1;
converted = 0;
WStr=(wchar_t*)malloc(len*sizeof(wchar_t));
mbstowcs_s(&converted, WStr, len, timeji222, _TRUNCATE);//转宽字符 
MessageBox(NULL,WStr,_T("ddd"),MB_OKCANCEL);
	
	//求R/T矩阵 
	//findHomography(
	//for(int i=0;i<scaleN;i++)
		//delete [] multiscaleDx[i];
	for(int i=1;i<scaleN;i++)
		delete [] chaDx[i];
	//delete [] multiscaleDx;
	delete [] chaDx;
	multiscaleDxGlobal=multiscaleDx;
	multiscaleDyGlobal=multiscaleDy;
	chaDyGlobal=chaDy;
	return pCountt0;
}
int featureExtractingHarrisDy( featureP *Pseries0, Mat *descriptors0, Mat im0, Mat im0_int)
{
	
	/// Build the response&findP
	//detectP
	//featureP Pseries0aaa[8887];
	int pCountt0=DetectY( Pseries0, multiscaleDxGlobal, multiscaleDyGlobal, chaDyGlobal, scale, scaleN, pianDX, thdy, im0);
//pDrawing( Pseries0aaa, pCountt0aaa, im0);
//pDrawing( Pseries0, pCountt0, im0);
	 // computing descriptors
    SurfDescriptorExtractor extractor;
	 vector<KeyPoint> keypoints0;
	 for(int i=0;i<pCountt0;i++)
		 keypoints0.push_back(Pseries0[i].kpt);
    extractor.compute(im0, keypoints0, *descriptors0);
	
	//求R/T矩阵 
	//findHomography(
	for(int i=0;i<scaleN;i++){
		delete [] multiscaleDxGlobal[i];
		delete [] multiscaleDyGlobal[i];
	}
	for(int i=1;i<scaleN;i++)
		delete [] chaDyGlobal[i];
	delete [] multiscaleDxGlobal;
	delete [] multiscaleDyGlobal;
	delete [] chaDyGlobal;
	return pCountt0;
}

//-------------------------------------------------------

float *dx1( Mat im0, Mat im0_int, int w, int pitch)
{
  float *d000 = new float[im0.rows*im0.cols];
  double inverse_windowArea = 1.f/(w*w);// normalisation factor
  int halfWindow=w/2;
  int WindowSize_3=w/3;
  int *data = (int*)im0_int.ptr(0);
  int step = im0_int.step/sizeof(int);
  int A, B, C, D;

  int rs=halfWindow;
  int rp=im0.rows-halfWindow;
  int cs=halfWindow;
  int cp=im0.cols-halfWindow;
  for(int r = rs; r < rp; ++r) 
  {
	 float *ptrF_d000=d000+r*im0.cols;
	 int *pr0=data + (r-halfWindow) * step;
	 int *pr1=data + (r+halfWindow) * step;
    for(int c = cs; c < cp; ++c) 
    {
      
      // Compute response components
      //double Dx = sumYikuai( im0_int, r-halfWindow, c, w, halfWindow)
          //- sumYikuai(im0_int, r-halfWindow, c-halfWindow, w, halfWindow);
		double Dx;
		A = pr0[c];
		B = pr0[c+halfWindow];
		C = pr1[c];
		D = pr1[c+halfWindow];
		Dx=A - B - C + D;
		A = pr0[c-halfWindow];
		B = pr0[c];
		C = pr1[c-halfWindow];
		D = pr1[c];
		Dx=Dx-(A - B - C + D);
      
      // Normalise the filter responses with respect to their size//太慢了
      //////////////////////////////////////////////////////////////Dx = inverse_windowArea*Dx;
           
      // Get the determinant of hessian response & laplacian sign
      ptrF_d000[c] = Dx;//(Dxx * Dyy - 0.81f * Dxy * Dxy);
    }
  }
  return d000;
}
int multiDx( float **D, float **chaD, int *scale, int octaves, int intervals, int _19, Mat im0, Mat im0_int)//int nOctaves=4, int nOctaveLayers=2
{
	scale[0]=_19;
	double scaleFactor=_19;
	double mulby=pow(2,1.0/intervals);
	int i,j,k;
	for( i=0;i<octaves;i++)
		for( j=0;j<intervals;j++)
		{
			scale[i*intervals+j]=scaleFactor;
			scaleFactor=scaleFactor*mulby;
		}
	int scaleN=octaves*intervals;

	for( i=0;i<scaleN;i++)
		D[i]=dx1( im0, im0_int, scale[i], -1);
	int imLength=im0.rows*im0.cols;
	for( i=1;i<scaleN;i++)
	{
		chaD[i]=new float[imLength];
		float *p_chaD=chaD[i];
		float *pD=D[i];
		float *p_PD=D[i-1];
		for(int c=0;c<imLength;c++)
			p_chaD[c]=pD[c]-p_PD[c];
	}
	return scaleN;
}

//-------------------------------------------------------

float *dy1( Mat im0, Mat im0_int, int w, int pitch)
{
  float *d000 = new float[im0.rows*im0.cols];
  float inverse_windowArea = 1.f/(w*w);// normalisation factor
  int halfWindow=w/2;
  int WindowSize_3=w/3;
  int *data = (int*)im0_int.ptr(0);
  int step = im0_int.step/sizeof(int);
  int A, B, C, D;

  int rs=halfWindow;
  int rp=im0.rows-halfWindow;
  int cs=halfWindow;
  int cp=im0.cols-halfWindow;
  for(int r = rs; r < rp; ++r) 
  {
	 float *ptrF_d000=d000+r*im0.cols;
	 int *pr0=data + (r-halfWindow) * step;
	 int *pr1=data + r * step;
	 int *pr2=data + (r+halfWindow) * step;
    for(int c = cs; c < cp; ++c) 
    {
      
      // Compute response components
      //float Dy = sumYikuai( im0_int, r, c-halfWindow, halfWindow, w)
          //- sumYikuai(im0_int, r-halfWindow, c-halfWindow, halfWindow, w);
		float Dy;
		A = pr1[c-halfWindow];
		B = pr1[c+halfWindow];
		C = pr2[c-halfWindow];
		D = pr2[c+halfWindow];
		Dy=A - B - C + D;
		A = pr0[c-halfWindow];
		B = pr0[c+halfWindow];
		C = pr1[c-halfWindow];
		D = pr1[c+halfWindow];
		Dy=Dy-(A - B - C + D);
      
      // Normalise the filter responses with respect to their size//太慢了
      //////////////////////////////////////////////////////////////Dy = inverse_windowArea*Dy;
           
      // Get the determinant of hessian response & laplacian sign
      ptrF_d000[c] = Dy;//(Dxx * Dyy - 0.81f * Dxy * Dxy);
    }
  }
  return d000;
}
int multiDy( float **D, float **chaD, int *scale, int scaleN, int _19, Mat im0, Mat im0_int)//int nOctaves=4, int nOctaveLayers=2
{

	int i,j,k;
	for( i=0;i<scaleN;i++)
		D[i]=dy1( im0, im0_int, scale[i], -1);
	int imLength=im0.rows*im0.cols;
	for( i=1;i<scaleN;i++)
	{
		chaD[i]=new float[imLength];
		float *p_chaD=chaD[i];
		float *pD=D[i];
		float *p_PD=D[i-1];
		for(int c=0;c<imLength;c++)
			p_chaD[c]=pD[c]-p_PD[c];
	}
	return -1;
}

//-------------------------------------------------------
double thdx=13;
double thdy=13;
int DetectX( featureP *Pseries0, float **multiscaleDx, float **multiscaleDy, float **chaDx, int *scale, int scaleN, int flag, int th, Mat im0)
{
	int pCountt0=0;
	for ( int i = 2; i < scaleN-1; ++i)
  {
    float *b = multiscaleDx[i];
	 float *b_cha = chaDx[i];
    float *s_cha = chaDx[i-1];
    float *t_cha = chaDx[i+1];
    // loop over middle response layer at density of the most 
    // sparse layer (always top), to find maxima across scale and space
	 double inverse_scalei = 1.f/(scale[i]*scale[i]);// normalisation factor
	 double inverse_scalei_1 = 1.f/(scale[i-1]*scale[i-1]);
	 double inverse_scaleijia1 = 1.f/(scale[i+1]*scale[i+1]);

	 int rs=scale[i+1];
	 int rp=im0.rows-scale[i+1];
	 int cs=scale[i+1];
	 int cp=im0.cols-scale[i+1];
    for (int r = rs; r < rp; ++r)
    {
		float *pb=b+r*im0.cols;
		float *pb_cha=b_cha+r*im0.cols;
		float *ps_cha=s_cha+r*im0.cols;
		float *pt_cha=t_cha+r*im0.cols;
      for (int c = cs; c < cp; ++c)
      {

		  float candidate=pb[c];
		  int ifExtremum=-1;
		  if( candidate>=pb[c-1]&& candidate>=pb[c+1])//极大值//只比较左右 
		  {
				float cha_candidate=pb_cha[c]*inverse_scalei;
				if(cha_candidate<ps_cha[c]*inverse_scalei_1&& cha_candidate<pt_cha[c]*inverse_scaleijia1)//取尺度上的极小值 
					ifExtremum=7;
		  }
		  if( candidate<pb[c-1]&& candidate<pb[c+1])//极小值//左右 
		  {
				float cha_candidate=pb_cha[c]*inverse_scalei;
				if(cha_candidate>=ps_cha[c]*inverse_scalei_1&& cha_candidate>=pt_cha[c]*inverse_scaleijia1)//取尺度上的极大值 
					ifExtremum=7;
		  }
		  // check the point is above th
		  if(ifExtremum==7)
		  if(ABS(candidate*inverse_scalei) < th) 
				ifExtremum=-1;
		  if(ifExtremum==7)
		  {
			  float responsey=multiscaleDy[i][r*im0.cols+c];
			  if(ABS(candidate)>=5.5*ABS(responsey))//18度倾角 
				  ;
			  else
				  ifExtremum=-1;
		  }

        if(ifExtremum==7)
        {
			  KeyPoint *kpt0=&Pseries0[pCountt0].kpt;
			  kpt0->pt.x=c;
			  kpt0->pt.y=r;
			  kpt0->size=scale[i];
			  //??kpt.responce 
			  //??kpt.angle
			  Pseries0[pCountt0].ifpdx=pianDX;
			  pCountt0++;
          //interpolateExtremum(r, c, b, s, t);
        }
      }
    }
  }
	return pCountt0;
}
//! Non Maximal Suppression function
int isExtremumX(int r, int c, float *b, float *b_cha, float *s_cha, float *t_cha,  float th, int scalei, int scalei_1, int scaleijia1, Mat im0)
{
  
  /*for (int rr = r-1; rr <=r+1; ++rr)
  {
	 float *ptr_b=t+
    for (int cc = c-1; cc <=c+1; ++cc)
    {
      // if any response in 3x3x3 is greater candidate not maximum
      if (
        t->getResponse(r+rr, c+cc) >= candidate ||
        ((rr != 0 || cc != 0) && m->getResponse(r+rr, c+cc, t) >= candidate) ||
        b->getResponse(r+rr, c+cc, t) >= candidate
        ) 
        return -1;
    }
  }
  */
  float candidate=b[r*im0.cols+c];
  int ifExtremum=-1;
  if( candidate>=b[r*im0.cols+c-1]&& candidate>=b[r*im0.cols+c+1])//极大值//只比较左右 
  {
	  float cha_candidate=b_cha[r*im0.cols+c]/(scalei*scalei);
	  if(cha_candidate<s_cha[r*im0.cols+c]/(scalei_1*scalei_1)&& cha_candidate<t_cha[r*im0.cols+c]/(scaleijia1*scaleijia1))//取尺度上的极小值 
		  ifExtremum=7;
  }
  if( candidate<b[r*im0.cols+c-1]&& candidate<b[r*im0.cols+c+1])//极小值//左右 
  {
	  float cha_candidate=b_cha[r*im0.cols+c]/(scalei*scalei);
	  if(cha_candidate>=s_cha[r*im0.cols+c]/(scalei_1*scalei_1)&& cha_candidate>=t_cha[r*im0.cols+c]/(scaleijia1*scaleijia1))//取尺度上的极大值 
		  ifExtremum=7;
  }

  // check the point is above th
  if(ifExtremum==7)
  if (ABS(candidate/(scalei*scalei)) >= th) 
    return 7; 

  return -1;
}

//-------------------------------------------------------

int DetectY( featureP *Pseries0, float **multiscaleDx, float **multiscaleDy, float **chaDy, int *scale, int scaleN, int flag, int th, Mat im0)
{
	int pCountt0=0;
	for ( int i = 2; i < scaleN-1; ++i)
  {
    float *b = multiscaleDy[i];	 
	 float *b_cha = chaDy[i];
    float *s_cha = chaDy[i-1];
    float *t_cha = chaDy[i+1];
	 double inverse_scalei = 1.f/(scale[i]*scale[i]);// normalisation factor
	 double inverse_scalei_1 = 1.f/(scale[i-1]*scale[i-1]);
	 double inverse_scaleijia1 = 1.f/(scale[i+1]*scale[i+1]);

	 int rs=scale[i+1];
	 int rp=im0.rows-scale[i+1];
	 int cs=scale[i+1];
	 int cp=im0.cols-scale[i+1];
    for (int r = rs; r < rp; ++r)
    {
		float *pb=b+r*im0.cols;
		float *pbup=b+(r-1)*im0.cols;
		float *pbdown=b+(r+1)*im0.cols;
		float *pb_cha=b_cha+r*im0.cols;
		float *ps_cha=s_cha+r*im0.cols;
		float *pt_cha=t_cha+r*im0.cols;
      for (int c = cs; c < cp; ++c)
      {
		  
		  float candidate=pb[c];
		  int ifExtremum=-1;
		  if( candidate>=pbup[c]&& candidate>=pbdown[c])//极大值//上下 
		  {
				float cha_candidate=pb_cha[c]*inverse_scalei;
				if(cha_candidate<ps_cha[c]*inverse_scalei_1&& cha_candidate<pt_cha[c]*inverse_scaleijia1)//取尺度上的极小值 
					ifExtremum=7;
		  }
		  if( candidate<pbup[c]&& candidate<pbdown[c])//极小值//上下 
		  {
				float cha_candidate=pb_cha[c]*inverse_scalei;
				if(cha_candidate>=ps_cha[c]*inverse_scalei_1&& cha_candidate>=pt_cha[c]*inverse_scaleijia1)//取尺度上的极大值 
					ifExtremum=7;
		  }
		  // check the point is above th
		  if(ifExtremum==7)
		  if(ABS(candidate*inverse_scalei) < th) 
				ifExtremum=-1;
		  if(ifExtremum==7)
		  {
			  float responsex=multiscaleDx[i][r*im0.cols+c];
			  if(ABS(candidate)>=5.5*ABS(responsex))
				  ;
			  else
				  ifExtremum=-1;
		  }

        if(ifExtremum==7)
        {
			  KeyPoint *kpt0=&Pseries0[pCountt0].kpt;
			  kpt0->pt.x=c;
			  kpt0->pt.y=r;
			  kpt0->size=scale[i];
			  //??kpt.responce 
			  //??kpt.angle
			  Pseries0[pCountt0].ifpdx=pianDY;
			  pCountt0++;
          //interpolateExtremum(r, c, b, s, t);
        }
      }
    }
  }
	return pCountt0;
}
int isExtremumY(int r, int c, float *b, float *b_cha, float *s_cha, float *t_cha,  float th, int scalei, int scalei_1, int scaleijia1, Mat im0)
{

  float candidate=b[r*im0.cols+c];
  int ifExtremum=-1;
  if( candidate>=b[(r-1)*im0.cols+c]&& candidate>=b[(r+1)*im0.cols+c])//极大值//上下 
  {
	  float cha_candidate=b_cha[r*im0.cols+c]/(scalei*scalei);
	  if(cha_candidate<s_cha[r*im0.cols+c]/(scalei_1*scalei_1)&& cha_candidate<t_cha[r*im0.cols+c]/(scaleijia1*scaleijia1))//取尺度上的极小值 
		  ifExtremum=7;
  }
  if( candidate<b[(r-1)*im0.cols+c]&& candidate<b[(r+1)*im0.cols+c])//极小值//上下 
  {
	  float cha_candidate=b_cha[r*im0.cols+c]/(scalei*scalei);
	  if(cha_candidate>=s_cha[r*im0.cols+c]/(scalei_1*scalei_1)&& cha_candidate>=t_cha[r*im0.cols+c]/(scaleijia1*scaleijia1))//取尺度上的极大值 
		  ifExtremum=7;
  }

  // check the point is above th
  if(ifExtremum==7)
  if (ABS(candidate/(scalei*scalei)) >= th) 
    return 7;
  return -1;
}

//-------------------------------------------------------

int averaging( IplImage *im0, IplImage *im0_int, int filterSize)//cal averaging by int
{
  unsigned char *ptr0;
  int *i_data = (int *) im0_int->imageData;  

  for( int r=filterSize/2; r<im0->height-filterSize/2; r++) 
  {
	  ptr0=(unsigned char *)im0->imageData+r*im0->widthStep;
	  for(int c=filterSize/2; c<im0->width-filterSize/2; c++) 
    {
      ptr0[c]=sumYikuai(im0_int, r-filterSize/2, c-filterSize/2, filterSize, filterSize); 
    }
  }
  return -1;
}

//-------------------------------------------------------

Mat Integral(Mat im0)
{
  // assume the image tobe single channel
	//cc7 
	//convert to grey
	/*
	*/
  Mat im0_int;
  im0_int.create(cvSize(im0.cols,im0.rows),CV_32SC1);//int32

  // set up variables for data access
  int height = im0.rows;
  int width = im0.cols;
  unsigned char *data;  
  int *i_data;
  int *i_datap;

  // first row only
  data=im0.ptr(0);
  int rs = 0;
  i_data=(int *)im0_int.ptr(0);
  for(int j=0; j<width; j++) 
  {
    rs += data[j]; 
    i_data[j] = rs;
  }

  // remaining cells are sum above and to the left
  for(int i=1; i<height; ++i) 
  {
    rs = 0;
	 data=im0.ptr(i);
	 i_data=(int *)im0_int.ptr(i);
	 i_datap=(int *)im0_int.ptr(i-1);
    for(int j=0; j<width; ++j) 
    {
      rs += data[j]; 
      i_data[j] = i_datap[j] + rs;
    }
  }

  //cc7
  // release the gray image
  //cvReleaseImage(&img);

  // return the integral image
  return im0_int;
}

int sumYikuai(Mat im0_int, int row, int col, int rows, int cols) 
{
  int *data = (int *) im0_int.ptr(0);
  int step = im0_int.step/sizeof(int);

  // The subtraction by one for row/col is because row/col is inclusive.
  /*int r1 = std::min(row,im0->height) - 1;
  int c1 = std::min(col,          im0->width)  - 1;
  int r2 = std::min(row + rows,   im0->height) - 1;
  int c2 = std::min(col + cols,   im0->width)  - 1;
  */
  int r1 = row;
  int c1 = col;
  int r2 = row + rows;
  int c2 = col + cols;
  
  int A, B, C, D;
  A = data[r1 * step + c1];
  B = data[r1 * step + c2];
  C = data[r2 * step + c1];
  D = data[r2 * step + c2];

  //return std::max(0, A - B - C + D);
  return A - B - C + D;
}

//-------------------------------------------------------