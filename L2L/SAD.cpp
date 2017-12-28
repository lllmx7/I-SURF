// SAD.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "sad.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
using namespace cv;

#include "math.h"


double cornerTh=cornerThLUT[0];
int featureExtractingSURF( vector<KeyPoint> *keypoints0, Mat *descriptors0, Mat im0)
{
	
	// detecting keypoints
    SurfFeatureDetector detector(887);
    detector.detect(im0, *keypoints0);
	 if((*keypoints0).size()<17)
	 {
		 SurfFeatureDetector detector(287);
		 detector.detect(im0, *keypoints0);
	 }	 
    // computing descriptors
    SurfDescriptorExtractor extractor;
    extractor.compute(im0, *keypoints0, *descriptors0);	
	return -1;
}
int featureExtractingORB( vector<KeyPoint> *keypoints0, Mat *descriptors0, Mat im0)
{
	
	ORB orb7;
	orb7(im0, Mat(), *keypoints0, *descriptors0);
	return -1;
}


/*extern int _19;
SADreturn findDistinctive( Mat im0, Mat im0_int)
{

	///////
	SADreturn distinctiveMap0 = histSADlike(im0,im0_int,_19,_7);
	
	return distinctiveMap0;
}
*/


int dmax=89;//leftIplGrey->width/3;
//_19
int _19=19;
int _7=7;
double cornerThLUT[]={ 0.0019, 0.39, 0.49, 0.59, 0.69, 0.79, 0.89, 0.99, 1.1, 1.3, 1.5, 1.7, 1.9, 2.1, 2.2, 2.3, 2.6, 2.8, 2.9};//19 
double uniquenessRatio=0.39;
double thABS;


double uniquenessRatio1=1.5;
double uniquenessRatio2=0.99;
double uniquenessRatioMultiScale=0.89;//0.39;
int findCorrespondence( Match *m0, featureP *Pseries0, int CounttPs0, featureP *Pseries1, int CounttPs1, Mat img0, Mat img0_int, Mat img1, Mat img1_int)//p0去p1中找对应 
{

	int matchesCountt=0;
	for(int i=0;i<CounttPs0;i++)
	{

		int chaAbsMIN[3];//从小到大排 
		for(int k=0;k<3;k++)
			chaAbsMIN[k]=8888889;
		featureP MinCorresP[3];
		for(int k=0;k<3;k++)
		MinCorresP[k]=Pseries1[0];

		/*unsigned char *ptr0,ptr1;
		for(int r=-halfWindow; r<=halfWindow; r++)
		{
			ptr0=img0.ptr((int)Pseries0[i].y+r);
			for(int c=-halfWindow; c<=halfWindow; c++)
				local0[halfWindow+r][halfWindow+c]=ptr0[(int)Pseries0[i].x+c];
		}
		*/

		for(int j=0;j<CounttPs1;j++)
		{

			int sumchaAbs=0;
			for(int k=0;k<32;k++)
				sumchaAbs=sumchaAbs+ABS(Pseries1[j].histG[k]-Pseries0[i].histG[k]);

			int index=-1;
			for(int k=2;k>=0;k--)
			{
				if(sumchaAbs<chaAbsMIN[k])
					index=k;
				else
					break;
			}
			if(index>=0)
			{
				for(int k=2;k-1>=index;k--)//腾个位置先 
				{
					chaAbsMIN[k]=chaAbsMIN[k-1];
					MinCorresP[k]=MinCorresP[k-1];
				}
				chaAbsMIN[index]=sumchaAbs;//插入 
				MinCorresP[index]=Pseries1[j];
			}
			/*if(sumchaAbs<chaAbsMIN1)
			{
				chaAbsMIN1=sumchaAbs;
				MinCorres1=Pseries1[j];
				if(sumchaAbs<chaAbsMIN0)
				{
					int tt=chaAbsMIN0;//一、二换
					chaAbsMIN0=chaAbsMIN1;
					chaAbsMIN1=tt;
					PP Ptt=MinCorres0;
					MinCorres0=MinCorres1;
					MinCorres1=Ptt;
				}
			}
			*/
		}

		/////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////要么差别大、要么距离变化明显
		int ifmatch=-1;
		double thABS=0.03*uniquenessRatio*_19*_19;//绝对门槛可以适当放宽 
		if(chaAbsMIN[2]>=uniquenessRatio1*chaAbsMIN[0]+thABS)//if(chaAbsMIN[1]>=uniquenessRatio1*chaAbsMIN[0]+thABS)
			ifmatch=7;
		/*int pairJuLi0=(MinCorresP[0].x-m0[matchesCountt].p0.x)*(MinCorresP[0].x-m0[matchesCountt].p0.x)+(MinCorresP[0].y-m0[matchesCountt].p0.y)*(MinCorresP[0].y-m0[matchesCountt].p0.y);
		int pairJuLi1=(MinCorresP[1].x-m0[matchesCountt].p0.x)*(MinCorresP[1].x-m0[matchesCountt].p0.x)+(MinCorresP[1].y-m0[matchesCountt].p0.y)*(MinCorresP[1].y-m0[matchesCountt].p0.y);
		if(pairJuLi1>=3*pairJuLi0 ||pairJuLi0>=3*pairJuLi1)
			ifmatch=7;
			*/

		/*if(ifmatch==7)
		{
			m0[matchesCountt].p0=Pseries0[i];
///////////////////////////////////////、、、/距离判断机制【取近的】暂时没用上 
			int index=0;
			int pairJuLiMIN=(MinCorresP[0].r-m0[matchesCountt].p0.r)*(MinCorresP[0].r-m0[matchesCountt].p0.r)+(MinCorresP[0].c-m0[matchesCountt].p0.c)*(MinCorresP[0].c-m0[matchesCountt].p0.c);
			double th=uniquenessRatio2*chaAbsMIN[0];
			for(int k=0;k<3;k++)
			{
				if(chaAbsMIN[k]<th)//保证相似度够 
				{
					int pairJuLi=(MinCorresP[k].r-m0[matchesCountt].p0.r)*(MinCorresP[k].r-m0[matchesCountt].p0.r)+(MinCorresP[k].c-m0[matchesCountt].p0.c)*(MinCorresP[k].c-m0[matchesCountt].p0.c);
					if(3*3*pairJuLi<pairJuLiMIN)//距离明显小的多 
					{
						pairJuLiMIN=pairJuLi;
						index=k;
					}
				}
			}
			m0[matchesCountt].p1=MinCorresP[index];
			//m0[matchesCountt].p1=MinCorres0;
			matchesCountt++;
		}
		*/
		/////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////
		if(ifmatch==7)
		{
		ifmatch=-1;

		int index=-1;
		double thABSMultiScale=uniquenessRatioMultiScale*_19*_19;//绝对门槛可以适当放宽 
		//if(chaAbsMIN[2]>=uniquenessRatio1*chaAbsMIN[0]+thABS)//if(chaAbsMIN[1]>=uniquenessRatio1*chaAbsMIN[0]+thABS)
			//ifmatch=7;
		
		int WindowSizeL=_19*3/2;
		if(WindowSizeL%2==0)//保证是奇数
			WindowSizeL++;
		int halfWindowL=WindowSizeL/2;
		int WindowSizeXL=_19*2+1;//保证是奇数
		int halfWindowXL=WindowSizeXL/2;
		for(int j=0;j<1;j++)//第二个点不算 
		{
			if(chaAbsMIN[j]<thABSMultiScale)
			{
				
				int hist0[32];
				int hist1[32];
				if( Pseries0[i].r-halfWindowL<0|| Pseries0[i].c-halfWindowL<0|| Pseries0[i].r+halfWindowL>=img0.rows|| Pseries0[i].c+halfWindowL>=img0.cols)
					break;
				if( MinCorresP[j].r-halfWindowL<0|| MinCorresP[j].c-halfWindowL<0|| MinCorresP[j].r+halfWindowL>=img1.rows|| MinCorresP[j].c+halfWindowL>=img1.cols)
					continue;
				sumYikuai( hist0, img0_int, Pseries0[i].r-halfWindowL, Pseries0[i].c-halfWindowL, WindowSizeL, WindowSizeL);
				sumYikuai( hist1, img1_int, MinCorresP[j].r-halfWindowL, MinCorresP[j].c-halfWindowL, WindowSizeL, WindowSizeL);
				int sumchaAbs=0;
				for(int k=0;k<32;k++)
					sumchaAbs=sumchaAbs+ABS(hist0[k]-hist1[k]);
				//归一化
				sumchaAbs=sumchaAbs*_19*_19/(WindowSizeL*WindowSizeL);
				if(sumchaAbs<thABSMultiScale)
				{
					if( Pseries0[i].r-halfWindowXL<0|| Pseries0[i].c-halfWindowXL<0|| Pseries0[i].r+halfWindowXL>=img0.rows|| Pseries0[i].c+halfWindowXL>=img0.cols)
						break;
					if( MinCorresP[j].r-halfWindowXL<0|| MinCorresP[j].c-halfWindowXL<0|| MinCorresP[j].r+halfWindowXL>=img1.rows|| MinCorresP[j].c+halfWindowXL>=img1.cols)
						continue;
					sumYikuai( hist0, img0_int, Pseries0[i].r-halfWindowXL, Pseries0[i].c-halfWindowXL, WindowSizeXL, WindowSizeXL);
					sumYikuai( hist1, img1_int, MinCorresP[j].r-halfWindowXL, MinCorresP[j].c-halfWindowXL, WindowSizeXL, WindowSizeXL);
					sumchaAbs=0;
					for(int k=0;k<32;k++)
						sumchaAbs=sumchaAbs+ABS(hist0[k]-hist1[k]);
					//归一化
					sumchaAbs=sumchaAbs*_19*_19/(WindowSizeXL*WindowSizeXL);
					if(sumchaAbs<thABSMultiScale)
					{
						ifmatch=7;
						index=j;
						break;
					}
				}
			}
		}
		
		if(ifmatch==7)
		{
			m0[matchesCountt].idx0=i;
			m0[matchesCountt].idx1=-1;
			matchesCountt++;
		}
		}
		
	}

	return matchesCountt;
}


Mat _4show;
int matchesDrawing( Match *m0, int matchesCountt, featureP *Pseries0, featureP *Pseries1, Mat im0, Mat im1)
{
	_4show.create(cvSize(2*im0.cols,2*im0.rows),CV_8UC(3));
	unsigned char *ptr0,*ptr1;
	int w=3;
	for(int r=0;r<im0.rows;r++)
	{
		ptr0=im0.ptr(r);
		ptr1=_4show.ptr(r);
		for(int c=0;c<im0.cols;c++)
		{
			ptr1[c*w]=ptr0[c];
			ptr1[c*w+1]=ptr0[c];
			ptr1[c*w+2]=ptr0[c];
		}
	}
	for(int r=0;r<im0.rows;r++)
	{
		ptr0=im1.ptr(r);
		ptr1=_4show.ptr(r);
		for(int c=0;c<im0.cols;c++)
		{
			ptr1[(im0.cols+c)*w]=ptr0[c];
			ptr1[(im0.cols+c)*w+1]=ptr0[c];
			ptr1[(im0.cols+c)*w+2]=ptr0[c];
		}
	}
	for(int r=0;r<im0.rows;r++)
	{
		ptr0=im1.ptr(r);
		ptr1=_4show.ptr(im0.rows+r);
		for(int c=0;c<im0.cols;c++)
		{
			ptr1[c*w]=ptr0[c];
			ptr1[c*w+1]=ptr0[c];
			ptr1[c*w+2]=ptr0[c];
		}
	}

	for(int j=0;j<matchesCountt;j++)
	{
		//line( im2show, cvPoint(m0[j].p0.x,m0[j].p0.y), cvPoint(leftGrey.cols+m0[j].p1.x,m0[j].p1.y), cvScalar(0,255,0), 3);
		line( _4show, cvPoint(Pseries0[m0[j].idx0].kpt.pt.x,Pseries0[m0[j].idx0].kpt.pt.y), cvPoint(Pseries1[m0[j].idx1].kpt.pt.x,im0.rows+Pseries1[m0[j].idx1].kpt.pt.y), cvScalar(0,255,0), 1);
		line( _4show, cvPoint(Pseries0[m0[j].idx0].kpt.pt.x,Pseries0[m0[j].idx0].kpt.pt.y), cvPoint(im0.cols+Pseries1[m0[j].idx1].kpt.pt.x,Pseries1[m0[j].idx1].kpt.pt.y), cvScalar(0,255,0), 1);
		//imshow("_4show",_4show);
		//waitKey();
	}

	cvNamedWindow("_4show",1);
	imshow("_4show",_4show);
	waitKey();

	return -1;
}


int pDrawing( featureP *Pseries0, int pCountt0, Mat im0)
{
	_4show.create(cvSize(im0.cols,im0.rows),CV_8UC(3));
	cvNamedWindow("_4show",1);
	unsigned char *ptr0,*ptr1;
	int w=_4show.step/_4show.cols;
	for(int r=0;r<im0.rows;r++)
	{
		ptr0=im0.ptr(r);
		ptr1=_4show.ptr(r);
		for(int c=0;c<im0.cols;c++)
		{
			ptr1[c*w]=ptr0[c];
			ptr1[c*w+1]=ptr0[c];
			ptr1[c*w+2]=ptr0[c];
		}
	}

	for(int j=0;j<pCountt0;j++)
	{
		//line( im2show, cvPoint(m0[j].p0.x,m0[j].p0.y), cvPoint(leftGrey.cols+m0[j].p1.x,m0[j].p1.y), cvScalar(0,255,0), 3);
		line( _4show, cvPoint(Pseries0[j].kpt.pt.x,Pseries0[j].kpt.pt.y), cvPoint(Pseries0[j].kpt.pt.x+1,Pseries0[j].kpt.pt.y), cvScalar(0,255,0), 3);
		//画个框框
		rectangle( _4show, cvRect(Pseries0[j].kpt.pt.x-Pseries0[j].kpt.size/2,Pseries0[j].kpt.pt.y-Pseries0[j].kpt.size/2,Pseries0[j].kpt.size,Pseries0[j].kpt.size),cvScalar(0,0,255));
		imshow("_4show",_4show);
		waitKey();
	}

	imshow("_4show",_4show);
	waitKey();
	return -1;
}



int numberofCao=32;
Mat histIntegral(Mat img0)
{
  // assume the image tobe single channel
	//cc7 
	//convert to grey
	/*
	*/
  Mat im0_int;
  im0_int.create(cvSize(img0.cols,img0.rows),CV_32SC(numberofCao));//int32
//im0=19;
  
  // set up variables for data access
  int height = img0.rows;
  int width = img0.cols;
  unsigned char *data;  
  int *i_data0;
  int *i_datap0;

  // first row only
  data=img0.ptr(0);
  i_data0=(int *)im0_int.ptr(0);
  int rs[32];
  memset( rs, 0, 32*sizeof(int));
  for(int j=0; j<width; j++) 
  {
    int index=(*data)>>3;
    rs[index]++;
	 for(int k=0;k<numberofCao;k++)
    i_data0[k]=rs[k];//为了加快计算速度//im0_int[0][j][k] = rs[k];

	 data++;
	 i_data0=i_data0+numberofCao;
  }

  // remaining cells are sum above and to the left
  for(int i=1; i<height; ++i) 
  {
	 data=img0.ptr(i);
	 i_data0=(int *)im0_int.ptr(i);
	 i_datap0=(int *)im0_int.ptr(i-1);
    memset( rs, 0, 32*sizeof(int));
    for(int j=0; j<width; ++j) 
    {
		int index=(*data)>>3;
      rs[index]++;
		for(int k=0;k<numberofCao;k++)
      i_data0[k]=i_datap0[k]+rs[k];//为了加快计算速度//im0_int[i][j][k] = rs[k] + im0_int[i-1][j][k];

		data++;
		i_data0=i_data0+numberofCao;
		i_datap0=i_datap0+numberofCao;
    }
  }

  return im0_int;
}

int sumYikuai( int *hist, Mat im0_int, int row, int col, int rows, int cols) 
{
  //int hist[32];
  //int *data = (int *) im0_int.ptr(0);
  //int step = im0_int.step/sizeof(int);

  int r1 = row;
  int c1 = col;
  int r2 = row + rows;
  int c2 = col + cols;
  
  int *A,*B,*C,*D;
  A=(int *)im0_int.ptr(r1)+c1*numberofCao;//为了加快计算速度 
  B=(int *)im0_int.ptr(r1)+c2*numberofCao;
  C=(int *)im0_int.ptr(r2)+c1*numberofCao;
  D=(int *)im0_int.ptr(r2)+c2*numberofCao;
  for(int j=0;j<numberofCao;j++)
	  hist[j]=D[j]-C[j]-B[j]+A[j];//im0_int[r1][c1][j]-im0_int[r1][c2][j]-im0_int[r2][c1][j]+im0_int[r2][c2][j];

  /*int A, B, C, D;
  if (r1 >= 0 && c1 >= 0) A = data[r1 * step + c1];
  if (r1 >= 0 && c2 >= 0) B = data[r1 * step + c2];
  if (r2 >= 0 && c1 >= 0) C = data[r2 * step + c1];
  if (r2 >= 0 && c2 >= 0) D = data[r2 * step + c2];
  return A - B - C + D;
  */
  return -1;
  //return hist;
}


Mat Integral0(Mat im0)
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
  //cvReleaseImage(&im0);

  // return the integral image
  return im0_int;
}

int sumYikuai0( Mat im0_int, int row, int col, int rows, int cols) 
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


#if 0
 ///////////////////////////////////////
 ///////////////////////////////////////ransac
 ///////////////////////////////////////
 int getSubset7( Match *matchSelected, int *IndexofmatchSelected, Match *matches, int matchesCountt, int matchSelectedCountt)//matchSelectedCountt是选中点数目，matchesCountt是matches的数目 
{
    
	 // 初始化随机数种子
	 // time函数返回从1970年1月1日零时零分零秒到目前为止所经过的时间，单位为秒
	 srand((int)time(NULL));
	 for(int j=0;j<matchSelectedCountt;j++)
	 {
		int s=rand()%matchesCountt;
		matchSelected[j]=matches[s];
		IndexofmatchSelected[j]=s;
	 }
	 return -1;
}


double errCal7(Match *matches, int matchesCountt, int *mask, int Inliers, double T[][4])
{
	double err;
	err=0;

	for(int j=0;j<matchesCountt;j++)
	if(mask[j]==7)
	{
	  err=err+(line7.k*matches[j].x+line7.b-matches[j].y)*(line7.k*matches[j].x+line7.b-matches[j].y);
	}

	err=err/Inliers;

	return err;
}


//y=kx+b
double errTh=0.89;
double beta=0.7;
int runCal70( double T[][4], Match *matches, int matchesCountt, int *mask, int Inliers)//matchSelectedCountt是选中点数目，matchesCountt是matches的数目 
{
	//数据归一化先 
	Match matches1[889];
	for(int j=0;j<matchesCountt;j++)
		matches1[j]=matches[j];
	for(int j=0;j<matchesCountt;j++)
	{
		matches1[j].x=matches1[j].x/361;
		matches1[j].y=matches1[j].y/361;
	}
	line7->b=line7->b/361;

	double err;
	double dk,ddk,dkb;
	double db,ddb;

	for( int iter = 0; iter < 3; iter++ )//牛顿法、很快就收敛了 
	{
		err=errCal7(matches1, matchesCountt, mask, Inliers, *line7);

		//if(err<errTh)
			//break;

		dk=0;
		db=0;
		ddk=0;
		dkb=0;
		for(int j=0;j<matchesCountt;j++)
		if(mask[j]==7)
		{
			dk=dk+2*(line7->k*matches1[j].x+line7->b-matches1[j].y)*matches1[j].x;
			db=db+2*(line7->k*matches1[j].x+line7->b-matches1[j].y);
			ddk=ddk+2*matches1[j].x*matches1[j].x;
			dkb=dkb+2*matches1[j].x;
		}
		dk=dk/Inliers;
		db=db/Inliers;
		ddk=ddk/Inliers;
		ddb=2;
		dkb=dkb/Inliers;

		double g00=ddk;
		double g01=dkb;
		double g10=g01;
		double g11=ddb;
		double invg00=g11/(g00*g11 - g01*g10);
		double invg01=-g01/(g00*g11 - g01*g10);
		double invg10=-g10/(g00*g11 - g01*g10);
		double invg11=g00/(g00*g11 - g01*g10);

		line7->k=line7->k-beta*(invg00*dk+invg01*db);//更新在此 牛顿法 
		line7->b=line7->b-beta*(invg10*dk+invg11*db);
	}

	//数据得还原 
	line7->b=line7->b*361;

	/*double err,errLast;
	errLast=2.1475e+009;
	double kLast;
	double bLast;
	double dk;
	double db;

	beta=0.007;
	for( int iter = 0; iter < 89; iter++ )
	{
		err=errCal7(matches, matchesCountt, mask, Inliers, *line7);

		if(err<errTh)
			break;

		if(err>=errLast)//如果beta大了， 
		{
			line7->k=kLast;
			line7->b=bLast;
			beta=beta/3;
		}
		else
		{
			errLast=err;
			kLast=line7->k;
			bLast=line7->b;
		}

		dk=0;
		db=0;
		for(int j=0;j<matchesCountt;j++)
		if(mask[j]==7)
		{
			dk=dk+2*(line7->k*matches[j].x+line7->b-matches[j].y)*matches[j].x;
			db=db+2*(line7->k*matches[j].x+line7->b-matches[j].y);
		}
		dk=dk/Inliers;
		db=db/Inliers;

		line7->k=line7->k-beta*dk;//更新在此 
		line7->b=line7->b-beta*db;
	}
	*/
	return -1;
}


int offsetTh=7;
int findInliers7( int *mask, Match *matches, int matchesCountt, LL line7)
{
	//init 
	int Count=0;
	for(int j=0;j<matchesCountt;j++)
		mask[j]=-1;
	//

	for(int j=0;j<matchesCountt;j++)
	{
		if(abs(line7.k*matches[j].x+line7.b-matches[j].y)<offsetTh)
		if(matches[j].y!=0)//因为有好多0所以要排除 
		{
			mask[j]=7;
			Count++;
		}
	}

	return Count;
}


int runCal7( double T[][4], int *mask, Match *matches, int matchesCountt, int *IndexofmatchSelected)//matchSelectedCountt是选中点数目，matchesCountt是matches的数目 
{

	
	
	//init

	line7.k=(matches[IndexofmatchSelected[1]].y-matches[IndexofmatchSelected[0]].y)/(matches[IndexofmatchSelected[1]].x-matches[IndexofmatchSelected[0]].x);
	line7.b=matches[IndexofmatchSelected[0]].y-line7.k*matches[IndexofmatchSelected[0]].x;
line7.k=3;
	//

	for( int iter = 0; iter < 19; iter++ )
	{
		//求内点 
		int Inliers=findInliers7( mask, matches, matchesCountt, line7);

		//什么时候break?

		//求参 
		runCal70( T, matches, matchesCountt, mask, Inliers);
	}

	//curve dealing//没这个了上一次的残留、
	
	return -1;
}


int runranSAc7( double T[][4], int* mask, Match* matches, int matchesCountt,
                                    double confidence, int maxIters)
{
    int result = -1;
    //err
    
    //int count = 
    //if( count >= modelPoints )modelPoints是一次最少要挑出来的点数目 
	 int InliersGlobal=-1;
	 double T7[4][4];
	 int mask7[889];
	 for(int j=0;j<matchesCountt;j++)
		mask7[j]=-1;
	 
	 int iter, niters = maxIters;
    for( iter = 0; iter < niters; iter++ )
    {
		  Match matchSelected[7];
		  int IndexofmatchSelected[7];
        getSubset7( matchSelected, IndexofmatchSelected, matches, matchesCountt, 2);//挑2个直接算
for(int j=0;j<7;j++)
IndexofmatchSelected[j]=j;

        runCal7( T7, mask7, matches, matchesCountt, IndexofmatchSelected);//计算最好的情况】、 
        
		  //find Inliers
		  int Inliers=0;
		  for(int j=0;j<matchesCountt;j++)
		  if(mask7[j]==7)
			  Inliers++;

		  if(Inliers>=InliersGlobal)
        {
					 InliersGlobal=Inliers;
                *line=line7;
					 for(int j=0;j<matchesCountt;j++)
						 mask[j]=mask7[j];
                //niters = cvRANSACUpdateNumIters( confidence,//不知道怎么弄的】先放着 
                    //(double)(count - goodCount)/count, modelPoints, niters );
        }
	 }

    if( (double)InliersGlobal/matchesCountt>=confidence )//检查是否占了大多数 
    {
		 result=7;
    }
	 else
		 result=-1;

    return result;
}
#endif