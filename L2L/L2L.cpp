// L2L.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "L2L.h"
#include "sad.h"
#include "Harris仿写.h"
#include "PnPsolver.h"
#include "_thread7.h"

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
using namespace cv;

#include <windows.h>
#include "time.h"
#include "stdlib.h"
#include <string>


Mat L2L( Mat im0, Mat im0R, Mat im1, int if1stTime, Mat Tcw_es)//返回CV_32FC1数据类型 
{

	if(if1stTime==7)
	{
		cv::Mat Tcw=Mat(cvSize(4,4),CV_32FC1);
		Tcw=0;
		float *ptrF=(float *)Tcw.ptr(0);
		ptrF[0]=1;
		ptrF[5]=1;
		ptrF[2*Tcw.step/sizeof(float)+2]=1;
		ptrF[3*Tcw.step/sizeof(float)+3]=1;
//fpCal=fopen("fpCal.txt","wt+");
		return Tcw;
	}
	
	//PnPsolver* pSolver = new PnPsolver( m000,matchesCountt,Pseries0,Pseries1,Pseries0X,Pseries0Y,Pseries1X,Pseries1Y,d0);
   //pSolver->SetRansacParameters( 0.7, 4, -1 , 17, 0.3 , 5.991, Tcw);//0.091表示内点占一共匹配数的百分比 
	//cv::Mat Tcw = L2L000( im0, im0R, im1, Tcw_es);
/*if(Tcw.cols==0)
{
	cv::Mat Tcw1 = L2L111( im0, im0R, im1, Tcw_es);
	if(Tcw1.cols!=0)
	{
	return Tcw1;
	}
}
*/
	cv::Mat Tcw1 = L2L111( im0, im0R, im1, Tcw_es);
	return Tcw1;
}



vector<KeyPoint> keypoints0,keypoints1;
featureP Pseries0lins[8887];
int d0[8887];
BFMatcher matcher7(NORM_L1);
BFMatcher matcherH(NORM_HAMMING);
vector<DMatch> m000;
bool cmpDMatch( const DMatch &d0, const DMatch &d1)
{
	return d0.distance<d1.distance;
}
Mat L2L000( Mat im0L, Mat im0R, Mat im1L, Mat Tcw_es)
{
	///featureExtracting
	//vector<KeyPoint> keypoints0;
	keypoints0.clear();
	Mat descriptors0;
	featureExtractingSURF( &keypoints0, &descriptors0, im0L);
	int CountPs0=keypoints0.size();
	for( int j=0;j<keypoints0.size();j++)
	{
		Pseries0lins[j].r=keypoints0[j].pt.y;
		Pseries0lins[j].c=keypoints0[j].pt.x;
	}
	//ocv_zhaod( d0, im0L, im0R, Pseries0lins, CountPs0, _19, 32);
	Mat dMat0=ocv_sgbm(im0L, im0R);
	ocv_zhaod222( d0, im0L, im0R, dMat0, Pseries0lins, CountPs0);
	//vector<KeyPoint> keypoints1;
	keypoints1.clear();
	Mat descriptors1;
	featureExtractingSURF( &keypoints1, &descriptors1, im1L);

	///Matching 
	// matching descriptors
	m000.clear();
	int i,j,k;
	if(keypoints0.size()>=1&& keypoints1.size()>=1)
	{
   matcher7.match(descriptors0, descriptors1, m000);
	std::sort(m000.begin(),m000.end(),cmpDMatch);//小的排前面 
	int cutIdx=m000.size();
	for( i=0;i<m000.size();i++)
	if(m000[i].distance>=1.5)
	{
		cutIdx=i;
		break;
	}
	m000.resize(cutIdx);
	}
    // drawing the results
    namedWindow("matches", 1);
	 Mat img_matches;
    drawMatches( im0L, keypoints0, im1L, keypoints1, m000, img_matches);
	 //drawKeypoints( im0L, keypoints0, img_matches);
    imshow("matches", img_matches);
	 waitKey(0);
	 //string strm="img_matchesSURF_";
	 //strm=strm+timeStamp+".jpg";
	 char strm[87];
	 sprintf(strm,"img_matchesSURF_%.0lf.jpg",(double)timeStamp);
	 imwrite(strm,img_matches);
	 /*for(int j=0;j<m000.size();j++)
	 {
	 m789.clear();
	 m789.push_back(m000[j]);
    Mat img_matches;
    drawMatches( im0L, keypoints0, im1L, keypoints1, m789, img_matches);
    imshow("matches", img_matches);
    waitKey(0);
	 }
	 */
		
	//Bucketing
	if(0)
	{
	vector<DMatch> m000_Bucketing[4][7];
	int bwX=im0L.cols/7;
	int bwY=im0L.rows/4;
	for(int i=0;i<m000.size();i++)
	{
		KeyPoint *kpt=&keypoints0[m000[i].queryIdx];
		int b=kpt->pt.y/bwY;
		int c=kpt->pt.x/bwX;
		m000_Bucketing[b][c].push_back(m000[i]);
	}
	m000.clear();
	for(int b=0;b<4;b++)
		for(int c=0;c<7;c++)
		{
			if(m000_Bucketing[b][c].size()>3)
			{
				/*//random选3个 
				srand( (unsigned)time( NULL ) );
				for(int iter=0;iter<3;iter++)
				{
				int idx=rand()%m000_Bucketing[b][c].size();
				m000.push_back(m000_Bucketing[b][c][idx]);
				m000_Bucketing[b][c][idx]=m000_Bucketing[b][c][m000_Bucketing[b][c].size()-1];
				m000_Bucketing[b][c].pop_back();
				}
				*/
				//按顺序 
				for(int i=0;i<m000_Bucketing[b][c].size();i++)
					for(int j=0;j<m000_Bucketing[b][c].size()-1;j++)
					{
						if(m000_Bucketing[b][c][j].distance>=m000_Bucketing[b][c][j+1].distance)
						{
							DMatch m000lins=m000_Bucketing[b][c][j];
							m000_Bucketing[b][c][j]=m000_Bucketing[b][c][j+1];
							m000_Bucketing[b][c][j+1]=m000lins;
						}
					}
				for(int i=0;i<3;i++)
					m000.push_back(m000_Bucketing[b][c][i]);
			}
			else
			{
			for(int i=0;i<m000_Bucketing[b][c].size();i++)
				m000.push_back(m000_Bucketing[b][c][i]);
			}
		}
	}


	if(m000.size()<6)//都小于6了还算个啥啊
		return Mat();
	PnPsolver* pSolver = new PnPsolver( m000, keypoints0, keypoints1, d0);
   pSolver->SetRansacParameters( 0.7, 6, -1 , 6, 0.3, 5.991, Tcw_es);//0.091表示内点占一共匹配数的百分比 
	// Perform 4 Ransac Iterations
	vector<bool> vbInliers;
   int nInliers;
	Mat Tcw = pSolver->iterate(18887,vbInliers,nInliers);//287,vbInliers,nInliers);
	//keypoints0.clear();
   //vector<KeyPoint> (keypoints0).swap(keypoints0);
	//keypoints1.clear();
   //vector<KeyPoint> (keypoints1).swap(keypoints1);
	return Tcw;
}


featureP Pseries111_0[8887],Pseries111_1[8887],Pseries0X[8887],Pseries0Y[8887],Pseries1X[8887],Pseries1Y[8887];
int d0X[8887],d0Y[8887];
int ifpdx[8887];
vector<DMatch> m0X,m0Y;
vector<DMatch> m000SB0;
Match m111[8887];
double InliersP111;
Mat L2L111( Mat im0L, Mat im0R, Mat im1L, Mat Tcw_es)
{
	///featureExtracting
	//vector<KeyPoint> keypoints0;
	Mat descriptors0;
	featureExtractingSURF( &keypoints0, &descriptors0, im0L);	
	int i,j,k;
	for( i=0;i<keypoints0.size();i++)
		Pseries111_0[i].kpt=keypoints0[i];
	int pCountt111_0=keypoints0.size();
	//vector<KeyPoint> keypoints1;
	Mat descriptors1;
	featureExtractingSURF( &keypoints1, &descriptors1, im1L);
	for( i=0;i<keypoints1.size();i++)
		Pseries111_1[i].kpt=keypoints1[i];
	int pCountt111_1=keypoints1.size();

SYSTEMTIME st;
GetLocalTime(&st);
int time0=st.wSecond*1000+st.wMilliseconds;
	Mat im0_int=Integral0( im0L);
	Mat im1_int=Integral0( im1L);
	Mat descriptors0X;
	int pCountt0X=featureExtractingHarrisDx( Pseries0X, &descriptors0X, im0L, im0_int);
	Mat descriptors0Y;
	int pCountt0Y=featureExtractingHarrisDy( Pseries0Y, &descriptors0Y, im0L, im0_int);
	Mat descriptors1X;
	int pCountt1X=featureExtractingHarrisDx( Pseries1X, &descriptors1X, im1L, im1_int);
	Mat descriptors1Y;
	int pCountt1Y=featureExtractingHarrisDy( Pseries1Y, &descriptors1Y, im1L, im1_int);

	for( int j=0;j<pCountt111_0;j++)
	{
		Pseries111_0[j].r=Pseries111_0[j].kpt.pt.y;
		Pseries111_0[j].c=Pseries111_0[j].kpt.pt.x;
	}
	//ocv_zhaod( d0, im0L, im0R, Pseries111_0, pCountt111_0, _19, 32);
	Mat dMat0=ocv_sgbm(im0L, im0R);
	ocv_zhaod222( d0, im0L, im0R, dMat0, Pseries111_0, pCountt111_0);
	for( i=0;i<pCountt0X;i++)
	{
		Pseries0X[i].r=Pseries0X[i].kpt.pt.y;
		Pseries0X[i].c=Pseries0X[i].kpt.pt.x;
	}
	//ocv_zhaod( d0X, im0L, im0R, Pseries0X, pCountt0X, _19, 32);
	ocv_zhaod222( d0X, im0L, im0R, dMat0, Pseries0X, pCountt0X);
	for( i=0;i<pCountt0Y;i++)
	{
		Pseries0Y[i].r=Pseries0Y[i].kpt.pt.y;
		Pseries0Y[i].c=Pseries0Y[i].kpt.pt.x;
	}
	ocv_zhaod222( d0Y, im0L, im0R, dMat0, Pseries0Y, pCountt0Y);

	///Matching 
	memset( ifpdx, 0x00, (pCountt111_0+pCountt0X+pCountt0Y)*sizeof(int));
	// matching descriptors
	m000SB0.clear();
	m000.clear();
	if(keypoints0.size()>=1&& keypoints1.size()>=1)
	{
   matcher7.match(descriptors0, descriptors1, m000);
	std::sort(m000.begin(),m000.end(),cmpDMatch);//小的排前面 
	int cutIdx=m000.size();
	for( i=0;i<m000.size();i++)
	if(m000[i].distance>=1.5)
	{
		cutIdx=i;
		break;
	}
	m000.resize(MIN(cutIdx,87));
	}
	for(int i=0;i<m000.size();i++)
		m000SB0.push_back(m000[i]);
	int idxEnd=m000.size();
	//
	m0X.clear();
	if(pCountt0X>=1&& pCountt1X>=1)
	{
	matcher7.match(descriptors0X, descriptors1X, m0X);
	std::sort(m0X.begin(),m0X.end(),cmpDMatch);//小的排前面 
	int cutIdx=m0X.size();
	for( i=0;i<m0X.size();i++)
	if(m0X[i].distance>=1.5)
	{
		cutIdx=i;
		break;
	}
	m0X.resize(MIN(cutIdx,87));	
	}
	for(int i=0;i<m0X.size();i++)
		m000SB0.push_back(m0X[i]);
	for(int i=0;i<m0X.size();i++)
		ifpdx[idxEnd+i]=pianDX;
	idxEnd=m000SB0.size();
	//
	m0Y.clear();
	if(pCountt0Y>=1&& pCountt1Y>=1)
	{
	matcher7.match(descriptors0Y, descriptors1Y, m0Y);
	std::sort(m0Y.begin(),m0Y.end(),cmpDMatch);
	int cutIdx=m0Y.size();
	for( i=0;i<m0Y.size();i++)
	if(m0Y[i].distance>=1.5)
	{
		cutIdx=i;
		break;
	}
	m0Y.resize(MIN(cutIdx,87));
	}
	for( i=0;i<m0Y.size();i++)
		m000SB0.push_back(m0Y[i]);
	for( i=0;i<m0Y.size();i++)
		ifpdx[idxEnd+i]=pianDY;
	// drawing the results
    //namedWindow("matches", 1);
	 //Mat img_matches;
    //drawMatches( im0L, keypoints0, im1L, keypoints1, m000, img_matches);
    //imshow("matches", img_matches);
    //waitKey(0);
	// drawing the results
/*namedWindow("matches", 1);
Mat img_matches;
drawMatches( im0L, refKF->mvKeysUn, im1L, F->mvKeysUn, m000, img_matches);
imshow("matches", img_matches);
waitKey(0);
vector<DMatch> m789;
vector<KeyPoint> ptlins0,ptlins1;
ptlins0.clear();
ptlins1.clear();
for(int i=0;i<pCountt0X;i++)
	ptlins0.push_back(Pseries0X[i].kpt);
for(int i=0;i<pCountt1X;i++)
	ptlins1.push_back(Pseries1X[i].kpt);
Mat im0Ljiajia=Mat(cvSize(im0L.cols,im0L.rows),CV_8UC3);
Mat im1Ljiajia=Mat(cvSize(im0L.cols,im0L.rows),CV_8UC3);
int w=3;
for(int r=0;r<im0L.rows;r++)
{
unsigned char *ptr0,*ptr1,*ptr2,*ptr3;
ptr0=img_matches.ptr(r);
ptr1=img_matches.ptr(r)+im0L.cols*w;
ptr2=im0Ljiajia.ptr(r);
ptr3=im1Ljiajia.ptr(r);
for(int c=0;c<im0L.cols;c++)
{
ptr2[0]=ptr0[0];
ptr3[0]=ptr1[0];
ptr2[1]=ptr0[1];
ptr3[1]=ptr1[1];
ptr2[2]=ptr0[2];
ptr3[2]=ptr1[2];
ptr0=ptr0+w;
ptr1=ptr1+w;
ptr2=ptr2+w;
ptr3=ptr3+w;
}
}
drawMatches( im0Ljiajia, ptlins0, im1Ljiajia, ptlins1, m0X, img_matches);
ptlins0.clear();
ptlins1.clear();
for(int i=0;i<pCountt0Y;i++)
	ptlins0.push_back(Pseries0Y[i].kpt);
for(int i=0;i<pCountt1Y;i++)
	ptlins1.push_back(Pseries1Y[i].kpt);
for(int r=0;r<im0L.rows;r++)
{
unsigned char *ptr0,*ptr1,*ptr2,*ptr3;
ptr0=img_matches.ptr(r);
ptr1=img_matches.ptr(r)+im0L.cols*w;
ptr2=im0Ljiajia.ptr(r);
ptr3=im1Ljiajia.ptr(r);
for(int c=0;c<im0L.cols;c++)
{
ptr2[0]=ptr0[0];
ptr3[0]=ptr1[0];
ptr2[1]=ptr0[1];
ptr3[1]=ptr1[1];
ptr2[2]=ptr0[2];
ptr3[2]=ptr1[2];
ptr0=ptr0+w;
ptr1=ptr1+w;
ptr2=ptr2+w;
ptr3=ptr3+w;
}
}
drawMatches( im0Ljiajia, ptlins0, im1Ljiajia, ptlins1, m0Y, img_matches);
imshow("matches", img_matches);
waitKey(0);
	 //char strm[87];
	 //sprintf(strm,"img_matchesSURF++_%.0lf.jpg",(double)timeStamp);
	 //imwrite(strm,img_matches);
	 /*{
	 case DXDY:
		 pts0.push_back(Pseries111_0[m000SB0[j].queryIdx].kpt);
		 pts1.push_back(Pseries111_1[m000SB0[j].trainIdx].kpt);
		 break;
	 case pianDX:
		 pts0.push_back(Pseries0X[m000SB0[j].queryIdx].kpt);
		 pts1.push_back(Pseries1X[m000SB0[j].trainIdx].kpt);
		 break;
	 case pianDY:
		 pts0.push_back(Pseries0Y[m000SB0[j].queryIdx].kpt);
		 pts1.push_back(Pseries1Y[m000SB0[j].trainIdx].kpt);
		 break;
	 default:
		 pts0.push_back(Pseries111_0[m000SB0[j].queryIdx].kpt);
		 pts1.push_back(Pseries111_1[m000SB0[j].trainIdx].kpt);
		 break;
	 }
	 */

	int leaveOut=_19;
	int searchWithin=1.5*_19;
	int matchesCountt=0;
	for(int i=0;i<m000SB0.size();i++)
	{

					int idx1=m000SB0[i].trainIdx;
					//if(ABS(Pseries1[idx1].kpt.pt.x-p1_es.c)<searchWithin&& ABS(Pseries1[idx1].kpt.pt.y-p1_es.r)<searchWithin)
					{
						m111[matchesCountt].idx0=m000SB0[i].queryIdx;
						m111[matchesCountt].idx1=idx1;
						m111[matchesCountt].ifpdx=ifpdx[i];
						m111[matchesCountt].distance=m000SB0[i].distance;
						matchesCountt++;
					}

	}

	//Bucketing
	if(1)
	{
	vector<Match> m000_Bucketing[7][13];
	int bwX=im0L.cols/13;
	int bwY=im0L.rows/7;
	for(int i=0;i<matchesCountt;i++)
	{
		double ptx,pty;
		switch(m111[i].ifpdx)
		{
		case DXDY:
			ptx=keypoints1[m111[i].idx1].pt.x;
			pty=keypoints1[m111[i].idx1].pt.y;
			break;
		case pianDX:
			ptx=Pseries1X[m111[i].idx1].kpt.pt.x;
			pty=Pseries1X[m111[i].idx1].kpt.pt.y;
			break;
		case pianDY:
			ptx=Pseries1Y[m111[i].idx1].kpt.pt.x;
			pty=Pseries1Y[m111[i].idx1].kpt.pt.y;
			break;
		}
		int b=pty/bwY;
		int c=ptx/bwX;
		m000_Bucketing[b][c].push_back(m111[i]);
	}
	matchesCountt=0;
	for(int b=0;b<7;b++)
		for(int c=0;c<13;c++)
		{
			if(m000_Bucketing[b][c].size()>3)
			{
				//按顺序 
				for(int i=0;i<m000_Bucketing[b][c].size();i++)
					for(int j=0;j<m000_Bucketing[b][c].size()-1;j++)
					{
						if(m000_Bucketing[b][c][j].distance>=m000_Bucketing[b][c][j+1].distance)
						{
							Match m000lins=m000_Bucketing[b][c][j];
							m000_Bucketing[b][c][j]=m000_Bucketing[b][c][j+1];
							m000_Bucketing[b][c][j+1]=m000lins;
						}
					}
				for(int i=0;i<3;i++)
				{
					m111[matchesCountt]=m000_Bucketing[b][c][i];
					matchesCountt++;
				}
				for(int i=3;i<m000_Bucketing[b][c].size();i++)
				if(m000_Bucketing[b][c][i].ifpdx==DXDY)//surf不减 
				{
					m111[matchesCountt]=m000_Bucketing[b][c][i];
					matchesCountt++;
				}
			}
			else
			{
			for(int i=0;i<m000_Bucketing[b][c].size();i++)
			{
				m111[matchesCountt]=m000_Bucketing[b][c][i];
				matchesCountt++;
			}
			}
		}
	}
/*Mat img_matchesgai;
m789.clear();
ptlins0.clear();
ptlins1.clear();
for( i=0;i<matchesCountt;i++)
{
	switch(m111[i].ifpdx)
	{
	case DXDY:
		ptlins0.push_back(keypoints0[m111[i].idx0]);
		ptlins1.push_back(keypoints1[m111[i].idx1]);
		break;
	case pianDX:
		ptlins0.push_back(Pseries0X[m111[i].idx0].kpt);
		ptlins1.push_back(Pseries1X[m111[i].idx1].kpt);
		break;
	case pianDY:
		ptlins0.push_back(Pseries0Y[m111[i].idx0].kpt);
		ptlins1.push_back(Pseries1Y[m111[i].idx1].kpt);
		break;
	}
	DMatch Mone;
	Mone.queryIdx=i;
	Mone.trainIdx=i;
	m789.push_back(Mone);
}
drawMatches( im0L, ptlins0, im1L, ptlins1, m789, img_matchesgai);
imshow("matches", img_matchesgai);
waitKey(0);
*/

	if(matchesCountt<6)//都小于6了还算个啥啊
		return Mat();
	PnPsolver* pSolver = new PnPsolver( im0L, im1L, m111,matchesCountt,Pseries111_0,Pseries111_1,Pseries0X,Pseries1X,Pseries0Y,Pseries1Y,d0,d0X,d0Y);
   pSolver->SetRansacParameters( 0.7, 6, -1 , 6, 0.6, 5.991, Tcw_es);//0.091表示内点占一共匹配数的百分比 
	// Perform 4 Ransac Iterations
	vector<bool> vbInliers;
   int nInliers;
	Mat Tcw = pSolver->iterate(1887,vbInliers,nInliers);//287,vbInliers,nInliers);
//InliersP111=(double)nInliers/matchesCountt;
	return Tcw;
}


int ocv_zhaod( int *d1, Mat im1L, Mat im1R, featureP *Pseries1, int CounttPs1, int _19, int dMAX)
{
	int halfWindow=_19/2;
	unsigned char local0[39][39],local1[39][39];
	int cha[39][39];
	for(int j=0;j<CounttPs1;j++)
	{
		featureP *p1=&Pseries1[j];

		for(int m=0,p=p1->r-halfWindow;m<_19;m++,p++)
		{
			uchar *pl0=im1L.ptr(p);
			for(int n=0,q=p1->c-halfWindow;n<_19;n++,q++)
				local0[m][n]=pl0[q];
		}

		int chaMIN=88888889;
		int d=-1;
		for(int offset=0;offset<dMAX;offset++)
		{
			if(p1->c-halfWindow-offset>=0)
			{
			int chasumAbs=0;
			for(int m=0,p=p1->r-halfWindow;m<_19;m++,p++)
			{
				uchar *pr0=im1R.ptr(p);
				for(int n=0,q=p1->c-halfWindow-offset;n<_19;n++,q++)
					chasumAbs=chasumAbs+ABS(pr0[q]-local0[m][n]);
			}

			if(chasumAbs<chaMIN)
			{
				chaMIN=chasumAbs;
				d=offset;
			}
			}
		}
		
		d1[j]=d;
	}

	return -1;
}


int ocv_zhaod222( int *d0, Mat im0L, Mat im0R, Mat dMat, featureP *Pseries0, int CounttPs0)
{

	for(int j=0;j<CounttPs0;j++)
	{
		featureP *p0=&Pseries0[j];
		int *ptr_dMat=(int *)dMat.ptr(p0->r);
		d0[j]=ptr_dMat[p0->c];
	}
	return -1;
}


//opencv中BM算法获得视差图//就是sad 
//int d0=14.9/640*640;//disparity修正、和图片大小有关 
Mat ocv_bm(Mat left0, Mat right0){

	Mat disp,disp7,disp8;

	static cv::StereoBM sbm;

    sbm.state->SADWindowSize = 19;
	 sbm.state->numberOfDisparities = (left0.cols/3>>4)<<4;//112;

//open cv左面一大块黑边不作匹配，所以要先加宽一些把那个让过去 
	 //左面加宽 
	 Mat leftEX,rightEX;
	 copyMakeBorder(left0, leftEX, 0, 0, sbm.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	 copyMakeBorder(right0, rightEX, 0, 0, sbm.state->numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	 sbm.state->uniquenessRatio = 9;//5;

	 //sbm.state->preFilterType; // =CV_STEREO_BM_NORMALIZED_RESPONSE now
    //sbm.state->preFilterSize=5; // averaging window size: ~5x5..21x21
    //sbm.state->preFilterCap=31; // the output of pre-filtering is clipped by [-preFilterCap,preFilterCap]
    /*bm.state->textureThreshold = 10;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1
	 */

    sbm(leftEX, rightEX, disp);///////////////////////////////////////////////////////////////////视差计算函数在这里 
	 //cvFindStereoCorrespondenceBM(
	 // 截取与原始画面对应的视差区域（舍去加宽的部分）
	 disp7 = disp.colRange(sbm.state->numberOfDisparities, disp.cols);
//imshow("ooo", disp);
/*FILE *fp7;
fp7=fopen("disp7.txt","wt+");
int dat;
unsigned char *ptr0,*ptr1;
int w=disp7.step/disp7.cols;
for(int r=0;r<disp7.rows;r++)
{
fprintf( fp7, "\n");
ptr0=disp7.ptr<uchar>(r);
for(int c=0;c<disp7.cols;c++)
{
if(ptr0[c*w+1]&0x80)//负的
	dat=0;
else
dat=(ptr0[c*w]+((int)ptr0[c*w+1]<<8))>>4;
fprintf( fp7, "%d, ", dat);
}
}
*/
    //normalize(disp7, disp8, 0.1, 255, CV_MINMAX, CV_8UC1);

//open cv的结果大了16倍，这里换算成真实视差图、没匹配上的视差=-1 
	 Mat disp_bmCV_32SC1=Mat( disp7.size(), CV_32SC1);
	 unsigned char *ptr0;
	 int *p1Int32;
	 int w=2;//返回int16
	 for(int r=0;r<disp7.rows;r++)
	 {
		ptr0=disp7.ptr(r);
		p1Int32=(int *)disp_bmCV_32SC1.ptr(r);
		for(int c=0;c<disp7.cols;c++)
		{
			if(ptr0[1]&0x80)//负的
				p1Int32[c]=-1;
			else
			p1Int32[c]=(ptr0[0]+((int)ptr0[1]<<8))>>4;

			ptr0=ptr0+w;
		}
	 }
/*FILE *fp7;
fp7=fopen("disp_bmCV_32SC1.txt","wt+");
int dat;
//unsigned char *ptr0,*ptr1;
for(int r=0;r<disp_bmCV_32SC1.rows;r++)
{
fprintf( fp7, "\n");
ptr0=disp_bmCV_32SC1.ptr<uchar>(r);
for(int c=0;c<disp_bmCV_32SC1.cols;c++)
{
dat=ptr0[c];
fprintf( fp7, "%d, ", dat);
}
}
*/
	///视差修正 
	return disp_bmCV_32SC1;
}


//opencv中SGBM算法获得视差图
Mat ocv_sgbm(Mat left,Mat right){

	Mat disp;
	static cv::StereoSGBM sgbm;
	//sgbm参数设置
	sgbm.SADWindowSize=9;
	sgbm.numberOfDisparities=32;
	sgbm.uniquenessRatio=9;//5;

	sgbm.preFilterCap=31;	
	sgbm.P1=4*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2=32*sgbm.SADWindowSize*sgbm.SADWindowSize;
	
	//sgbm.speckleWindowSize=-1;
	sgbm.disp12MaxDiff=-1;
	sgbm.fullDP=false;

	sgbm(left,right,disp);

	//disp.convertTo(disp8,CV_8U,255/(sgbm.numberOfDisparities*16.));
	//还要把结果除16才是实际上的视差 
	//disp.convertTo(disp8,CV_8UC1,1/16.);
	 Mat disp_bm_Int32=Mat( disp.size(), CV_32SC1);
	 unsigned char *ptr0;
	 int *p1Int32;
	 int w=2;//返回int16
	 for(int r=0;r<disp_bm_Int32.rows;r++)
	 {
		ptr0=disp.ptr(r);
		p1Int32=(int *)disp_bm_Int32.ptr(r);
		for(int c=0;c<disp_bm_Int32.cols;c++)
		{
			if(ptr0[1]&0x80)//负的
				p1Int32[c]=-1;
			else
			p1Int32[c]=(((int)ptr0[1]<<8)+ptr0[0])>>4;

			ptr0=ptr0+w;
		}
	 }

	return disp_bm_Int32;
}
