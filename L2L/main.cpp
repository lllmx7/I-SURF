//=============================================================================
// Copyright ?2004 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// grabstereo
//
// Gets input from the Bumblebee, and performs stereo processing
// to create a disparity image. A rectified image from the reference camera
// and the disparity image are both written out.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "grabstereogai.h"
#include "LUT.h"
#include "L2L.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace cv;


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//执行一帧一帧抓图
///////////////////////////////////////////////////////////////////
int
main( )
{
    /*// grabbed unprocessed image
    FC2::Image grabbedImage;//原始图片
    // grab image from camera.
    // this image contains both right and left images
    if ( grabImage( camera, grabbedImage, if1stTime) )//抓图 
    {
		return EXIT_FAILURE;
    }
	 // right and left image extracted from grabbed image
    ImageContainer imageCont;
	 TriclopsInput triclopsColorInput;
	 TriclopsInput     triclopsInput;
    // generate triclops input from grabbed image
    if ( generateTriclopsColorInput( grabbedImage, imageCont, triclopsColorInput ) )//提取彩色图片
    {
		return EXIT_FAILURE;
    }	 
    // carry out the stereo pipeline 
    if ( doStereo( triclops, triclopsColorInput) )//图片修正、求视差；【是以彩色图片作为输入triclopsColorInput】
    {
		return EXIT_FAILURE;
    }
	 */

	 
	 //char* leftname="2010_03_09_drive_0019/2010_03_09_drive_0019/I1_000001.png";
	 char fileL[89];
	 sprintf(fileL,"1s_626/LLgrey_%.0lf.jpg", (double)ct);
	 char fileR[89];
	 sprintf(fileR,"1s_626/RRgrey_%.0lf.jpg", (double)ct);
	 IplImage* leftIpl= cvLoadImage(fileL);
	 IplImage* rightIpl= cvLoadImage(fileR);
	//convert to grey
	IplImage *leftIplGrey=cvCreateImage(cvSize(leftIpl->width,leftIpl->height),8,1);
	unsigned char *ptr0,*ptr1;
	int w=leftIpl->widthStep/leftIpl->width;
	for(int i=0;i<leftIplGrey->height;i++)
	{
		ptr0=(unsigned char *)(leftIpl->imageData+i*leftIpl->widthStep);
		ptr1=(unsigned char *)(leftIplGrey->imageData+i*leftIplGrey->widthStep);
		for(int j=0;j<leftIplGrey->width;j++)
			ptr1[j]=ptr0[j*w];
	}

	IplImage* rightIplGrey=cvCreateImage(cvSize(rightIpl->width,rightIpl->height),8,1);
	for(int i=0;i<rightIplGrey->height;i++)
	{
		ptr0=(unsigned char *)(rightIpl->imageData+i*rightIpl->widthStep);
		ptr1=(unsigned char *)(rightIplGrey->imageData+i*rightIplGrey->widthStep);
		for(int j=0;j<rightIplGrey->width;j++)
			ptr1[j]=ptr0[j*w];
	}

	Mat im1L=leftIplGrey;
//resize(im1L,im1L,cvSize(0,0),0.5,0.5);
	Mat im1R=rightIplGrey;
//resize(im1R,im1R,cvSize(0,0),0.5,0.5);
/*Mat imEq;
equalizeHist( d1, imEq);
imshow("bm",imEq);
waitKey(2);
*/
///////////////////////////////////////////////////////////////////////
//slam
///////////////////////////////////////////////////////////////////////

	 Mat Tcw_es=Mat(cvSize(4,4),CV_32FC1);
	 float *ptrF=(float *)Tcw_es.ptr(0);
	 ptrF[0]=1;
	 ptrF[5]=1;
	 ptrF[10]=1;
	 ptrF[15]=1;
	 Mat Tcw=L2L( imLast, im0R, im1L, -1, Tcw_es);
float tt[16];
if(Tcw.cols!=0)
{
float *ptr0F;
for(int r=0;r<4;r++)
{
ptr0F=Tcw.ptr<float>(r);
for(int c=0;c<4;c++)
tt[4*r+c]=ptr0F[c];
}
}
	 if(Tcw.cols!=0)
	 {
	 imLast=im1L;
	 im0R=im1R;

	 tr4007=TcwTransform( tr4007, Tcw);
	 
	 float *ptrF0=(float *)tr4007.ptr(0);
	 fprintf( fp7, "\n%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, ", ptrF0[0], ptrF0[1], ptrF0[2], ptrF0[3], ptrF0[4], ptrF0[5], ptrF0[6], ptrF0[7], ptrF0[8], ptrF0[9], ptrF0[10], ptrF0[11]);
	 }
	 else
	 {
imLast=im1L;
im0R=im1R;
		 printf("\nTcwComputingFailed!!");
		 fprintf( fp7, "\nTcwComputingFailed!!");
	 }
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
	 cvReleaseImage(&leftIpl);
	 cvReleaseImage(&rightIpl);
	 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 //因为还有用，不能清 
	 //cvReleaseImage(&leftIplGrey);
	 //cvReleaseImage(&rightIplGrey);
    return EXIT_SUCCESS;
}
//int
//main( int /* argc */, char** /* argv */ )


///////////////////////////////////////////获得原始彩色图像 
Mat img7colorLLORI,img7colorRRORI;//opencv默认是三通道bgr、照相机里是bgru、那个u不知道有啥用？暂丢掉. 
int width0;//图片宽
int height0;//图片的高 
// generare triclops input
/*int generateTriclopsColorInput( const FC2::Image & grabbedImage, 
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput )
									 */


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//图像校正、生成视差图主要的活都在这里 
///////////////////////////////////////////////////////////////////
Mat img7colorLLRectified,img7colorRRRectified;
Mat LLGreyRectified;
Mat RRGreyRectified;
Mat disparityImage7;

double Intrinsic_matLL[][3]={535.12379, 0, 327.94,
	 0, 537.573, 231.689,
	 0, 0, 1};
double Distortion_coeffsLL[]={ -0.37746,   0.21336,   -0.00059,   0.00024,  0};
double Intrinsic_matRR[][3]={536.24, 0, 313.091,
	 0, 538.578, 239.768,
	 0, 0, 1};
double Distortion_coeffsRR[]={ -0.35238,   0.14470,   -0.00009,   0.00035,  0};
/*int doStereo( TriclopsContext const & triclops, 
                TriclopsInput const & triclopsColorInput)
					 */


Mat TcwTransform( Mat tr4007, Mat Tcw)
{
	Mat invTcw;
	invert(Tcw,invTcw);
	Mat tr4007gai;
	tr4007gai=tr4007*invTcw;
	/*Mat tr4007gai;
	tr4007gai.create(tr4007.size(),CV_32FC1);
	tr4007gai=0;
	float *ptrF0,*ptrF1,*ptrF2;
	ptrF1=(float *)tr4007.ptr(0);
	for(int i=0;i<4;i++)
	{
		ptrF0=(float *)invTcw.ptr(i);
		ptrF2=(float *)tr4007gai.ptr(i);
		for(int j=0;j<4;j++)
		{
			for(int k=0;k<4;k++)
				ptrF2[j]=ptrF2[j]+ptrF0[k]*ptrF1[k*tr4007.step/sizeof(float)+j];
		}
	}
	*/
	return tr4007gai;
}


// addon the FPS figure on the image (requires at least 2 calls)
void addonFPS(IplImage *img)//暂时只能给一幅图加 
{
  static int counter = 0;
  static clock_t t;
  static float fps;
  char fps_text[20];
  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1.0,1.0,0,2);

  // Add fps figure (every 10 frames)
  if (counter > 10)
  {
    fps = (10.0f/(clock()-t) * CLOCKS_PER_SEC);
    t=clock(); 
    counter = 0;
  }

  // Increment counter
  ++counter;

  // Get the figure as a string
  sprintf(fps_text,"FPS: %.1f",fps);

  // Draw the string on the image
  cvPutText (img,fps_text,cvPoint(10,25), &font, cvScalar(0,255,0));
}