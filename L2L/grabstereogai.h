
#ifndef GRABSTEREO
#define GRABSTEREO 

#include "MapPoint.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace cv;

/*//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "fc2triclops.h"

//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( description, error )	\
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "*** Triclops Error '%s' at line %d :\n\t%s\n", \
	 triclopsErrorToString( error ), \
	 __LINE__, \
	 description );	\
      printf( "Press any key to exit...\n" ); \
      getchar(); \
      exit( 1 ); \
   } \
}

//
// aliases namespaces
//
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// struct containing image needed for processing
struct ImageContainer
{

	FC2::Image unprocessed[2];	
	FC2::Image tmp[2];
   FC2::Image bgru[2];
   FC2::Image packed;
} ;
*/

#include <cv.h>
#include <cxcore.h>


extern double Intrinsic_matLL[][3];
extern double Distortion_coeffsLL[];
extern double Intrinsic_matLLBuffer[][3];
extern double Distortion_coeffsLLBuffer[];

extern int saveCountt;
extern int SeriesNo;
extern FILE *fp7;
int grabstereo0( time_t ct);
int allocating();

extern cv::Mat imLast;
extern cv::Mat im0R;
extern cv::Mat dLast;
extern cv::Mat tr4007;
int grabstereo( time_t ct);

cv::Mat TcwTransform( cv::Mat tr4007, cv::Mat Tcw);

void addonFPS(IplImage *img);

//cv::Mat img7;
extern cv::Mat img7colorLLORI;
extern cv::Mat img7colorRRORI;
extern cv::Mat img7colorLLRectified;//8位3通道 
extern cv::Mat img7colorRRRectified;
//extern cv::Mat img7rectified;
//extern cv::Mat img7rectifiedRR;
extern cv::Mat LLGreyRectified;
extern cv::Mat RRGreyRectified;
extern cv::Mat disparityImage7;

extern int width0;//图片宽
extern int height0;//图片的高 
#endif