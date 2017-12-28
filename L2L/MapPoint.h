
#ifndef PPstruct_def
#define PPstruct_def

#include <cv.h>
#include <cxcore.h>

#define DXDY 0
#define pianDX 1
#define pianDY 2
typedef struct{
	int r;
	int c;
	cv::KeyPoint kpt;
	int ifpdx;
	int histG[32];
	float levelD;
}featureP;

typedef struct{
	int idx0;
	int idx1;
	int ifpdx;
	double distance;
}Match;

typedef struct{
	int r;
	int c;
}PP;
typedef struct{
	double x;
	double y;
	double z;
}PP3D;
#endif