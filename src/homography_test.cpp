#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDot2.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>

#include <ros/ros.h>
#include <ros/param.h>

#include <image_transport/image_transport.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/version.hpp>

#include <opencv-2.3.1/opencv2/opencv.hpp>

#include <visp_camera_calibration/CalibPointArray.h>

#include "conversion.hh"
#include "callbacks.hh"


void printMat(CvMat *A)
{
int i, j;
for (i = 0; i < A->rows; i++)
{
printf("\n"); 
switch (CV_MAT_DEPTH(A->type))
{
case CV_32F:
case CV_64F:
for (j = 0; j < A->cols; j++)
printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
break;
case CV_8U:
case CV_16U:
for(j = 0; j < A->cols; j++)
printf ("%6d",(int)cvGetReal2D(A, i, j));
break;
default:
break;
}
}
printf("\n");
}

int main(int argc, char **argv)
{
	int N = 4;
	
	CvMat * src = cvCreateMat( N, 3, CV_64FC1);
	CvMat * srcT = cvCreateMat( 3, N, CV_64FC1);
	CvMat * dst = cvCreateMat( N, 3, CV_64FC1);
	CvMat * dstT = cvCreateMat( 3, N, CV_64FC1);
	
	CvMat * out = cvCreateMat( N, 3, CV_64FC1);
	CvMat * outT = cvCreateMat( 3, N, CV_64FC1);
	
	CvMat * homography_ = cvCreateMat(3, 3, CV_64FC1);
	CvMat * mask= cvCreateMat(1, N, CV_8UC1);
				
	
	src->data.db[0] = 0.0+1;
	src->data.db[1] = 0.0+1;
	src->data.db[2] = 1.0;
	
	src->data.db[3] = 1.0+1;
	src->data.db[4] = 0.0+1;
	src->data.db[5] = 1.0;
	
	src->data.db[6] = 1.0+1;
	src->data.db[7] = 1.0+1;
	src->data.db[8] = 1.0;
	
	src->data.db[9] =  0.0+1;
	src->data.db[10] = 1.0+1;
	src->data.db[11] = 1.0;
	
	cvTranspose(src, srcT);
	
	dst->data.db[0] = 0.0;
	dst->data.db[1] = 0.0;
	dst->data.db[2] = 1.0;
	
	dst->data.db[3] = 2.0;
	dst->data.db[4] = 0.0;
	dst->data.db[5] = 1.0;
	
	dst->data.db[6] = 2.0;
	dst->data.db[7] = 2.0;
	dst->data.db[8] = 1.0;
	
	dst->data.db[9] = 0.0;
	dst->data.db[10] = 2.0;
	dst->data.db[11] = 1.0;
	
	cvTranspose(dst, dstT);
	
	cvFindHomography(src, dst, homography_,0, 0, mask);
	
	printf("src");
	printMat(src);
	printf("srcT");
	printMat(srcT);
	printf("dst");
	printMat(dst);
	printf("hom");
	printMat(homography_);
	
	cvMatMul(homography_, srcT, outT);
	
	printf("outT");
	printMat(outT);
	
	cvMatMul(src, homography_, out);
	
	printf("out");
	printMat(out);
			
}
