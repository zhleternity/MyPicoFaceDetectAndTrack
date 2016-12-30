#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "allTracking.h"

using namespace std;
using namespace cv;

struct LINE
{
	double a;
	double b;
	double c;
	cv::Point start;
	cv::Point end;
};


class MyPico
{
public:
	vector<cv::Rect> rects_detect;
	cv::Rect toTrack;
	void help();
	void processImage(cv::Mat &frame, ALLTracks track, int drawFlag, bool usepyr, bool NO_Clistering, bool verbose, void *cascade, float angle, float qthreshold, float scalefactor, float stridefactor, int minsize, int maxsize);
	void process_webcam_frames(char *video_path, ALLTracks track, int drawFlag, bool usepyr, bool NO_Clistering, bool verbose, void *cascade, float angle, float qthreshold, float scalefactor, float stridefactor, int minsize, int maxsize);
private:
	double myMax(double a, double b);
	double myMin(double a, double b);
	int run_cascade(void* cascade, float* o, int r, int c, int s, void* vppixels, int nrows, int ncols, int ldim);
	int run_rotated_cascade(void* cascade, float* o, int r, int c, int s, float a, void* vppixels, int nrows, int ncols, int ldim);
	int find_objects
		(
		float rs[], float cs[], float ss[], float qs[], int maxndetections,
		void* cascade, float angle, // * `angle` is a number between 0 and 1 that determines the counterclockwise in-plane rotation of the cascade: 0.0f corresponds to 0 radians and 1.0f corresponds to 2*pi radians
		void* pixels, int nrows, int ncols, int ldim,
		float scalefactor, float stridefactor, float minsize, float maxsize
		);
	float get_overlap(float r1, float c1, float s1, float r2, float c2, float s2);
	int find_connected_components(int a[], float rs[], float cs[], float ss[], int n);
	int cluster_detections(float rs[], float cs[], float ss[], float qs[], int n);
	void ccdfs(int a[], int i, float rs[], float cs[], float ss[], int n);
	float get_ticks();
	//int getRectOverlap(cv::Rect r1, cv::Rect r2);
	double line_size(cv::Point &p1, cv::Point &p2);
	float area(cv::Point &pt, cv::Point &p1, cv::Point &p2);
	bool isInRectangle(cv::Point& pt, vector<cv::Point>& pr);
	double area_vertice_intersection(vector<cv::Point> &points);
	double cross_product(cv::Point &a, cv::Point &b);
	cv::Point vector_g(cv::Point &a, cv::Point &b);
	void DecideOberlap(vector<cv::Point> &pr1, vector<cv::Point> &pr2, Size imgsize, int& k1, int & k2);
	LINE makeline(cv::Point &p1, cv::Point &p2);
	bool lineintersect(LINE l1, LINE l2, cv::Point &p);
	bool online(LINE l, cv::Point p);
	bool isOverlap(cv::Rect &rc1, cv::Rect &rc2);
	float computeRectJoinUnion(cv::Rect &rc1, cv::Rect &rc2, float& AJoin, float& AUnion);
};
