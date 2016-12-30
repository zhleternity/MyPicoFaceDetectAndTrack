#include "CMT/CMT.h"
#include "ct/CompressiveTracker.h"
#include "Camshift/Camshift.h"




class ALLTracks
{
public:
	cv::Rect initBox = cv::Rect(0, 0, 0, 0);
	cv::Rect box;
	//cv::Rect selectBox;
	cv::Rect rect_new;
	bool beginInit;
	bool startTracking;
	
	//camshift tracking
	Camshift *camshift;
	//CMT tracker
	cmt::CMT *cmtTracker;
	//CT tracker
	CompressiveTracker *ctTracker;
	//color tracker
	//cv::colortracker::ColorTracker *colorTracker;

	//main function
	void trackInit();
	//void select_init_box();
	void processImage(cv::Mat &image, cv::Rect faceBox);


private:
	
	typedef enum {
		CT_TRACKER,
		TLD_TRACKER,
		CMT_TRACKER,
		COLOR_TRACKER,
		CAMSHIFT_TRACKER,
		STRUCK_TRACKER,
	}TrackType;

	TrackType track_type;
	void camshiftTracking(cv::Mat &frame, cv::Rect faceBox);
	void cmtTracking(cv::Mat &frame, cv::Rect &faceBox);
	void ctTracking(cv::Mat &frame, cv::Rect &faceBox);
	void colorTracking(cv::Mat &frame, cv::Rect &faceBox);



};

