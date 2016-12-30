#include "allTracking.h"

using namespace cv;

void ALLTracks::trackInit(){

	beginInit = false;
	startTracking = false;
	//camshift
	track_type = CAMSHIFT_TRACKER;
	//cmt
	//track_type = CMT_TRACKER;
	//CT
	//track_type = CT_TRACKER;
	//color tracker
	//track_type = COLOR_TRACKER;


}



//camshift
void ALLTracks::camshiftTracking(cv::Mat &frame, cv::Rect faceBox){
	if (beginInit){
		if (camshift != NULL)
			delete camshift;
	}
		//camshift = new Camshift();
	Camshift camshift;
		//if need change the initBox or not
		//std::cout << "faceBox: " << faceBox.x << "," << faceBox.y << ","<< faceBox.width << "," << faceBox.height << std::endl;
		if (0 == initBox.area())
		    initBox = faceBox;
		else
		{
			//cv::Point c1 = cv::Point(initBox.x + initBox.width / 2, initBox.y + initBox.height / 2);
			//cv::Point c2 = cv::Point(faceBox.x + faceBox.width / 2, faceBox.y + faceBox.height / 2);
			//if (initBox == faceBox || (sqrt(pow(c1.x - c2.x, 2) + pow(c1.y - c2.y, 2)) < 5))
			//{
				initBox = rect_new;
				
			//}
		}
		initBox = faceBox;
		camshift.initialize(frame, initBox);
		std::cout << "Camshift tracking init!" << std::endl;
		startTracking = true;
		beginInit = false;
	//}
		
	
	
	//bool res;
	if (startTracking){
		std::cout << "Camshift tracking process..." << std::endl;
		camshift.processFrame(frame);
		RotatedRect rect_rotate = camshift.objectBox;
		Point2f vertices[4];
		rect_rotate.points(vertices);
		rect_new = rect_rotate.boundingRect();

		//for (int i = 0; i < 4; i++)
			//line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 255));
		rectangle(frame, rect_new, Scalar(255, 0, 255), 3);
	}
	//return res;
}

//cmt
void ALLTracks::cmtTracking(cv::Mat &frame, cv::Rect &faceBox){
	cv::Mat img_gray;
	cvtColor(frame, img_gray, CV_BGR2GRAY);
	if (beginInit)
	{
		if (cmtTracker != NULL)
			delete cmtTracker;

		cmtTracker = new cmt::CMT();
		initBox = faceBox;
		cmtTracker->initialize(img_gray, initBox);
		std::cout << "cmt tracker init!" << std::endl;
		startTracking = true;
		beginInit = false;
	}

	if (startTracking)
	{
		std::cout << "cmt tracker process..." << std::endl;
		cmtTracker->processFrame(img_gray);
		for (size_t i = 0; i < cmtTracker->points_active.size(); i++)
			circle(frame, cmtTracker->points_active[i], 2, Scalar(0, 255, 0));
		RotatedRect rect = cmtTracker->bb_rot;
		Point2f vertices[4];
		rect.points(vertices);
		for (int i = 0; i < 4; i++)
			line(frame, vertices[i], vertices[(i+1)%4], Scalar(0, 255, 255));
	}	
}

//CT
void ALLTracks::ctTracking(cv::Mat &frame, cv::Rect &faceBox){
	cv::Mat img_gray;
	cvtColor(frame, img_gray, CV_BGR2GRAY);
	//init
	if (beginInit){
		if (ctTracker != NULL)
			delete ctTracker;

		ctTracker = new CompressiveTracker();
		initBox = faceBox;
		//std::cout << "CT tracker init!" << std::endl;
		ctTracker->init(img_gray, initBox);
		//std::cout << "init CT box: " << initBox.x << "," << initBox.y << "," << initBox.width << "," << initBox.height << std::endl;
		box = initBox;
		//rectangle(frame, initBox, Scalar(255, 0, 0), 1);
		startTracking = true;
		beginInit = false;
	}

	if (startTracking)
	{
		//std::cout << "ct tracking process... " << std::endl;
		ctTracker->processFrame(img_gray, box);
		rectangle(frame, box, Scalar(0, 255, 255),3);
	}
}

////color tracker
//void ALLTracks::colorTracking(cv::Mat &frame, cv::Rect &bbox)
//{
//	if (beginInit)
//	{
//		if (colorTracker != NULL)
//			delete colorTracker;

//	cv::colortracker::ColorTrackerParameters params;
//	params.visualization = 0;
//	initBox = bbox;
//	params.init_pos.x = initBox.x + initBox.width / 2;
//	params.init_pos.y = initBox.y + initBox.height / 2;
//	params.wsize = initBox.size();
//
//	colorTracker = new cv::colortracker::ColorTracker(params);
//	colorTracker->init_tracking();
//	std::cout << "color tracker init!" << std::endl;
//	startTracking = true;
//	beginInit = false;
//	}

//
//	if (startTracking)
//	{
//		std::cout << "color tracker process..." << std::endl;
//		cv::Rect rect = colorTracker->track_frame(frame);
//		rectangle(frame, rect, Scalar(0, 255, 255), 1);
//	}
//
//}
//
////
void ALLTracks::processImage(cv::Mat &image, cv::Rect faceBox){
	
	switch (track_type)
	{
	case CAMSHIFT_TRACKER:
		camshiftTracking(image, faceBox);
		break;
	case CMT_TRACKER:
		cmtTracking(image, faceBox);
		break;
	case CT_TRACKER:
		ctTracking(image, faceBox);
		break;
	case TLD_TRACKER:
		break;
	case COLOR_TRACKER:
		//colorTracking(image, faceBox);
		break;
	case STRUCK_TRACKER:
		break;
	default:
		break;
	}
}