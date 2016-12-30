#include "pico.h"
//#include "ImageVideoTrans.h"


int main()
{
	void *cascade = 0;

	// set default parameters
	int minsize = 45;//45
	int maxsize = 1024;//300
	float angle = 0.0f;
	float scalefactor = 1.1f;
	float stridefactor = 0.1f;
	float qthreshold = 6.0f;
	bool usepyr = true;
	bool noclustering = false;
	bool verbose = false;

	MyPico pico;
	ALLTracks track;
	//help infos
	pico.help();

	int size;
	FILE* file;
	//char cascade_file_name[500] = "D:/opencv3.0.0-alpha/opencv/sources/data/haarcascades/haarcascade_frontalface_alt.xml";
	char cascade_file_name[500] = "D:\\zhlWorkDocs\\mot\\pico-master\\pico-master\\rnt\\cascades\\facefinder";
	fopen_s(&file, cascade_file_name, "rb");
	if (!file)
	{
		printf("# Can not read cascade from the path.\r\n");
		return 1;
	}
	fseek(file, 0L, SEEK_END);
	size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	cascade = malloc(size);
	if (!cascade || size != fread(cascade, 1, size, file))
		return 1;
	fclose(file);
	
	char input_path[500] = "D:/YinYueTai/sbs.mp4";
	string output_path = "";

	if (verbose)
	{
		//infos about params
		printf("# Copyright (c) 2016, zhleternity\r\n");
		printf("# All rights reserved.\r\n\n");

		printf("# cascade parameters:\r\n");
		printf("#	tsr = %f\r\n", ((float*)cascade)[0]);
		printf("#	tsc = %f\r\n", ((float*)cascade)[1]);
		printf("#	tdepth = %d\r\n", ((int*)cascade)[2]);
		printf("#	ntrees = %d\r\n", ((int*)cascade)[3]);
		printf("# detection parameters:\r\n");
		printf("#	minsize = %d\r\n", minsize);
		printf("#	maxsize = %d\r\n", maxsize);
		printf("#	scalefactor = %f\r\n", scalefactor);
		printf("#	stridefactor = %f\r\n", stridefactor);
		printf("#	qthreshold = %f\r\n", qthreshold);
		printf("#	usepyr = %d\r\n", usepyr);
	}
	double t = GetTickCount();
	pico.process_webcam_frames(input_path, track, 1, usepyr, noclustering, verbose, cascade, angle, qthreshold, scalefactor, stridefactor, minsize, maxsize);
	cout << "Time is: " << (GetTickCount() - t) / getTickFrequency() << "ms" << endl;
	return 0;
}







//CascadeClassifier face_cascade, eyes_cascade;
//String window_name = "Face Detection";
//
///**
// * Detects faces and draws an ellipse around them
// */
//void detectFaces(Mat frame) {
//
//  std::vector<Rect> faces;
//  Mat frame_gray;
//
//  // Convert to gray scale
//  cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
//
//  // Equalize histogram
//  equalizeHist(frame_gray, frame_gray);
//
//  // Detect faces
//  face_cascade.detectMultiScale(frame_gray, faces, 1.1, 3,
//				0|CASCADE_SCALE_IMAGE, Size(30, 30));
//
//  // Iterate over all of the faces
//  for(size_t i = 0; i < faces.size(); i++) {
//
//    // Find center of faces
//    Point center(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2);
//	ellipse(frame, center, Size(faces[i].width / 2, faces[i].height / 2),
//		   0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
//    Mat face = frame_gray(faces[i]);
//    std::vector<Rect> eyes;
//
//    // Try to detect eyes, inside each face
//    //eyes_cascade.detectMultiScale(face, eyes, 1.1, 2,
//				// CASCADE_SCALE_IMAGE, Size(30, 30) );
//
//    //if(eyes.size() > 0)
//    //  // Draw ellipse around face
//    //  ellipse(frame, center, Size(faces[i].width/2, faces[i].height/2),
//	   //   0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
//  }
//
//  // Display frame
//  imshow( window_name, frame );
//}
//
//
//int main() {
//
//  VideoCapture cap("D:/YinYueTai/sbs.mp4"); // Open default camera
//  Mat frame;
//  //D:/zhlWorkDocs/testWorksDemo/opencvTest/opencvTest/
//  face_cascade.load("D:/opencv3.0.0-alpha/opencv/sources/data/haarcascades/haarcascade_frontalface_alt.xml"); // load faces
//  eyes_cascade.load("D:/opencv3.0.0-alpha/opencv/sources/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml"); // load eyes
//
//  while(cap.read(frame)) {
//    detectFaces(frame); // Call function to detect faces
//    if( waitKey(30) >= 0)    // pause
//      break;
//  }
//  return 0;
//}


//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//
//#include <iostream>
//#include <stdio.h>
//#include <conio.h>
//
//using namespace std;
//using namespace cv;
//
//vector < vector<Point> > detectAndDisplay(Mat frame);
//
//// to find where your module is, type "locate haarcascade_frontalface_alt.xml" in your cmd line
//string face_cascade_name = "D:/opencv3.0.0-alpha/opencv/sources/data/haarcascades/haarcascade_frontalface_alt.xml";
//CascadeClassifier face_cascade;
//
//int main(void)
//{
//	vector < vector<Point> > faceDetectionVector;
//
//	// Load the cascade
//	if (!face_cascade.load(face_cascade_name))
//	{
//		printf("Couldn't load face cascade module\n");
//		return (-1);
//	};
//
//	// Read the image file
//	//Mat frame = imread("lenna.png");
//	cv::Mat frame;
//	vector<cv::Rect> face_positions;
//	VideoCapture cap("D:/YinYueTai/sbs.mp4");
//	if (!cap.isOpened()) {                                                 // if unable to open video file
//		cout << "error reading video file!" << endl << endl;      // show error message
//		_getch();                    // it may be necessary to change or remove this line if not using Windows
//		return 0;                                                              // and exit program
//	}
//	int frame_count = cap.get(CAP_PROP_FRAME_COUNT);
//
//
//	if (frame_count < 2) {
//		cout << "error: video file must have at least two frames!";
//		_getch();
//		return 0;
//	}
//
//	//if (!frame.empty())
//	//{
//	//	faceDetectionVector = detectAndDisplay(frame); //the magic happens here : detects faces from a pic
//	//	waitKey(0);
//	//}
//	//else
//	//	printf("Are you sure you have the correct filename?\n");
//
//	cap.read(frame);
//	char chCheckForEscKey = 0;
//	int frameCount = 1;
//	//tracing setup
//	while (cap.isOpened() && chCheckForEscKey != 27)
//	{
//		imshow("TestFace", frame);   //´°¿ÚÖÐÏÔÊ¾Í¼Ïñ
//		//face datection
//		//faceDetect(frame, face_positions);
//		faceDetectionVector = detectAndDisplay(frame);
//		if ((cap.get(CAP_PROP_POS_FRAMES) + 1) < cap.get(CAP_PROP_FRAME_COUNT))
//			cap.read(frame);
//		else
//		{
//			cout << "end of video\n";
//			break;
//		}
//		frameCount++;
//		chCheckForEscKey = cv::waitKey(1);
//	}
//	if (chCheckForEscKey != 27)               // if the user did not press esc (i.e. we reached the end of the video)
//		waitKey(0);                         // hold the windows open to allow the "end of video" message to show
//
//
//	return 0;
//}
//
//// detecs faces and displays them in the picture
//vector < vector<Point> > detectAndDisplay(Mat frame)
//{
//	std::vector<Rect> faces;
//	Mat frame_gray;
//	Mat crop;
//	Mat res;
//	Mat gray;
//	vector<Point> faceCoords;
//	vector < vector<Point> > faceVector;
//
//	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
//	equalizeHist(frame_gray, frame_gray);
//
//	// Detect faces
//	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
//
//	int i;
//
//	for (i = 0; i < faces.size(); i++) // Iterate through all detected faces
//	{
//		Point pt1(faces[i].x, faces[i].y); // Display detected faces on main window
//		faceCoords.push_back(pt1);
//		Point pt2((faces[i].x + faces[i].height), (faces[i].y + faces[i].width));
//		faceCoords.push_back(pt2);
//		rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
//
//		faceVector.push_back(faceCoords);
//		faceCoords.clear();
//	}
//
//	imshow("original", frame);
//
//	return faceVector;
//}


