#include <iostream>
#include <io.h>
#include <direct.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video.hpp>





using namespace std;
using namespace cv;


class ImageVideoTrans
{
public:
	//ImageVideoTrans();
	//~ImageVideoTrans();
	//transform frame image sequence to avi video
	int ImageToVideo(char* outDir, char* videoName, char* inputDir, int startFrame, int endFrame, int imgW, int imgH, char* imgExt, double fps, int isColor, int fourcc)
	{
		//判断输入文件夹是否存在
		if (_access(inputDir, 0) == -1)
		{
			std::cout << "the input directory does not exist!" << std::endl;
			return 0;
		}
		//判断输出文件夹是否创建 若没有则创建；若为NULL则默认当前工作目录
		char fullVideoName[255];//输出视频的完整文件名：路径+文件名
		strcpy_s(fullVideoName, "");
		if (outDir == NULL)
		{
			sprintf_s(fullVideoName, "%s", videoName);//把videoName打印成一个字符串保存在fullVideoName 中 
		}
		else
		{
			if (_access(outDir, 0) == -1)
			{
				_mkdir(outDir);
			}
			sprintf_s(fullVideoName, "%s%s", outDir, videoName);//将字符串outDir和videoName连接起来，打印，保存在fullVideoName中
		}
		int frameCount = 0;
		//CvVideoWriter *pWriter = NULL;
		Size size = Size(imgW, imgH);
		VideoWriter pWriter(videoName, fourcc, fps, size, isColor);//CREATE WRITER

		cv::Mat pImg;
		char cur_fn[500];//表示某张图片的路径
		while (startFrame <= endFrame)
		{
			strcpy_s(cur_fn, "");
			char name[] = ".jpg";
			sprintf_s(cur_fn, "%s%s%07d%s", inputDir, imgExt, startFrame, name);//need to change 
			//cout <<"输入文件夹名为："<< cur_fn << endl;
			pImg = imread(cur_fn, isColor);
			if (pImg.empty())
			{
				cout << "can't open an image file" << endl;
				return frameCount;
			}
			pWriter << pImg;
			waitKey(1);
			cout << "Write frame " << startFrame << std::endl;
			startFrame++;
			pImg.release();
			frameCount++;
		}
		//cvReleaseVideoWriter(&pWriter);
		pWriter.release();
		rename(videoName, fullVideoName);//移动文件到指定文件夹
		return  frameCount;
	}

	//将视频转换为图片序列 返回由视频分解得到的图片总帧数 目前OpenCV只支持AVI格式 因此使用之前需要
	//将视频转化问AVI格式

	int  VideoToImage(char* videoName, char* outDir, char* imgExt, int maxFrameCount)
	{
		VideoCapture cap(videoName);
		if (!cap.isOpened())
		{
			cout << "Failed open video file!" << endl;
			return 1;
		}
		if (cap.get(CAP_PROP_FRAME_COUNT) < 2)
		{
			cout << "The video must have at least two frames." << endl;
			return 1;
		}
		//保存图片的文件夹路径一定要有，因为OpenCV不会自动创建文件夹
		if (_access(outDir, 0) == -1)
		{
			recursive_mkdir(outDir);
			std::cout << "the ouput directory does not exist, and the have been created autonomously!" << std::endl;
		}
		char cur_fn[255];//保存当前帧所得图片的文件名
		cv::Mat pImg;
		int frame = 0;
		double rate = cap.get(CAP_PROP_FPS);
		double delay = 1000 / rate;
		cap.read(pImg);
		bool stop = true;
		//while (!pImg.empty() && (frame<maxFrameCount))
		while (cap.isOpened() && stop)
		{
			if (frame < maxFrameCount)
			{
				frame++;
				strcpy_s(cur_fn, "");
				sprintf_s(cur_fn, "%s%d%s", outDir, frame, imgExt);//这里的设置适合形如 123.jpg 124.jpg的图片序列
				imwrite(cur_fn, pImg);
				cap.read(pImg);
			}
			else
				stop = false;
			
		}
		pImg.release();
		cap.release();
		return frame;
	}


	//该函数借鉴了网上资料，自动创建多级目录
	int recursive_mkdir(char *dir)
	{
		//分解路径名E:\\AA\\BB\\CC\\
		 //
		string str = dir;
		int index = 0;
		int i = 0;
		while (1)
		{
			string::size_type pos = str.find("\\", index);
			string str1;
			str1 = str.substr(0, pos);
			if (pos != -1 && i > 0)
			{
				if (_access(str1.c_str(), 0) == -1)
				{
					_mkdir(str1.c_str());
				}
			}
			if (pos == -1)
			{
				break;
			}
			i++;
			index = pos + 1;
		}
		return 0;
	}

	/*void iv_trans(int startFrame, int endFrame, char *imgExt, double fps, int flag){
		if (flag)
			ImageToVideo();
	}*/

};



