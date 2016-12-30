//
//  line_segment.h
//  opencv
//
//  Created by incer on 15/5/7.
//  Copyright (c) 2015年 ce. All rights reserved.
//

#ifndef __opencv__line_segment__
#define __opencv__line_segment__

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "LSWMS.h"
#include <iomanip>

using namespace cv;
using namespace std;



struct LINE
{
    double a;
    double b;
    double c;
    cv::Point start;
    cv::Point end;
};

struct SpineLines
{
    cv::Point2f point1;//one end-point
    cv::Point2f point2;//the other end-point
    double length;//the length of the line segment
    double theta;
    double rho;
};

struct Line
{
    cv::Point _p1;
    cv::Point _p2;
    cv::Point _center;//连线中点
    
    Line(cv::Point p1, cv::Point p2)
    {
        _p1 = p1;
        _p2 = p2;
        _center = cv::Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }
};


class ls
{
private:
    int krows = 480;// 640;
    float la =30,lb = 70,lc = 80;
    double line_size(cv::Point &p1,cv::Point &p2);
    float line_jiao(cv::Point &p1,cv::Point &p2);
    int ca(LSEG &line,int k);
    void run(vector<LSEG> &lines1,int left,int right,int k,LSEG &baseline);
    void QuickSort(vector<LSEG> &lines1,int Count,int k,LSEG &baseline);
    double power(cv::Mat &src,cv::Mat &angle,cv::Point &a,cv::Point &b);
    cv::Point prpoint(cv::Point &center,int d,Point2f &v);
    int point_line(cv::Point &p1,cv::Point &p2,cv::Point &tp);
    float pr_area(cv::Mat &angle,vector<cv::Point>& pr,float& height);
    void DecideOberlap(vector<cv::Point> &pr1,vector<cv::Point> &pr2,cv::Size imgsize,int& k1,int& k2);
    int warf(cv::Mat &src,cv::Mat &src1,cv::Mat &angle,LSEG &line1,LSEG &line2,vector<cv::Point> &rc ,int k,float &tg1,float &tg2);
    void warp_change(vector<cv::Point>& in_prs,cv::Mat& src1,cv::Mat& src,cv::Mat& out,vector<cv::Point2f> &corner);
    bool pr_detect(cv::Mat &src,cv::Mat &src1,cv::Mat &angle,vector<LSEG> &oolines,vector<vector<cv::Point> > &out_result,vector<vector<cv::Point> > &rest,int k,
                   LSEG &baseline,vector<cv::Mat> &book_cordinates);
    void findline(Mat &flood,vector<LSEG> &lines,int k);
    void parallelines(vector<LSEG> &lines,vector<LSEG> &outlines);
    int pers_point_inside(cv::Point &p1,cv::Point &p2,cv::Point &p_pers);
    int is_onLine(Point2f p,LSEG lines);
    void point_class(Point2f p1,Point2f p2,vector<Point2f> corners_y,vector<Point2f> corners_x,
                     vector<Point2f> corners_l,vector<Point2f> corners_r);
    bool isInRectangle(cv::Point& pt,vector<cv::Point>& pr);
    float area(cv::Point &pt,cv::Point &p1,cv::Point &p2);
    void select_points(LSEG pts);
    void get_lines(cv::Mat &blur_out,vector<LSEG> &corners,vector<LSEG> &linesy, vector<LSEG> &linesr, vector<LSEG> &linesl,vector<LSEG> &linesx);
    int is_parallel(LSEG l1,LSEG l2);
    vector<vector<LSEG>> intersect_trunk(vector<LSEG> &oolines,int count);
    vector<vector<LSEG>> lines_detection(Mat &a,double thresh);
    void get_angle(Mat &blur_out,Mat &angle,Mat &magnitude);
    void getlines2(vector<LINE> & ls,vector<LSEG> les,vector<int> &k);
    void exchange(vector<LSEG> &l,Mat & src);
    double max_array(vector<double> a);
    double min_array(vector<double> a);
    void horizon_classify(vector<LSEG> &lines);
    vector<LSEG> line_preprocess(vector<LSEG> &lines,int k);
    void line_conn(vector<LSEG> &oolines);
    LINE makeline(cv::Point p1,cv::Point p2);
    int zero_point_x(LSEG & line,LSEG &baseline,int k);
    bool lineintersect(LINE l1,LINE l2,cv::Point &p);
    void select_right_spine(cv::Mat & angle,vector<LSEG> &res,vector<LSEG>res_out,int k);
    cv::Point get_gravity_center(LSEG &pr);
    cv::Point2f get_up_left(LSEG &pr,float os);
    void markcounter();
    float line_jiao1(cv::Point &p1,cv::Point &p2);
    float ca1(LSEG &line, int k);
    double power1(vector<cv::Point> &in_prs,cv::Mat &src,cv::Mat &angle,double tg);
    void gammaCorrection(cv::Mat &src, cv::Mat &dst, float gamma);
    void scanNoteFilter(cv::Mat &src,cv::Mat &dst,int flagProcess);
    void scan1(cv::Mat &img, float out_point[8]);
    //    bool cmp_x(const Line &p1, const Line &p2) ;
    
    void DecideOberlap1(vector<cv::Point> &pr1,vector<cv::Point> &pr2,cv::Size imgsize,int& k1,int & k2);
    void selectThresh(cv::Mat &Gx,cv::Mat &Gy,double percent,double thresh_ratio,double &thresh_high,double &thresh_low);
    void smoothGradient(cv::Mat &binaryImage,double sigma,cv::Mat &Gx,cv::Mat &Gy);
    void adaptiveHistEqual(cv::Mat &src,cv::Mat &dst,double clipLimit);
    vector<double> linspace(double d1,double d2,int n);
    void multiplyWithSameSize(cv::Mat &mat1,cv::Mat &mat2,cv::Mat &dst);
    //    void houghT(cv::Mat &src,cv::Mat &H,int theta_in,vector<double> &theta,vector<double> &rho);
    cv::Mat repmat(cv::Mat &mat,int m,int n);
    void sub_vector(vector<double> &vec, int idx1,int idx2,vector<double> &out );
    void houghPeaks(cv::Mat &H,int numpeaks,double thresh,const int *nhood_size,vector<double> &r,vector<double> &c);
    void mat2Cvector(cv::Mat &mat,vector<double> &vec);
    void mat2Rvector(cv::Mat &mat,vector<double> &vec);
    vector<double> getVector(const Mat &_t1f);
    int sub2ind(cv::Mat &A,double pp,double qq);
    void matrixMul(cv::Mat &A,cv::Mat &B,cv::Mat &C);
    void findNonZeroIdx(cv::Mat &src,vector<double> &x,vector<double> &y,vector<double> &val);
    //    void houghPixels(cv::Mat &f,vector<double> &theta,vector<double> &rho,double rbin,double cbin,vector<double> &r,vector<double> &c);
    //    vector<SpineLines>  houghlines_rc(cv::Mat &gray_img,vector<double> &theta,vector<double> &rho,vector<double> &rr,vector<double> &cc,double fillgap,double minlength);
    void reSortHoughPixels(vector<double> &r,vector<double> &c,vector<double> &rnew,vector<double> &cnew);
    //    bool cmp_py(const Point2f &p1, const Point2f &p2);
    //    bool cmp_px(const Point2f &p1, const Point2f &p2);
    void sortP(vector<Point2f> &pts,int sort_order);
    void myHoughMex(cv::Mat &edge,cv::Mat &H, vector<double> theta,vector<double> rho);
    void myHoughPixels(vector<double> &nonzero_x,vector<double> &nonzero_y,vector<double> &theta,vector<double> &rho,int rbin,int cbin,vector<double> &r,vector<double> &c);
    vector<LSEG> myHoughLines(cv::Mat &gray_img,vector<double> &theta,vector<double> &rho,vector<double> &rr,vector<double> &cc,
                              vector<double> &nonzero_x,vector<double> &nonzero_y, double fillgap,double minlength);
    void getThetaRho(cv::Mat &edges,vector<double> &theta,vector<double> &rho,double theta_in,double dtheta,double drho);
    void myMeshgrid(int l1,int l2,int r1,int r2,cv::Mat &left,cv::Mat &right);
    int WriteData(string fileName, cv::Mat& matData);
    int warf1(cv::Mat &src,cv::Mat &src1,cv::Mat &angle,LSEG &line1,LSEG &line2,vector<cv::Point> &rc  ,int k);
    bool pr_detect1(cv::Mat &src,cv::Mat &src1,cv::Mat &angle,vector<LSEG> &oolines,vector<vector<cv::Point> > &out_result,int k);
    void QuickSort1(vector<LSEG> &lines1,int Count,int k);
    void run1(vector<LSEG> &lines1,int left,int right,int k);
    
    
    
public:
    
    void testStart(cv::Mat &edge);
    void bookSegmentStart(cv::Mat &src1,vector<cv::Mat> &result,vector<cv::Mat> &corner);
    void bookShelfSegment(cv::Mat &src, vector<cv::Mat> &results,vector<cv::Mat> &corner);
    void perspective(cv::Mat &src,float in_point[8],cv::Mat &dst);
    //    void book_segment(Mat &src,vector<Mat> &out_result);
    //void bookSegmentStart2(Mat &src1,vector<Mat> &out_result);
    //void QuickSort(vector<LSEG> &lines1,int Count,int k);
    void black(cv::Mat &src1,vector<cv::Mat> &coordinates);
    vector<vector<LSEG>> lines_same,lines_same_y,lines_same_x,lines_same_r,lines_same_l;
    void nonbook_extract(cv::Mat &image,cv::Mat &src);
    void one_scanner(cv::Mat &img, float out_point[8]);
    void adaptiveAddFilter(cv::Mat &src,cv::Mat &dst);
    void adaptiveRGBFilter(cv::Mat &src,cv::Mat &dst);
    void gammaSingleFilter(cv::Mat &src,cv::Mat &dst);
    void matrixAddFilter(cv::Mat &src,cv::Mat &dst);
    void gamma2AddFilter(cv::Mat &src,cv::Mat &dst);
    
    
    
};

class PerspectiveTransform
{
public:
    
    float a11,a12,a13,a21,a22,a23,a31,a32,a33;
    PerspectiveTransform times(PerspectiveTransform t);
    PerspectiveTransform result(PerspectiveTransform f);
    PerspectiveTransform quadrilateralToQuadrilateral(float x0, float y0, float x1, float y1,
                                                                            float x2, float y2, float x3, float y3, float x0p,
                                                      float y0p, float x1p, float y1p, float x2p, float y2p,
                                                                            float x3p, float y3p);
    PerspectiveTransform squareToQuadrilateral(float x0, float y0, float x1, float y1, float x2,
                                                                     float y2, float x3, float y3);
    PerspectiveTransform quadrilateralToSquare(float x0, float y0, float x1, float y1, float x2,
                                                                     float y2, float x3, float y3);
    PerspectiveTransform buildAdjoint();
    
    void transformPoints(vector<float> &points);
    PerspectiveTransform PerspectiveTransformM(float inA11, float inA21,float inA31, float inA12,
                                                                     float inA22, float inA32,float inA13, float inA23,float inA33);
    
    
        
    
    
    
};




//帧处理类
class frameProcessor{
public:
    virtual void process(cv::Mat &input,cv::Mat &output) = 0;
    
    
};



class VideoProcessor
{
private:
    cv::VideoCapture capture;//video cpature object
    void(*process)(cv::Mat &,cv::Mat &);//callback function of every frame calls
    bool call_it;//确定是否调用回调函数的bool变量
    string windowNameInput;
    string windowNameOutput;
    int delay;
    long fnumber;//已处理的帧数
    long frame_to_stop;//在该帧数停止
    bool stop;//是否停止处理
    vector<string> images;
    vector<string>::const_iterator imgIt;
    //opencv的写视频对象
    VideoWriter writer;
    //输出文件名称
    string outputfile;
    //输出文件当前索引
    int current_index;
    //输出图像名称的位数
    int digits;
    //输出图像的扩展名
    string extension;
    //制定跟踪的书脊矩形框
    cv::Rect pr;
public:
    //initialize
    VideoProcessor():call_it(true),delay(0),fnumber(0),frame_to_stop(-1),stop(false) {}
    //    void canny(cv::Mat &img,cv::Mat &out);
    //    设置frameProcessor实例
    frameProcessor *frameprocess;
    void set_frame_processor(frameProcessor *fptr)
    {
        //使回调函数无效
        process = 0;
        //重新设置实例
        
        frameprocess = fptr;
        callProcess();
        
        
    }
    
    //设置回调函数
    void set_frame_processor(void(*frame_processing_callback) (cv::Mat &,cv::Mat &))
    {
        frameprocess = 0;
        process = frame_processing_callback;
        callProcess();
    }
    //设置视频文件的名称
    bool set_input(string filename)
    {
        fnumber = 0;
        capture.release();
        images.clear();
        return capture.open(filename);
    }
    //设置输入的图像向量
    bool set_input(vector<string> &imgs)
    {
        fnumber = 0;
        //释放之前打开过的资源
        capture.release();
        //输入将是该图像的向量
        images = imgs;
        imgIt = images.begin();
        return  true;
    }
    
    //设置输出为视频文件，默认使用与输入视频相同的参数
    bool set_output(const string &filename,int codec = 0,double fps = 0.0,bool is_color = true)
    {
        outputfile = filename;
        extension.clear();
        if(fps == 0.0)fps = get_frame_rate();
        char c[4];
        if(0 == codec )codec = get_codec(c);
        //        Size size = capture.get(cv_cap_pro_fra)
        return writer.open(outputfile, codec, fps, get_frame_size(),is_color);
    }
    cv::Size get_frame_size()
    {
        int h = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
        int w = capture.get(CV_CAP_PROP_FRAME_WIDTH);
        return cv::Size(w,h);
        
    }
    long get_frame_rate()
    {
        return capture.get(CV_CAP_PROP_FPS);
    }
    
    //获取输入视频的编码方式
    int get_codec(char codec[4])
    {
        if(images.size() != 0)return -1;
        union{
            //4-char的数据编码结果
            int value;
            char code[4];
        }returned;
        returned.value = (int)capture.get(CV_CAP_PROP_FOURCC);//得到编码
        codec[0] = returned.code[0];
        codec[1] = returned.code[1];
        codec[2] = returned.code[2];
        codec[3] = returned.code[3];
        return returned.value;
    }
    //保存输出帧，可能是：视频或图像
    void write_next_frame(cv::Mat &frame)
    {
        if(extension.length())//输出到图像文件
        {
            stringstream ss;
            ss<<outputfile<<std::setfill('0')
            <<std::setw(digits)
            <<current_index++<<extension;
            imwrite(ss.str(), frame);
        }
        else//输出到视频文件
        {
            //            cv::Mat tmp();
            if(!frame.empty())
            {
                if(frame.channels() != 3 || frame.channels() != 4)
                    cvtColor(frame, frame, CV_GRAY2BGR);
                writer.write(frame);
            }
        }
    }
    
    //设置输出为独立的图像文件，扩展名必须是jpg，bmp
    bool set_output(const string &filename,const string &ext,int number_of_digits = 3,int start_index = 0)
    {
        if(number_of_digits < 0)
            return false;
        outputfile = filename;
        extension = ext;
        digits = number_of_digits;
        current_index = start_index;
        return true;
    }
    //捕捉视频设备是否已经打开
    bool is_opened()
    {
        return capture.isOpened() || !images.empty();
    }
    //create input window
    void display_input(string window)
    {
        windowNameInput = window;
        namedWindow(windowNameInput);
    }
    //create input window
    void display_output(string window)
    {
        windowNameOutput = window;
        namedWindow(windowNameOutput);
    }
    //不在显示处理后的帧
    void dont_display()
    {
        destroyWindow(windowNameInput);
        destroyWindow(windowNameOutput);
        windowNameInput.clear();
        windowNameOutput.clear();
    }
    //得到下一帧，可以是：视频文件，摄像头，图像数组
    bool read_next_frame(cv::Mat &frame)
    {
        if(!images.size())
            return capture.read(frame);
        else
        {
            if(imgIt != images.end())
            {
                frame = imread(*imgIt);
                //                cout<<*imgIt<<endl;
                //                display_output("first frame");
                //                imshow(windowNameOutput, frame);
                imgIt ++;
                return frame.data != 0;
            }
            else
                return false;
        }
    }
    
    
    //获取并处理序列帧
    void run()
    {
        //current frame
        cv::Mat frame;
        //ouput frame
        cv::Mat output;
        if (! is_opened())
        {
            return;
        }
        stop = false;
        while (! is_stopped())
        {
            //读取下一帧
            if(! read_next_frame(frame))
                break;
            //            display_input("display a frame");
            if(windowNameInput.length() != 0)
                imshow(windowNameInput, frame);
            //调用处理函数
            if(call_it)
            {
                if(process)
                {
                    //处理当前帧
                    process(frame,output);
                }
                else if (frameprocess)
                    frameprocess->process(frame, output);
                fnumber ++;
                
            }
            else
            {
                output = frame;
            }
            //输出图像序列
            if(outputfile.length())
                write_next_frame(output);
            //显示输出帧
            //            display_output("display out frame");
            if( windowNameOutput.length() != 0)
                imshow(windowNameOutput, output);
            //            imshow("display out frame", output);
            //引入延迟
//            if(delay >= 0 && waitKey(delay) >= 0 )
//                stopIt();//停止运行
            if(frame_to_stop >= 0 && get_frame_number() == frame_to_stop)
                stopIt();
        }
    }
    //停止运行
    void stopIt()
    {
        stop = true;
    }
    //是否已经停止
    bool is_stopped()
    {
        return stop;
    }
    
    //set the delay,0 means waiting for the user to press key,
    void set_delay(int d)
    {
        delay = d;
    }
    //需要调用回调函数
    void callProcess()
    {
        call_it = true;
    }
    //不需要调用回调函数
    void not_callProcess()
    {
        call_it = false;
    }
    //在指定帧停止
    void stop_frame_num(long frame)
    {
        frame_to_stop = frame;
    }
    //返回一帧的帧数
    long get_frame_number()
    {
        return capture.get(CV_CAP_PROP_POS_FRAMES);
    }
    
    
    
};


class FeatureTracker:public frameProcessor{
    cv::Mat gray_curr;
    cv::Mat gray_prev;//之前的灰度图像
    //两幅图像间跟踪的特征点 0->1
    vector<cv::Point2f> points[2];
    //跟踪的初始点位置
    vector<cv::Point2f> initial;
    //特征点
    vector<cv::Point2f> features;
    //需要跟踪的最大特征数
    int max_count;
    //评估跟踪的质量
    double q_lel;
    //两点之间的最小距离
    double min_dist;
    //跟踪状态
    vector<uchar> status;
    //跟踪过程中的error
    vector<float> err;
public:
    FeatureTracker():max_count(500),q_lel(0.01),min_dist(10.) {}
    //该方法将在每一帧被调用，首先，在需要时检测特征点，接着，跟踪这些点，将无法跟踪或是不再希望跟踪的点剔除掉，最后当前帧以及它的点在下一次迭代中成为之前的帧以及之前的点
    void process(cv::Mat &frame,cv::Mat & output)
    {
        cvtColor(frame, gray_curr, CV_BGR2GRAY);
        frame.copyTo(output);
        //1.如果需要添加新的特征点
        if(add_new_points())
        {
            //检测新的特征点
            detect_features();
            //添加到当前跟踪的特征中
            points[0].insert(points[0].end(), features.begin(), features.end());
            initial.insert(initial.end(), features.begin(), features.end());
            
        }
        //对于序列中的第一幅图像
        if(gray_prev.empty())
            gray_curr.copyTo(gray_prev);
        //2.跟踪特征点
        calcOpticalFlowPyrLK(gray_prev, gray_curr, points[0], points[1], status, err);
        //2.遍历所有跟踪的点进行筛选
        int k = 0;
        for (int i = 0; i < points[1].size(); i ++)
        {
            //是否保留
            if(accept_tracked_ponints(i))
            {
                initial[k] = initial[i];
                points[1][k++] = points[1][i];
            }
        }
        //剔除跟踪不成功的点
        points[1].resize(k);
        initial.resize(k);
        //3.处理接受的跟踪点
        handle_tracked_points(frame, output);
        //4.当前帧的点和图像变为前一帧的点和图像
        swap(points[1], points[0]);
        swap(gray_prev,gray_curr);
        
    }
    
    
    //是否添加新的特征点
    bool add_new_points()
    {
        //点的数量太少
        return points[0].size() <= 10;
    }
    //检测特征点
    void detect_features()
    {
        goodFeaturesToTrack(gray_curr, features, max_count, q_lel, min_dist);
    }
    //决定哪些点应该跟踪,剔除不在移动的点，以及calcOpticalFlowPyrLK无法跟踪的点
    bool accept_tracked_ponints(int i)
    {
        return status[i] &&
        //表示移动了
        ((abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y)) > 2);
        
    }
    //处理当前跟踪的点
    void handle_tracked_points(cv::Mat &frame,cv::Mat &out)
    {
        //遍历所有跟踪点
        for (int i = 0; i < points[1].size(); i ++)
        {
            line(out, initial[i], points[1][i], Scalar(0,255,0));
            circle(out, points[1][i], 3, Scalar(255,255,255),-1);
        }
    }
    
    
};


class BGFGSegmentor:public frameProcessor{
    cv::Mat gray;//当前灰度图
    cv::Mat background;//累积的背景
    cv::Mat back_image;//背景图像
    cv:: Mat fore_image;//前景图像
    //背景累加中的学习率
    double lr;
    int threshold;//前景提取的阈值
    //制定跟踪的书脊矩形框
    cv::Rect pr;
public:
    BGFGSegmentor():threshold(10),lr(0.01) {}
    //处理方法
    void process(cv::Mat &frame,cv::Mat &out)
    {
        cvtColor(frame, gray, CV_BGR2GRAY);
        //初始化背景为第一帧
        if(background.empty())
            gray.convertTo(background,CV_32F);
        background.convertTo(back_image, CV_8U);
        //计算差值
        //        Ptr<BackgroundSubtractorMOG2> mog = createBackgroundSubtractorMOG2(10,10,true);
        absdiff(back_image, gray, fore_image);
        //        mog->apply(frame,fore_image,0.01);
        //        out = fore_image.clone();
        cv::threshold(fore_image, out, threshold, 255, CV_THRESH_BINARY_INV);
        //对背景累加
        accumulateWeighted(gray, background, lr,out);
        vector<vector<cv::Point>> contour;
        findContours(out, contour, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        for (int i = 0; i < contour.size(); i ++)
        {
            cv::Rect rect = boundingRect(contour[i]);
            double area = contourArea(contour[i]);
            if (area > 3000 && area < 7000) {
                rectangle(frame, rect, Scalar(0,255,0));
                //                imshow("rect", frame);
                
            }
            
        }
        //        rectangle(frame,pr,Scalar(0,255,0));
        out = frame.clone();
        
        
    }
    void set_thresh(int thresh)
    {
        threshold = thresh;
    }
    void  set_pr(vector<cv::Point> corner)
    {
        pr = boundingRect(corner);
    }
};



//typedef MAT_TYPE double

class HarrisDetector
{
private:
    Mat cornerStrength;//表示角点强度的32位浮点图像
    Mat cornerThresh;//表示阈值化后角度的32位浮点图像
    Mat localMax;//局部极大值图像（内部）
    int neighbourhood;//导数平滑的相邻像素的尺寸
    int aperture;//梯度计算的孔径大小
    double k;//Harris参数
    double maxStrength;//阈值及孙的最大强度
    double threshold;//计算得到的阈值（内部）
    int nonMaxSize;//非极大值抑制的相邻像素的尺寸
    Mat kernel;//非极大抑制的核
    void setLocalMaxWindowSize(int _size){nonMaxSize = _size;}
    
public:
    HarrisDetector():neighbourhood(3),aperture(3),k(0.01),maxStrength(0.0),threshold(0.0001),nonMaxSize(3)
    {
         setLocalMaxWindowSize(nonMaxSize);//创建非极大抑制的核
    }
    void detect(Mat & image)
    {
        cornerHarris(image, cornerStrength, neighbourhood, aperture, k);
        double minStrength;
        //内部阈值计算
        minMaxLoc(cornerStrength, &minStrength, &maxStrength);
        Mat dilated;
        //局部极大值检测
        dilate(cornerStrength, dilated, Mat());
        compare(cornerStrength, dilated, localMax, CMP_EQ);
        
    }
    Mat getCornerMap(double qualityLevel)
    {
        Mat cornerMap;
        threshold = qualityLevel*maxStrength;
        cv::threshold(cornerStrength, cornerThresh, threshold, 255, THRESH_BINARY);
        cornerThresh.convertTo(cornerMap, CV_8U);
        //非极大值抑制
        bitwise_and(cornerMap, localMax, cornerMap);
        return cornerMap;
    }
    void getCorners2(vector<cv::Point> &corners,Mat &cornerMap)
    {
        for (int i = 0; i < cornerMap.rows; i ++)
        {
            const uchar *rowsPtr = cornerMap.ptr<uchar>(i);
            for (int j = 0; j < cornerMap.cols; j ++)
            {
                if (rowsPtr[j])
                {
                    corners.push_back(cv::Point(i,j));
                }
            }
        }
    }
    void getCorners(vector<cv::Point> &corners,double qualityLevel)
    {
        Mat cornerMap = getCornerMap(qualityLevel);
        getCorners2(corners,cornerMap);
    }
    void drawOnImage(Mat &src,vector<cv::Point> &points,Scalar color = Scalar(0,255,0),int radius = 3)
    {
        vector<cv::Point>::const_iterator it = points.begin();
        while (it != points.end())
        {
            circle(src, *it, radius, color);
            ++ it;
            
        }
    
    }
    
   
};



class Histogrom1D
{
private:
    int histSize[1];//项的数量
    float hranges[2];//像素的最大与最小值
    const float *ranges[1];
    int channels[1];//仅用到一个通道
public:
    Histogrom1D(){
        //准备1D直方图的参数
        histSize[0] = 256;
        hranges[0] = 0.0;
        hranges[1] = 255.0;
        ranges[0] = hranges;
        channels[0] = 0;//默认情况，我们考察0号通道
        
    }
    //计算一维直方图
    MatND getHistogram(const cv::Mat &image){
        cv::MatND hist;
        calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
        return hist;
    }
    //计算一维直方图，并返回一幅图像
    cv::Mat getHistogramImage(const cv::Mat &image)
    {
        MatND hist = getHistogram(image);
        double maxVal = 0,minVal = 0;
        //获取最大值和最小值
        minMaxLoc(hist, &minVal, &maxVal,0,0);
        cout<<maxVal<<endl<<minVal<<endl;
        cv::Mat histImg(histSize[0],histSize[0],CV_8U,Scalar(255));
        //设置最高点为nbins的90%
        int hpt = static_cast<int>(0.9 * histSize[0]);
        //每个条目绘制一条垂直线
        for(int h = 0;h < histSize[0];h ++)
        {
            float binVal = hist.at<float>(h);
            //cout<<binVal<<endl;
            int intensity = static_cast<int>(binVal*hpt/maxVal);
            line(histImg, cv::Point(h,histSize[0]), cv::Point(h,histSize[0]-intensity), Scalar::all(0));
        
        }
        return histImg;
    }
    cv::Mat applyLookUp(const cv::Mat &image,cv::Mat &lookup)
    {
        cv::Mat result;
        cv::LUT(image, lookup, result);
        return result;
    
    }
    cv::Mat stretch(const cv::Mat &image,int minValue)
    {
        //首先计算直方图
        MatND hist = getHistogram(image);
        //寻找直方图的左端
        int imin = 0;
        for (; imin < histSize[0]; imin++)
        {
            //cout<<"imin"<<hist.at<float>(imin)<<endl;
            if (hist.at<float>(imin) > minValue)break;
            else
            {
                hist.at<float>(imin) = minValue;
            }
        }
        //寻找直方图的右端
        int imax = histSize[0] - 1;
        for (; imax >= 0; imax --)
        {
            if (hist.at<float>(imax) > minValue)
            {
                break;
            }
            else
                hist.at<float>(imax) = 255;
        }
        //创建查找表
        int dim(256);
        cv::Mat lookup(1,&dim,CV_8U);
        //填充查找表
        for (int i = 0; i < 256; i ++)
        {
            //确保数值位于imin和imax之间
            if (i < imin)lookup.at<uchar>(i) = 0;
            else if (i > imax)lookup.at<uchar>(i) = 255;
            //线性映射
            else
               lookup.at<uchar>(i) = static_cast<uchar>(255.0 * (i - imin)/(imax - imin) + 0.5);
        }
        //应用查找表
        cv::Mat res;
        res = applyLookUp(image, lookup);
        
        return res;
    
    
    }
    
    
    void getKValue(Mat &src, int &mean,int &kMax)
    {
        Mat gray;
        cvtColor(src, gray, CV_BGR2GRAY);
        MatND hist = getHistogram(gray);
        double maxVal = 0,minVal = 0;
        //        int k_min,k_max;
        //获取最大值和最小值
        minMaxLoc(hist, &minVal, &maxVal,0,0);
        int hpt = static_cast<int>(0.9 * histSize[0]);
        long sum = 0;
        //每个条目绘制一条垂直线
        for(int h = 0;h < histSize[0];h ++)
        {
            float binVal = hist.at<float>(h);
            //cout<<binVal<<endl;
            int intensity = static_cast<int>(binVal*hpt/maxVal);
            sum = sum + intensity;
            if (hist.at<float>(h) == maxVal)
            {
                kMax = h;
            }
        }
        mean = (int)(sum / 256);
        
    }
    void getMeanVar(Mat &src,double &mean,double &var)
    {
        int width = src.cols;
        int height = src.rows;
        
        uchar *ptr = src.data;
        int Iij;
        
        double Imax = 0, Imin = 255, Iave = 0, Idelta = 0;
        
        for(int i=0;i<height;i++)
        {
            for(int j=0;j<width;j++)
            {
                Iij	= (int) ptr[i*width+j];
                
                if(Iij > Imax)
                    Imax = Iij;
                
                if(Iij < Imin)
                    Imin = Iij;
                
                Iave = Iave + Iij;
            }
        }
        
        Iave = Iave/(width*height);
        mean = Iave;
        
        for(int i=0;i<height;i++)
        {
            for(int j=0;j<width;j++)
            {
                Iij	= (int) ptr[i*width+j];
                
                Idelta	= Idelta + (Iij-Iave)*(Iij-Iave);
            }
        }
        
        Idelta = Idelta/(width*height);
        var = sqrt(Idelta);
    }
    
    void changeRGB(Mat &src)
    {
        //        Mat image;
        //        detailEnhance(src, src);
        Mat gray;
        cvtColor(src, gray, CV_BGR2GRAY);
        MatND hist = getHistogram(gray);
        double maxVal = 0,minVal = 0;
        //获取最大值和最小值
        int hpt = static_cast<int>(0.9 * histSize[0]);
        minMaxLoc(hist, &minVal, &maxVal,0,0);
        cout<<minVal<<","<<maxVal<<endl;
        vector<int> kChange;
        //        int th = me / maxVal;
        //        int nonzero = countNonZero(gray);
        //        double ratio = nonzero / (double)(gray.cols * gray.rows);
        //        cout<<ratio<<endl;
        for (int i = 0; i < histSize[0]; i ++)
        {
            float binVal = hist.at<float>(i);
            //cout<<binVal<<endl;
            int intensity = static_cast<int>(binVal*hpt/maxVal);
            if (abs(intensity - 255) < 50)
            {
                intensity = 255;
                hist.at<float>(i) = intensity * maxVal/hpt;
            }
            //            if (abs(intensity - 0) > 70) {
            //                intensity = 0;
            //                hist.at<float>(i) = intensity * maxVal/hpt;
            //            }
            
            
        }
        minMaxLoc(hist,&minVal, &maxVal,0,0);
        cout<<minVal<<","<<maxVal<<endl;
        for (int num = 0; num < histSize[0]; num ++)
        {
            if (hist.at<float>(num) >= 0.5*maxVal)//( hist.at<float>(i) >= ratio*maxVal && hist.at<float>(i) <= 1.0*maxVal)//3.63
            {
                kChange.push_back(num);
            }
        }
        int nl = src.rows;
        int nc = src.cols;
        //        if (src.isContinuous()) {
        //            nc = nc * nl;
        //            nl = 1;
        //        }
        for (int j = 0; j < nl; j ++)
        {
            //            uchar *data = src.ptr<uchar>(j);
            for (int l = 0; l < nc; l ++)
            {
                int b = 0.11 * src.at<Vec3b>(j, l)[0];
                int g = 0.59 * src.at<Vec3b>(j, l)[1];
                int r = 0.30 * src.at<Vec3b>(j, l)[2];
                int greyVal = b + g + r;
                for (int ii = 0; ii < kChange.size(); ii ++)
                {
                    if (abs(greyVal - kChange[ii]) <= 20)//
                        //                    if (greyVal == kChange[ii])
                    {
                        src.at<Vec3b>(j, l)[0] = 255;
                        src.at<Vec3b>(j, l)[1] = 255;
                        src.at<Vec3b>(j, l)[2] = 255;
                    }
                }
                
            }
        }
    }

    
    Mat& ScanImageAndReduceC(Mat& I, const uchar* const table)
    {
        // accept only char type matrices
        CV_Assert(I.depth() != sizeof(uchar));
        int channels = I.channels();
        int nRows = I.rows ;
        int nCols = I.cols* channels;
        if (I.isContinuous())
        {
            nCols *= nRows;
            nRows = 1;
        }
        int i,j;
        uchar* p;
        for( i = 0; i < nRows; ++i)
        {
            p = I.ptr<uchar>(i);
            for ( j = 0; j < nCols; ++j)
            {
                p[j] = table[p[j]];
            }
        }
        return I;
    }
    
    Mat& ScanImageAndReduceIterator(Mat& I, const uchar* const table)
    {
        // accept only char type matrices
        CV_Assert(I.depth() != sizeof(uchar));
        const int channels = I.channels();
        switch(channels)
        {
            case 1:
            {
                MatIterator_<uchar> it, end;
                for( it = I.begin<uchar>(), end = I.end<uchar>(); it != end; ++it)
                    *it = table[*it];
                break;
            }
            case 3:
            {
                MatIterator_<Vec3b> it, end;
                for( it = I.begin<Vec3b>(), end = I.end<Vec3b>(); it != end; ++it)
                {
                    (*it)[0] = table[(*it)[0]];
                    (*it)[1] = table[(*it)[1]];
                    (*it)[2] = table[(*it)[2]];
                }
            }
        }
        return I;
    }
    
    
    //方法零：.ptr和[]操作符
    //Mat最直接的访问方法是通过.ptr<>函数得到一行的指针，并用[]操作符访问某一列的像素值。
    void colorReduce0(cv::Mat &image, int div) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                data[i]= data[i]/div*div + div/2;
            }
        }
    }
    
    
    void grayIncrease(cv::Mat &image,int grayVal) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                data[i]= data[i] - grayVal;
            }
        }
    }
    
    //方法一：.ptr和指针操作
    //除了[]操作符，我们可以移动指针*++的组合方法访问某一行中所有像素的值。
    // using .ptr and * ++
    void colorReduce1(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                uchar * tmp;
                *tmp ++ = *data ++;
                *tmp++ = *data/div*div + div/2;
            } // end of row
        }
    }
    
    //方法二：.ptr、指针操作和取模运算
    //方法二和方法一的访问方式相同，不同的是color reduce用模运算代替整数除法
    // using .ptr and * ++ and modulo
    void colorReduce2(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                int v= *data;
                *data++= v - v%div + div/2;
            } // end of row
        }
    }
    
    //方法三：.ptr、指针运算和位运算
    //由于进行量化的单元div通常是2的整次方，因此所有的乘法和除法都可以用位运算表示。
    
    // using .ptr and * ++ and bitwise
    void colorReduce3(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                uchar * tmp;
                *tmp ++ = *data ++;
                *tmp++= *data&mask + div/2;
            } // end of row
        }
    }
    
    //方法四：指针运算
    ///方法四和方法三量化处理的方法相同，不同的是用指针运算代替*++操作。
    
    // direct pointer arithmetic
    void colorReduce4(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        int step= (int)image.step; // effective width
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        // get the pointer to the image buffer
        uchar *data= image.data;
        for (int j=0; j<nr; j++) {
            for (int i=0; i<nc; i++) {
                *(data+i)= *data&mask + div/2;
            } // end of row
            data+= step;  // next line
        }
    }
    
    //方法五：.ptr、*++、位运算以及image.cols * image.channels()
    //这种方法就是没有计算nc，基本是个充数的方法。
    
    // using .ptr and * ++ and bitwise with image.cols * image.channels()
    void colorReduce5(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<image.cols * image.channels(); i++) {
                uchar * tmp;
                *tmp ++ = *data ++;
                *tmp++= *data&mask + div/2;
            } // end of row
        }
    }
    
    
    //方法六：连续图像
    //Mat提供了isContinuous()函数用来查看Mat在内存中是不是连续存储，如果是则图片被存储在一行中。
    
    // using .ptr and * ++ and bitwise (continuous)
    void colorReduce6(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        if (image.isContinuous())  {
            // then no padded pixels
            nc= nc*nr;
            nr= 1;  // it is now a 1D array
        }
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                uchar * tmp;
                *tmp ++ = *data ++;
                *tmp++= *data&mask + div/2;
            } // end of row
        }
    }
    
    //方法七：continuous+channels
    //与方法六基本相同，也是充数的。
    
    // using .ptr and * ++ and bitwise (continuous+channels)
    void colorReduce7(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols ; // number of columns
        if (image.isContinuous())  {
            // then no padded pixels
            nc= nc*nr;
            nr= 1;  // it is now a 1D array
        }
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                uchar * tmp;
                *tmp ++ = *data ++;
                *tmp++= *data&mask + div/2;
                *tmp++= *data&mask + div/2;
                *tmp++= *data&mask + div/2;
            } // end of row
        }
    }
    
    //方法八：Mat _iterator
    //真正有区别的方法来啦，用Mat提供的迭代器代替前面的[]操作符或指针，血统纯正的官方方法~
    
    // using Mat_ iterator
    void colorReduce8(cv::Mat &image, int div=64) {
        // get iterators
        cv::Mat_<cv::Vec3b>::iterator it= image.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::iterator itend= image.end<cv::Vec3b>();
        for ( ; it!= itend; ++it) {
            (*it)[0]= (*it)[0]/div*div + div/2;
            (*it)[1]= (*it)[1]/div*div + div/2;
            (*it)[2]= (*it)[2]/div*div + div/2;
        }
    }
    
    
    //方法九：Mat_ iterator 和位运算
    //把方法八中的乘除法换成位运算。
    
    // using Mat_ iterator and bitwise
    void colorReduce9(cv::Mat &image, int div=64) {
        // div must be a power of 2
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        // get iterators
        cv::Mat_<cv::Vec3b>::iterator it= image.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::iterator itend= image.end<cv::Vec3b>();
        for ( ; it!= itend; ++it) {
            (*it)[0]= (*it)[0]&mask + div/2;
            (*it)[1]= (*it)[1]&mask + div/2;
            (*it)[2]= (*it)[2]&mask + div/2;
        }
    }
    
    //方法十：MatIterator_
    //和方法八基本相同。
    
    // using MatIterator_
    void colorReduce10(cv::Mat &image, int div=64) {
        cv::Mat_<cv::Vec3b> cimage= image;
        cv::Mat_<cv::Vec3b>::iterator it=cimage.begin();
        cv::Mat_<cv::Vec3b>::iterator itend=cimage.end();
        for ( ; it!= itend; it++) {
            (*it)[0]= (*it)[0]/div*div + div/2;
            (*it)[1]= (*it)[1]/div*div + div/2;
            (*it)[2]= (*it)[2]/div*div + div/2;
        }
    }
    
    
    //方法十一：图像坐标
    
    // using (j,i)
    void colorReduce11(cv::Mat &image, int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols; // number of columns
        for (int j=0; j<nr; j++) {
            for (int i=0; i<nc; i++) {
                image.at<cv::Vec3b>(j,i)[0]=     image.at<cv::Vec3b>(j,i)[0]/div*div + div/2;
                image.at<cv::Vec3b>(j,i)[1]=     image.at<cv::Vec3b>(j,i)[1]/div*div + div/2;
                image.at<cv::Vec3b>(j,i)[2]=     image.at<cv::Vec3b>(j,i)[2]/div*div + div/2;
            } // end of row
        }
    }
    
    //方法十二：创建输出图像
    //之前的方法都是直接修改原图，方法十二新建了输出图像，主要用于后面的时间对比。
    
    // with input/ouput images
    void colorReduce12(const cv::Mat &image, // input image
                       cv::Mat &result,      // output image
                       int div=64) {
        int nr= image.rows; // number of rows
        int nc= image.cols ; // number of columns
        // allocate output image if necessary
        result.create(image.rows,image.cols,image.type());
        // created images have no padded pixels
        nc= nc*nr;
        nr= 1;  // it is now a 1D array
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        for (int j=0; j<nr; j++) {
            uchar* data= result.ptr<uchar>(j);
            const uchar* idata= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                *data++= (*idata++)&mask + div/2;
                *data++= (*idata++)&mask + div/2;
                *data++= (*idata++)&mask + div/2;
            } // end of row
        }
    }
    
    //方法十三：重载操作符
    //Mat重载了+&等操作符，可以直接将两个Scalar(B,G,R)数据进行位运算和数学运算。
    
    // using overloaded operators
    void colorReduce13(cv::Mat &image, int div=64) {
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        // perform color reduction
        image=(image&cv::Scalar(mask,mask,mask))+cv::Scalar(div/2,div/2,div/2);
    }
    //指针*++访问和位运算是最快的方法；而不断的计算image.cols*image.channles()花费了大量重复的时间；另外迭代器访问虽然安全，但性能远低于指针运算；通过图像坐标(j,i)访问时最慢的，使用重载操作符直接运算效率最高。
    




};

//计算彩色的bgr图像的直方图
class ColorHistogram{
private:
    int histSize[3];//项的数量
    float hranges[2];//像素的最大与最小值
    const float *ranges[3];
    int channels[3];
public:
    ColorHistogram(){
        //准备彩色直方图的参数
        histSize[0] = histSize[1] = histSize[2] = 256;
        hranges[0] = 0.0;//bgr的范围
        hranges[1] = 255.0;
        ranges[0] = hranges;//所有通道拥有相同的范围
        ranges[1] = hranges;
        ranges[2] = hranges;
        channels[0] = 0;
        channels[1] = 1;
        channels[2] = 2;
        
    }
    MatND getHistogram(const cv::Mat &image){
        cv::MatND hist;
        calcHist(&image, 1, channels, cv::Mat(), hist, 3, histSize, ranges);
        return hist;
    }
    //使用掩码计算一维色调直方图，bgr图像需转换为hsv色彩空间
    //并取出低饱和度的像素
    MatND getHueHistogram(const cv::Mat &image,int minSaturation = 0){
        MatND hist;
        Mat hsv;
        cvtColor(image, hsv, CV_BGR2HSV);
        //是否使用掩码
        Mat mask;
        if(minSaturation > 0){
            vector<Mat> v;
            split(hsv, v);
            //标出低饱和度的像素
            threshold(v[1], mask, minSaturation, 255, CV_THRESH_BINARY);
        }
        hranges[0] = 0;
        hranges[1] = 180.0;
        channels[0] = 0;//色调通道
        calcHist(&hsv, 1, channels, mask, hist, 1, histSize, ranges);
        return hist;
    }
    cv::Mat colorReduce(cv::Mat &image, int div)
    {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                uchar * tmp;
                *tmp ++ = *data ++;
                *tmp++ = *data/div*div + div/2;
            } // end of row
        }
        return image;
    }
    void colorReduce6(cv::Mat &image, int div) {
        int nr= image.rows; // number of rows
        int nc= image.cols * image.channels(); // total number of elements per line
        if (image.isContinuous())  {
            // then no padded pixels
            nc= nc*nr;
            nr= 1;  // it is now a 1D array
        }
        int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
        // mask used to round the pixel value
        uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
        for (int j=0; j<nr; j++) {
            uchar* data= image.ptr<uchar>(j);
            for (int i=0; i<nc; i++) {
                uchar * tmp;
                *tmp ++ = *data ++;
                *tmp ++ = *data&mask + div/2;
            } // end of row
        }
    }

    

    
    
};

class ContentFinder{
private:
    float hranges[2];
    const float* ranges[3];
    int channels[3];
    float threshold;
    MatND histogram;
public:
    ContentFinder():threshold(-1.0f){
        ranges[0] = hranges;
        ranges[1] = hranges;
        ranges[2] = hranges;
    }
    void setThreshold(float t){
        threshold = t;
    }
    float getThreshold(){
        return threshold;
    }
    void setHistogram(const cv::MatND &h){
        histogram = h;
        normalize(histogram,histogram,1.0);
    }
    cv::Mat find(const cv::Mat &image,float minValue,float maxValue,int* channels,int dim)//探测图片中的指定roi
    {
        cv::Mat result;
        hranges[0] = minValue;
        hranges[1] = maxValue;
        for (int i = 0; i < dim; i ++)
        {
            this->channels[i] = channels[i];
        }
        calcBackProject(&image, 1, channels, histogram, result, ranges,255.0);
        if (threshold > 0.0)
        {
            cv::threshold(result, result, 255*threshold, 255, CV_THRESH_BINARY);
        }
        return result;
    }
    
    
};

//比较直方图
class ImageComparator{
    private:
    Mat reference;
    Mat input;
    MatND refH;
    MatND inputH;
    ColorHistogram hist;
    int div;//减色因子
    public:
    ImageComparator():div(32){}
    void setColorReduction(int factor){
        div = factor;
    }
    int getColorReduction(){
        return div;
    }
    void setReferenceImage( Mat &image){
        reference = hist.colorReduce(image,div);
        refH = hist.getHistogram(reference);
    }
    double compare(Mat &image){
        input = hist.colorReduce(image, div);
        inputH = hist.getHistogram(input);
        return compareHist(refH, inputH, CV_COMP_INTERSECT);//CV_COMP_INTERSECT交叉测量法，该方法比较每个直方图容器的值，并保留最小的一个
    }
};

//形态学检测图像特征
class MorphoFeatures{
    private:
    int threshold;
    Mat cross;
    Mat diamond;
    Mat square;
    Mat x;
    public:
    MorphoFeatures():threshold(-1),cross(5,5,CV_8U,Scalar(0)),diamond(5,5,CV_8U,Scalar(1)),square(5,5,CV_8U,Scalar(1)),x(5,5,CV_8U,Scalar(0))
    {
        //创建十字形元素
        for (int i = 0; i < 5; i ++) {
            cross.at<uchar>(2,i) = 1;
            cross.at<uchar>(i,2) = 1;
        }
        //创建菱形元素
        diamond.at<uchar>(0,0) = 0;
        diamond.at<uchar>(0,1) = 0;
        diamond.at<uchar>(1,0) = 0;
        diamond.at<uchar>(4,4) = 0;
        diamond.at<uchar>(3,4) = 0;
        diamond.at<uchar>(4,3) = 0;
        diamond.at<uchar>(4,0) = 0;
        diamond.at<uchar>(4,1) = 0;
        diamond.at<uchar>(3,0) = 0;
        diamond.at<uchar>(0,4) = 0;
        diamond.at<uchar>(0,3) = 0;
        diamond.at<uchar>(1,4) = 0;
        //创建x型元素
        for (int i = 0; i < 5; i ++) {
            x.at<uchar>(i,i) = 1;
            x.at<uchar>(4-i,i) = 1;
        }
    }
    void applyThreshold(Mat &res){
        if (threshold > 0) {
            cv::threshold(res, res, threshold, 255.0, CV_THRESH_BINARY);
        }
    }
    Mat getEdges(Mat &image){
        Mat res;
        morphologyEx(image, res, CV_MOP_GRADIENT, Mat());
        applyThreshold(res);
        return res;
        
    }
    Mat getCorners(Mat &image){
        Mat res;
        dilate(image, res, cross);
        erode(res, res, diamond);
        Mat result;
        dilate(image, result, x);
        erode(result, result, square);
        //通过对两张图像做差值得到角点图像
        absdiff(res, result, result);
        applyThreshold(result);
        return result;
        
    }
    void drawOnImage(Mat &binary,Mat &image){
        cv::Mat_<uchar>::const_iterator it = binary.begin<uchar>();
        cv::Mat_<uchar>::const_iterator itend = binary.end<uchar>();
        for (int i = 0; it != itend; ++it,++i)
        {
            if (!*it) {
                circle(image, cv::Point(i%image.step,i/image.step), 5, Scalar(255,0,0));
            }
        }
    }
};

//分水岭算法
class WatershedSegment{
    private:
    Mat markers;
    public:
    void setMarkers(Mat &markerImage){
        markerImage.convertTo(markers, CV_32S);//转换为整数图像
    }
    Mat process(Mat &image){
        watershed(image, markers);
        return markers;
    }
    cv::Mat getMarkers(cv::Mat &binary){
        cv::Mat fg;
        erode(binary, fg, cv::Mat(),cv::Point(-1,-1),6);//移除噪点与微小物体
        //识别不包含物体的像素
        cv::Mat bg;
        dilate(binary, bg, cv::Mat(),cv::Point(-1,-1),6);
        threshold(bg, bg, 1, 128, CV_THRESH_BINARY_INV);
        //创建标记图像
        cv::Mat markers(binary.size(),CV_8U,Scalar(0));
        markers = fg + bg;
        return markers;
    }
};

//拉普拉斯
class LaplacianZC{
    private:
    Mat img;
    Mat laplace;
    int aperture;
    public:
    LaplacianZC():aperture(3){
        
    }
    void setAperture(int a){
        aperture = a;
    }
    Mat computeLaplacian(Mat &image){
        Laplacian(image, laplace, CV_32F,aperture);
        img = image.clone();//用于零点交叉
        return laplace;
    }
    Mat getLaplacianImage(double scale = -1.0){//返回8位图像存储的拉普拉斯结果，零点交叉于灰度值128，必须在调用该函数之前，先调用computeLaplacian
        if (scale < 0) {
            double lapmin,lapmax;
            minMaxLoc(laplace, &lapmin,&lapmax);
            scale = 127/std::max(-lapmin, lapmax);
        }
        Mat laplaceImage;
        laplace.convertTo(laplaceImage, CV_8U,scale,128);
        return laplaceImage;
    }
    Mat getZeroCrossings(float thres = 1.0)
    {
        cv::Mat_<float>::const_iterator it = laplace.begin<float>() + laplace.step1();
        cv::Mat_<float>::const_iterator itend = laplace.end<float>();
        cv::Mat_<float>::const_iterator itup = laplace.begin<float>();
        Mat binary(laplace.size(),CV_8U,Scalar(255));//初始化为白色的二值图像
        cv::Mat_<uchar>::iterator itout = binary.begin<uchar>() + binary.step1();
        thres = -1.0 * thres;//对输入阈值取反
        for (; it != itend; ++it,++itout,++itup)
        {
            if (*it * *(it - 1) < thres) {//如果相邻像素的乘机为负数，则符号发生改变
                *itout = 0;//水平方向零点交叉
            }
            else if (*it * *itup < thres)*itout = 0;//垂直方向零点交叉
            
        }
        return binary;
    }
};

//霍夫变换
class LineFinder{
    private:
    Mat img;
    vector<Vec4i> lines;//包含线段的起点和终点的坐标
    double deltarho,deltatheta;//累加器的分辨率
    int minvote;//直线被接受时所需的最小投票数
    double minlength;//最小的直线长度
    double maxgap;//沿着直线方向的最大缺口
    public:
    //默认的累加器的分辨率为单个像素及1°
    //不设置缺口即最小长度的值
    LineFinder():deltarho(1),deltatheta(3.14/180),minvote(10),minlength(0.),maxgap(0.){
        
    }
    void steAccResolution(double drho,double dtheta){
        deltarho  = drho;
        deltatheta = dtheta;
    }
    void setMinVote(int minv){
        minvote = minv;
    }
    void setLineLengthAndGap(double length,double gap){
        minlength = length;
        maxgap = gap;
    }
    vector<Vec4i> findLines(Mat &image){
        lines.clear();
        HoughLinesP(image, lines, deltarho, deltatheta, minvote,minlength,maxgap);
        return lines;
    }
    void drawDetectedLines(Mat &image,Scalar color = Scalar::all(255)){
        vector<Vec4i>::const_iterator it2 = lines.begin();
        while (it2 != lines.end()) {
            cv::Point pt1((*it2)[0],(*it2)[1]);
            cv::Point pt2((*it2)[2],(*it2)[3]);
            line(image, pt1, pt2, Scalar(color));
            ++ it2;
            
        }
    }
    
};














#endif /* defined(__opencv__line_segment__) */
