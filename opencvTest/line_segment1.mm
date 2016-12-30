
//  line_segment.cpp
//  opencv
//
//  Created by incer on 15/5/7.
//  Copyright (c) 2015年 ce. All rights reserved.
//



#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/opencv.hpp>
//#include "opencv2/legacy.hpp"
//#include <opencv2/nonfree/nonfree.hpp>
#include "LSWMS.h"
//#include "bmp.h"
//#include "MWIS.h"
//#include "ScanBoardFilter.hpp"
#include "line_segment1.h"
#include <fstream>





#define _MAXI(_a,_b)      ((_a)-((_a)-(_b)&-((_b)>(_a))))
#define _MINI(_a,_b)      ((_a)+((_b)-(_a)&-((_b)<(_a))))
#define _INT_BITS         ((int)sizeof(int)*8)

//-----------------------------------【命名空间声明部分】---------------------------------------
//	     描述：包含程序所使用的命名空间
//-----------------------------------------------------------------------------------------------
using namespace cv;
using namespace std;
using namespace cv::line_descriptor;
//-----------------------------------【main( )函数】--------------------------------------------
//	     描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------



//struct LINE
//{
//    double a;
//    double b;
//    double c;
//    Point start;
//    Point end;
//};






class Graph
{
private:
    int V;
public:
    Graph(int V)
    {
        this->V =V;
        adj = new list<int> [V];
    }
    list<int> *adj;
    void addEdge(int v,int w);
    void BFS(int s,bool visited[]);
    void DFSUtil(int v, bool visited[]);
    Graph getTranspose();
    bool isCyclicUtil(int v, bool visited[], bool *rs);
    bool isConnected();
    bool isCyclic();
    vector<int> maxclique();
};

void Graph::addEdge(int v, int w)
{
    adj[v].push_back(w);
};

void Graph::DFSUtil(int v, bool visited[])
{
    visited[v] = true;
    list<int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i)
        if (!visited[*i])
        {
            DFSUtil(*i, visited);
        }
}

Graph Graph::getTranspose()
{
    Graph g(V);
    for (int v = 0; v<V;v++ ) {
        list<int>::iterator i;
        for (i = adj[v].begin(); i!=adj[v].end(); ++i) {
            g.adj[*i].push_back(v);
        }
    }
    return g;
}

bool Graph::isConnected()
{
    bool visited[V];
    for (int i = 0; i < V; i++)
        visited[i] = false;
    DFSUtil(0, visited);
    for (int i = 0; i < V; i++)
        if (visited[i] == false)
            return false;
    Graph gr = getTranspose();
    for(int i = 0; i < V; i++)
        visited[i] = false;
    gr.DFSUtil(0, visited);
    for (int i = 0; i < V; i++)
        if (visited[i] == false)
            return false;
    return true;
}

bool Graph::isCyclicUtil(int v, bool visited[], bool *recStack)
{
    if (visited[v] == false)
    {
        // Mark the current node as visited and part of recursion stack
        visited[v] = true;
        recStack[v] = true;
        
        // Recur for all the vertices adjacent to this vertex
        list<int>::iterator i;
        for (i = adj[v].begin(); i != adj[v].end(); ++i)
        {
            if (!visited[*i] && isCyclicUtil(*i, visited, recStack))
                return true;
            else if (recStack[*i])
                return true;
        }
        
    }
    recStack[v] = false; // remove the vertex from recursion stack
    return false;
}

bool Graph::isCyclic()
{
    // Mark all the vertices as not visited and not part of recursion
    // stack
    bool *visited = new bool[V];
    bool *recStack = new bool[V];
    for (int i = 0; i < V; i++)
    {
        visited[i] = false;
        recStack[i] = false;
    }
    
    // Call the recursive helper function to detect cycle in different
    // DFS trees
    for (int i = 0; i < V; i++)
        if (isCyclicUtil(i, visited, recStack))
            return true;
    
    return false;
}

class Clique
{
    friend int MaxClique(int **,int[],int);
private:
    void Backtrack(int i);
    int **a,		//圖G的鄰接矩陣
    n,			//圖G的頂點數
    *x,			//當前解
    *bestx,		//當前最優解
    cn,			//當前頂點數
    bestn;		//當前最大頂點數
};

// 計算最大團
void Clique::Backtrack(int i)
{
    if (i > n) // 到達葉結點
    {
        for (int j = 1; j <= n; j++)
        {
            bestx[j] = x[j];
        }
        bestn = cn;
        return;
    }
    // 檢查頂點 i 與當前團的連接
    int OK = 1;
    for (int j = 1; j < i; j++)
        if (x[j] && a[i][j] == 0)
        {
            // i與j不相連
            OK = 0;
            break;
        }
    
    if (OK)// 進入左子樹
    {
        x[i] = 1;
        cn++;
        Backtrack(i+1);
        x[i] = 0;
        cn--;
    }
    
    if (cn + n - i >= bestn)// 進入右子樹
    {
        x[i] = 0;
        Backtrack(i+1);
    }
}

int MaxClique(int **a, int *v, int n)
{
    Clique Y;
    
    //初始化Y
    Y.x = new int[n+1];
    Y.a = a;
    Y.n = n;
    Y.cn = 0;
    Y.bestn = 0;
    Y.bestx = v;
    Y.Backtrack(1);
    
    for (int j = 1; j <= n; j++)
    {
        v[j] = Y.bestx[j];
    }
    
    delete[] Y.x;
    return Y.bestn;
}
//计算两点之间的直线距离
double ls::line_size(cv::Point &p1,cv::Point &p2)
{
    return sqrt(pow(p1.x - p2.x,2)+pow(p1.y - p2.y,2));
}

//计算两点组成的直线与水平方向的夹角

float ls::line_jiao(cv::Point &p1,cv::Point &p2)
{
    if(p2.y == p1.y)//两点处于同一水平直线上,夹角为0
        return 0;
    else
    {
        cv::Point min = p2.y<p1.y?p2:p1;//取出具有最小纵坐标的点做最小点
        cv::Point max = p2.y>p1.y?p2:p1;//取出具有最大纵坐标的点做最大点
        return acos((max.x - min.x)/line_size(p1, p2))*(180.0/CV_PI);
        
    }
}

float ls::line_jiao1(cv::Point &p1,cv::Point &p2)
{
    if(p2.y == p1.y)
        return 0;
    else
    {
        cv::Point min = p2.y<p1.y?p2:p1;
        cv::Point max = p2.y>p1.y?p2:p1;
        return acos((min.x - max.x)/line_size(p1, p2))*(180.0/CV_PI);
    }
}


//求直线中间值
int ls::ca(LSEG &line,int k)
{
    // float kdx = (float)line[0].x - (float)line[1].x;
    //float kdy = (float)line[0].y - (float)line[1].y;
    // cv::Point p_middle;//两点连线的中点
    // p_middle.x = (line[0].x + line[1].x)/2.0;
    //  p_middle.y = (line[0].y + line[1].y)/2.0;
    if(k==1 )//竖直
    {
        return (line[0].x + line[1].x)*0.5;//((1.5*line[0].x+line[1].x)*0.5 - 0.15*(line[0].y+line[1].y)*0.5);//取横坐标的中间值-0.25   －0.12
    }
    //else if(k==1)//水平
    //{
    else if (0 == k)
    {
        return -((line[0].y+line[1].y)*0.5);//(0.85*((line[0].y+line[1].y)*0.5) - 0.35*((line[0].x+line[1].x)*0.5));//取纵坐标的中间值0.85,-0.35
    }
    //    else if(k==2)//右倾斜
    //    {
    //        return abs(-(kdx/line_size(line[0],line[1]))*krows*0.7 + (kdx/line_size(line[0],line[1]))*p_middle.y - (kdy/line_size(line[0],line[1]))*p_middle.x);//
    //    }
    //    else if(k==3)//左倾斜
    //    {
    //        return abs(-(kdx/line_size(line[0],line[1]))*krows*0.7 + (kdx/line_size(line[0],line[1]))*p_middle.y - (kdy/line_size(line[0],line[1]))*p_middle.x);
    //    }
    else
        return 0;
}


float ls::ca1(LSEG &line, int k)
{
    //float kdx = (float)line[0].x - (float)line[1].x;
    //float kdy = (float)line[0].y - (float)line[1].y;
    cv::Point zp2;
    zp2.x = (line[0].x + line[1].x)/2.0;
    zp2.y = (line[0].y + line[1].y)/2.0;
    double kk = line_jiao(line[0], line[1]);
    if(k == -1)
        return line_jiao(line[0],line[1]);
    //    else if(abs(kk - 90) <= 2)
    //    {
    //        return zp2.x;
    //    }
    //    else if(abs(kk - 0) <= 2)
    //    {
    //        return zp2.y;
    //    }
    else if(kk <= 90)
    {
        return fabs(sin(kk/180.0*CV_PI)*zp2.x+cos(kk/180.0*CV_PI)*zp2.y);
    }
    else
        return fabs(sin(kk/180.0*CV_PI)*zp2.x+cos(kk/180.0*CV_PI)*zp2.y-cos(kk/180.0*CV_PI)*krows);
    //    if(k==0)
    //    {
    //        return (line[0].x+line[1].x)*0.5;
    //    }
    //    else if(k==1)
    //    {
    //        return (line[0].y+line[1].y)*0.5;
    //    }
    //    else if(k==2)
    //    {
    //        return abs(-(kdx/line_size(line[0],line[1]))*krows*0.7 + (kdx/line_size(line[0],line[1]))*zp2.y - (kdy/line_size(line[0],line[1]))*zp2.x);
    //    }
    //    else if(k==3)
    //    {
    //        return abs(-(kdx/line_size(line[0],line[1]))*krows*0.7 + (kdx/line_size(line[0],line[1]))*zp2.y - (kdy/line_size(line[0],line[1]))*zp2.x);
    //    }
    //    else
    //    {
    //        return line_jiao(line[0],line[1]);
    //    }
}


//求直线与水平线的交点横坐标
int ls::zero_point_x(LSEG &line,LSEG &baseline, int k)
{
    LINE l_base = makeline(baseline[0], baseline[1]);
    LINE l = makeline(line[0],line[1]);
    cv::Point inter;
    lineintersect(l_base,l,inter);
    return inter.x;
}


//递归排序
void ls::run(vector<LSEG> &lines1,int left,int right,int k,LSEG &baseline)
{
    int i,j;
    int middle;
    int zero_point;
    LSEG iTemp;
    i = left;
    j = right;
    middle = ca(lines1[(left+right)/2],k);
    if (1 == k)
    {
        //求中间值
        do{
            while((ca(lines1[i],k)<middle) && (i<right))//从左扫描大于中值的数
                i++;
            while((ca(lines1[j],k)>middle) && (j>left))//从右扫描大于中值的数
                j--;
            if(i<=j)//找到了一对值
            {
                //交换
                iTemp = lines1[i];
                lines1[i] = lines1[j];
                lines1[j] = iTemp;
                i++;
                j--;
            }
        }while(i<=j);//如果两边扫描的下标交错，就停止（完成一次）
        //当左边部分有值(left<j)，递归左半边
        if(left<j)
            run(lines1,left,j,k,baseline);
        //当右边部分有值(right>i)，递归右半边
        if(right>i)
            run(lines1,i,right,k,baseline);
        
    }
    else if(0 == k || 2 == k || 3 == k)
    {
        zero_point = zero_point_x(lines1[(left+right)/2],baseline,k);
        do
        {
            while ((zero_point_x(lines1[i],baseline ,k) < zero_point)  && (i < right))
            {
                i ++;
            }
            while ((zero_point_x(lines1[j],baseline, k) > zero_point)  && (j > left))
            {
                j --;
            }
            if (i <= j)
            {
                //交换
                iTemp = lines1[i];
                lines1[i] = lines1[j];
                lines1[j] = iTemp;
                i++;
                j--;
            }
            
        }while (i <= j);
        //当左边部分有值(left<j)，递归左半边
        if(left<j)
            run(lines1,left,j,k,baseline);
        //当右边部分有值(right>i)，递归右半边
        if(right>i)
            run(lines1,i,right,k,baseline);
        
        
    }
    
    
}

//递归排序
void ls::run1(vector<LSEG> &lines1,int left,int right,int k)
{
    int i,j;
    int middle;
    //    int zero_point;
    LSEG iTemp;
    i = left;
    j = right;
    middle = ca(lines1[(left+right)/2],k);
    if (1 == k)
    {
        //求中间值
        do{
            while((ca(lines1[i],k)<middle) && (i<right))//从左扫描大于中值的数
                i++;
            while((ca(lines1[j],k)>middle) && (j>left))//从右扫描大于中值的数
                j--;
            if(i<=j)//找到了一对值
            {
                //交换
                iTemp = lines1[i];
                lines1[i] = lines1[j];
                lines1[j] = iTemp;
                i++;
                j--;
            }
        }while(i<=j);//如果两边扫描的下标交错，就停止（完成一次）
        //当左边部分有值(left<j)，递归左半边
        if(left<j)
            run1(lines1,left,j,k);
        //当右边部分有值(right>i)，递归右半边
        if(right>i)
            run1(lines1,i,right,k);
        
    }
    else if(0 == k || 2 == k || 3 == k)
    {
        //        zero_point = zero_point_x(lines1[(left+right)/2],baseline,k);
        //        do
        //        {
        //            while ((zero_point_x(lines1[i],baseline ,k) < zero_point)  && (i < right))
        //            {
        //                i ++;
        //            }
        //            while ((zero_point_x(lines1[j],baseline, k) > zero_point)  && (j > left))
        //            {
        //                j --;
        //            }
        //            if (i <= j)
        //            {
        //                //交换
        //                iTemp = lines1[i];
        //                lines1[i] = lines1[j];
        //                lines1[j] = iTemp;
        //                i++;
        //                j--;
        //            }
        //
        //        }while (i <= j);
        //        //当左边部分有值(left<j)，递归左半边
        //        if(left<j)
        //            run(lines1,left,j,k,baseline);
        //        //当右边部分有值(right>i)，递归右半边
        //        if(right>i)
        //            run(lines1,i,right,k,baseline);
        
        
    }
    
    
}



//快速排序算法
void ls::QuickSort(vector<LSEG> &lines1,int Count,int k,LSEG & baseline)
{
    run(lines1, 0,Count-1,k,baseline);
}

void ls::QuickSort1(vector<LSEG> &lines1,int Count,int k)
{
    run1(lines1, 0,Count-1,k);
}



void ls::horizon_classify(vector<LSEG> &lines)
{
    Point2d middle,min;
    vector<double> mid_x,mid_y;
    double th = 5;
    vector<vector<LSEG>> l;
    
    
    for (int i = 0; i < lines.size(); i ++)
    {
        mid_x.push_back((lines[i][0].x + lines[i][1].x)/2.0);
        mid_y.push_back((lines[i][0].y + lines[i][0].y)/2.0);
    }
    min.x = min_array(mid_x);
    for (int i = 0; i < lines.size(); i ++)
    {
        middle.x = ((lines[i][0].x + lines[i][1].x)/2.0);
        //mid_y.push_back((lines[i][0].y + lines[i][0].y)/2.0);
        if (abs(min.x - middle.x) < th) {
            l[0].push_back(lines[i]);
        }
        else if (abs(min.x - middle.x) >= th && abs(min.x - middle.x) < 5*th)
        {
            l[1].push_back(lines[i]);
        }
        else
        {
            l[2].push_back(lines[i]);
        }
    }
    
    
}

double ls::max_array(vector<double> a)
{
    int t=a[0],i;
    for(i=1;i<10;i++)t=(t>a[i])?t:a[i];
    return t;
}

double ls::min_array(vector<double> a)
{
    int t=a[0],i;
    for(i=1;i<10;i++)t=(t<a[i])?t:a[i];
    return t;
}


#if 0
bool getCross(LSEG line1, LSEG line2)
{
    Point CrossP;
    
    int meddium1 = (line1[0].y + line1[1].y)*0.5;
    int meddium2 = (line2[0].y + line2[1].y)*0.5;
    
    //y = a * x + b;
    int a1 = (line1[0].y - line1[1].y) / (line1[0].x - line1[1].x);
    int b1 = line1[0].y - a1 * (line1[1].x);
    
    int a2 = (line2[0].y - line2[1].y) / (line2[0].x - line2[1].x);
    int b2 = line2[0].y - a1 * (line2[1].x);
    if(a2 - a1  == 0)
        return 0;
    else
    {
        CrossP.x = (b1 - b2) / (a2 - a1);
        CrossP.y = a1 * CrossP.x + b1;
        if((CrossP.y < meddium1&&CrossP.y<meddium2)||(CrossP.y > meddium1&&CrossP.y > meddium2))
            return 0;
        else
            return 1;
    }
}

void ls::QuickSort(vector<LSEG> &lines1,int Count,int k)
{
    run(lines1, 0,Count-1,k);
    for (int i = 0; i < Count-1; i++) {
        if(getCross(lines1[i], lines1[i+1]))
        {
            LSEG iTemp;
            iTemp = lines1[i];
            lines1[i] = lines1[i+1];
            lines1[i+1] = iTemp;
        }
    }
}
#endif
//能量函数
double ls::power(Mat &src,Mat &angle,cv::Point &a,cv::Point &b)
{
    //    if (a.x == b.x || !y) {
    //        x = 50;
    //        y = 150;
    //    }
    double dx = (b.x-a.x)/(double)sqrt(pow(b.x-a.x,2)+pow(b.y-a.y, 2));
    double dy = (b.y-a.y)/(double)sqrt(pow(b.x-a.x,2)+pow(b.y-a.y, 2));
    double sum = 0;
    unsigned int n = 0;
    for(int i=0;i<(int)sqrt(pow(b.x-a.x, 2)+pow(b.y-a.y, 2));i+=1)
    {
        int y = a.y+i*dy;
        int x = a.x+i*dx;
        
        bool location_p = x < angle.cols && x > 0 && y < angle.rows && y > 0;
        if(location_p)
        {
            if(angle.at<float>(y,x) * (180.0/CV_PI) < 361 && angle.at<float>(y,x) * (180.0/CV_PI) > -1)
            {
                
                double e = acos(abs(dx*cos(angle.at<float>(y,x))+dy*sin(angle.at<float>(y,x))))*(180.0/CV_PI);
                sum +=e;
                n++;
            }
        }
        else
        {
            break;
        }
    }
    return sum/(double)n;
}
//已知一个点，点到直线的距离，以及直线的一个垂直向量，求该点的投影点
cv::Point ls::prpoint(cv::Point &center,int d,cv::Point2f &v)
{
    cv::Point pt;
    pt.x = center.x - v.x*d;
    pt.y = center.y - v.y*d;
    return pt;
}


//判断一个点的投影是否在线段上
int ls::point_line(cv::Point &p1,cv::Point &p2,cv::Point &tp)
{
    cv::Point ap;
    ap.x = -1;
    float kdx = abs((float)p1.x - (float)p2.x);
    float kdy = abs((float)p1.y - (float)p2.y);
    if(kdy < 20)//如果两个点组成的直线的垂直距离较短的话，将ap置为与p1水平的点
        ap.y = p1.y;
    else if(kdx < 20)//如果两个点组成的直线的水平距离较短的话，将ap置为与p1竖直的点
    {
        ap.x = p1.x;
        ap.y = -1;
    }
    else
        ap.y = p1.y - (kdy/kdx)*(-1-p1.x);
    
    float min_size = line_size(ap, p1)<line_size(ap, p2)?line_size(ap, p1):line_size(ap, p2);
    //float min_size = MIN(line_size(ap, p1), line_size(ap, p2));
    float max_size = line_size(ap, p1)>line_size(ap, p2)?line_size(ap, p1):line_size(ap, p2);
    //float max_size = MAX(line_size(ap, p1), line_size(ap, p2));
    //投影点在端点两侧，不在线段上
    if(line_size(ap, tp) > max_size)
        return  2;
    else if(line_size(ap, tp)<min_size)
        return  1;
    else//line_size(ap, tp) > min_size && line_size(ap, tp) < max_size,既投影点位于两端点中间，在线段上，返回0
        return  0;
}


//判断一个点的投影是否落在线段的端点中间
int ls::pers_point_inside(Point &p1,Point &p2,Point &p_pers)
{
    float d1,d2,d3;
    d1 = line_size(p1, p2);
    d2 = line_size(p1, p_pers);
    d3 = line_size(p2, p_pers);
    if (d1 == (d2 + d3))
        return 1;//落在端点中间
    else
        return 0;//落在端点两侧
}



int ls::warf(Mat &src,Mat &src1,Mat &angle,LSEG &line1,LSEG &line2,vector<Point> &rc  ,int k,float &tg_1,float &tg_2)
{
    Point2f v1,fv1,vt1,fvt1,v2,fv2,vt2,fvt2;
    cv::Point p1,p2,cp,tp1,tp2,p3,p4,tp3,tp4;
    cv::Point fp1_old,lp1_old;
    cv::Point fp2_old,lp2_old;
    cv::Point mid1,mid2;
    cv::Point u1,d1,u2,d2;
    //    cout<<ca(line1, k)<<" "<<ca(line2, k)<<endl;
#if 0
    if(k == 1)
    {
        fp1 = line1[1]; //< line1[1].y ? line1[0] : line1[1];8
        //circle(src, line1[1], 3, Scalar(0,0,255));
        lp1 = line1[0]; //> line1[1].y ? line1[0] : line1[1];
        //circle(src, line1[0], 3, Scalar(0,255,0));
        fp2 = line2[1]; //< line2[1].y ? line2[0] : line2[1];
        //circle(src, line2[1], 3, Scalar(255,0,0));
        lp2 = line2[0]; //> line2[1].y ? line2[0] : line2[1];
        //circle(src, line2[0], 3, Scalar(120,100,255));
        //imshow("fplp", src);
    }
    else
    {
#endif
        u1 = line1[0]; //< line1[1].y ? line1[0] : line1[1];
        //circle(src, line1[0], 3, Scalar(0,0,255));
        d1 = line1[1]; //> line1[1].y ? line1[0] : line1[1];
        // circle(src, line1[1], 3, Scalar(0,255,0));
        u2 = line2[0];
        //circle(src, line2[0], 3, Scalar(255,0,0));
        d2 = line2[1];
        //circle(src, line2[1], 3, Scalar(120,100,255));
        //imshow("fplp", src);
        //}
        mid1.x = (u1.x + d1.x)/2.0;
        mid1.y = (u1.y + d1.y)/2.0;
        
        mid2.x = (u2.x + d2.x)/2.0;
        mid2.y = (u2.y + d2.y)/2.0;
        
        
        double fl1 = line_size(u1,d1);
        double fl2 = line_size(u2,d2);
        float max = fl1 > fl2 ? fl1 : fl2;//最长直线
        
        v1.x = ((float)d1.x - (float)u1.x)/fl1;
        v1.y = ((float)d1.y - (float)u1.y)/fl1;
        vt1.x = -v1.y;
        vt1.y = v1.x;
        fv1.x = -v1.x;
        fv1.y = -v1.y;
        fvt1.x = -vt1.x;
        fvt1.y = -vt1.y;
        
        
        v2.x = ((float)d2.x - (float)u2.x)/fl2;
        v2.y = ((float)d2.y - (float)u2.y)/fl2;
        vt2.x = -v2.y;
        vt2.y = v2.x;
        fv2.x = -v2.x;
        fv2.y = -v2.y;
        fvt2.x = -vt2.x;
        fvt2.y = -vt2.y;
        
        fp1_old = prpoint(mid1, 50, v1);
        lp1_old = prpoint(mid1, 50, fv1);
        
        fp2_old = prpoint(mid2, 50, v2);
        lp2_old = prpoint(mid2, 50, fv2);
        
        
        p1 = u1;//fp1_old;
        p2 = d1;//lp1_old;
        p3 = u2;//fp2_old;
        p4 = d2;//lp2_old;
        
        //    cp.x = (line2[0].x + line2[1].x)/2.0;
        //    cp.y = (line2[0].y + line2[1].y)/2.0;
        
        float dd1 = abs(v2.y*p1.x - v2.x*p1.y + v2.x*p3.y -v2.y*p3.x);//p1到v2的距离
        float dd2 = abs(v2.y*p2.x - v2.x*p2.y + v2.x*p3.y -v2.y*p3.x);//p2到v2的距离
        float dd3 = abs(v1.y*p3.x - v1.x*p3.y + v1.x*p1.y -v1.y*p1.x);//p3到v1的距离
        float dd4 = abs(v1.y*p4.x - v1.x*p4.y + v1.x*p1.y -v1.y*p1.x);//p4到v1的距离
        //
        tp1 = prpoint(p1, dd1, vt2);
        tp2 = prpoint(p2, dd2, vt2);
        tp3 = prpoint(p3, dd3, fvt1);
        tp4 = prpoint(p4, dd4, fvt1);
        
        
        int a = point_line(p3, p4, tp1);
        int b = point_line(p3, p4, tp2);
        // int c = point_line(p1, p2, tp3);
        // int d = point_line(p1, p2, tp4);
        //int a = pers_point_inside(p3, p4, tp1);
        //int b = pers_point_inside(p3, p4, tp2);
        //
        Mat oo;
        oo = Mat::zeros(src.rows, src.cols, CV_8UC1);
        line(oo, line1[0], line1[1], Scalar(255,255,255),3,8);
        line(oo, line2[0], line2[1], Scalar(255,255,255),3,8);
        circle(oo, tp1, 3, Scalar(255,255,255));
        circle(oo, tp2, 3, Scalar(255,255,255));
        circle(oo, tp3, 3, Scalar(255,255,255));
        circle(oo, tp4, 3, Scalar(255,255,255));
        circle(oo, Point(3,krows*0.7), 3, Scalar(255,255,255));
        line(oo, Point(3,krows*0.7), Point(600,krows*0.4), Scalar(255,255,255));
        imshow("image1", oo);
        waitKey(10);
        
        
        
        if(a && b  && a == b)//l1的投影完全落在l2外
        {
            //        P1 = tp4;
            //        P2 = p1;
            //        P3 = p4;
            //        p4 = tp1;
            return 0;
        }
        
        if(abs(acos(v1.x*v2.x+v1.y*v2.y)*(180.0/CV_PI)) > 8)//放弃相交程度过大的直线2.8,2.7 2.2
        {
            return 3;
        }
        
        Point P1,P2,P3,P4;
        P1 = a?tp3:p1;
        P2 = b?tp4:p2;
        P3 = a?p3:tp1;
        P4 = b?p4:tp2;
        
        Point p11,p12,p21,p22;
        float del1 = 0,del2 = 0;
        double e_min1=0,e_min2=0;
        bool active_d1 = true,active_d2 = true;//生长停止的标志，true表示继续生长，反之停止生长
        bool location_p1 = true,location_p2 = true,location_p3 = true,location_p4 = true;
        
        float min_x = 0,max_x = 0,min_y = 0,max_y = 0;
        while((active_d1 && location_p1 && location_p3) || (active_d2 && location_p2 && location_p4))
        {
            Mat image2 = src.clone();
            
            Point op1,op2,op3,op4;
            p11 = prpoint(P1, del1, v1);
            p12 = prpoint(P2, del2, fv1);
            p21 = prpoint(P3, del1, v2);
            p22 = prpoint(P4, del2, fv2);
            double len1 = line_size(p11,p21);
            double len2 = line_size(p12,p22);
            double len3 = line_size(p11,p12);
            double len4 = line_size(p21,p22);
            min_x = len1 < len2 ? len1 : len2;
            max_x = len1 > len2 ? len1 : len2;
            min_y = len3 < len4 ? len3 : len4;
            max_y = len3 > len4 ? len3 : len4;
            
            if(min_x < 7.0 || max_x > 130 || max_y > 4*max)/////////////
            {
                return 2;
            }
            //保证不出边界
            location_p1 = p11.x < angle.cols && p11.x > 0 && p11.y < angle.rows && p11.y > 0;
            location_p2 = p12.x < angle.cols && p12.x > 0 && p12.y < angle.rows && p12.y > 0;
            location_p3 = p21.x < angle.cols && p21.x > 0 && p21.y < angle.rows && p21.y > 0;
            location_p4 = p22.x < angle.cols && p22.x > 0 && p22.y < angle.rows && p22.y > 0;
            //计算两条直线的端点的能量值，一致性
            float power1 = power(src, angle, p11, p21);
            float power2 = power(src, angle, p12, p22);
            // cout<<"power1:"<<power1<<"power2:"<<power2<<endl;
            //向两边生长,第二个满足条件，大于一个阈值，表示停止生长，并且需要满足第三个条件
            cv::Point fp1 = prpoint(p11,5,v1);
            cv::Point lp1 = prpoint(p21,5,v2);
            //
            circle(image2, p11, 3, Scalar(255,255,0));
            circle(image2, p21, 3, Scalar(255,255,0));
            circle(image2, fp1, 3, Scalar(255,255,0));
            circle(image2, lp1, 3, Scalar(255,255,0));
            circle(image2, p12, 3, Scalar(255,255,0));
            circle(image2, p22, 3, Scalar(255,255,0));
            //
            
            float zpower1_min = power(src, angle, p11, fp1) < power(src, angle, p21, lp1) ? power(src, angle, p11, fp1)
            : power(src, angle, p21, lp1);//第三个条件：取出能量最大值，小于一个阈值表示d不在增长
            float zpower1_max = power(src, angle, p11, fp1) > power(src, angle, p21, lp1) ? power(src, angle, p11, fp1)
            : power(src, angle, p21, lp1);
            float mean_power1 = 0.5 * (zpower1_min + zpower1_max);
            //float d_power1 =sqrt(0.5 * ((zpower1_max - mean_power1) * (zpower1_max - mean_power1) + (zpower1_min - mean_power1) * (zpower1_min - mean_power1))) ;
            // cout<<"mean_power1:"<<mean_power1<<"d_power1:"<<d_power1<<endl;
            
            //cout<<d_power1<<endl;
            //float zpower1 = 0.45 * zpower1_max + 0.4 * zpower1_min;//0.45  0.4
            float zpower1 = zpower1_max;
            
            //        cout<<"1:"<<power1<<"   "<<zpower1<<endl;
            
            
            
            cv::Point fp2 = prpoint(p12,5,fv1);
            cv::Point lp2 = prpoint(p22,5,fv2);
            
            float zpower2_min = power(src, angle, p12, fp2) < power(src, angle, p22, lp2) ? power(src, angle, p12, fp2) : power(src, angle, p22, lp2);
            float zpower2_max = power(src, angle, p12, fp2) > power(src, angle, p22, lp2) ? power(src, angle, p12, fp2) : power(src, angle, p22, lp2);
            float mean_power2 = 0.5 * (zpower2_min + zpower2_max);
            //float d_power2 =sqrt(0.5 * ((zpower2_max - mean_power2) * (zpower2_max - mean_power2) + (zpower2_min - mean_power2) * (zpower2_min - mean_power2))) ;
            //cout<<"mean_power2:"<<mean_power2<<"d_power2:"<<d_power2<<endl;
            //cout<<"zpower1_max:"<<zpower1_max<<"zpower1_min:"<<zpower1_min<<endl;
            //cout<<"zpower2_max:"<<zpower2_max<<"zpower2_min:"<<zpower2_min<<endl;
            
            
            float zpower2 = zpower2_max;
            
            float tg1 = 0.80*mean_power1 +  0.20*mean_power2;//0.65  0.35
            
            if(active_d1 && location_p1 && location_p3)//上端或左端
            {
                if((power1 > tg1  && zpower1 < 1.5*tg1))//|| abs(power1 - power2) < 10)//)//满足第二个条件和第三个条件，不再生长30,42
                {
                    active_d1 = false;
                    e_min1 = abs(power1);
                }
                else//否则继续生长
                {
                    del1 += 1;//每次增加1个像素
                    
                }
            }
            else
            {
                //            if(power1>60)
                //            {
                //                active_d1 = true;
                //            }
            }
            
            //        cout<<"2:"<<power2<<"   "<<zpower2<<endl;
            float tg2 = 0.20*mean_power1 + 0.80*mean_power2;
            if(active_d2 && location_p2 && location_p4)
            {
                if((power2 > tg2 && zpower2 < 1.5*tg2))//|| abs(power1 - power2) < 10)//右端和下端
                {
                    active_d2 = false;
                    e_min2 = abs(power2);
                }
                else
                {
                    del2 += 1;
                }
            }
            
            else
            {
                //           if(power2>60)
                //            {
                //                active_d2 = true;
                //            }
            }
            tg_1 = tg1;
            tg_2 = tg2;
            line(image2, p11, p12, Scalar(0,0,255),2,8);
            line(image2, p11, p21, Scalar(0,0,255),2,8);
            line(image2, p22, p12, Scalar(0,0,255),2,8);
            line(image2, p22, p21, Scalar(0,0,255),2,8);
            imshow("image", image2);
            waitKey(10);
        }
        
        //int x_size = line_size(p11,p12)<line_size(p21,p22)?line_size(p21,p22):line_size(p11,p12);
        //int y_size = line_size(p11,p21)<line_size(p12,p22)?line_size(p12,p22):line_size(p11,p21);
        //    {
        int R = ( rand() % (int) ( 255 + 1 ) );
        int G = ( rand() % (int) ( 255 + 1 ) );
        int B = ( rand() % (int) ( 255 + 1 ) );
        line(src, p11, p12, Scalar(R,G,B),2,8);
        line(src, p11, p21, Scalar(R,G,B),2,8);
        line(src, p22, p12, Scalar(R,G,B),2,8);
        line(src, p22, p21, Scalar(R,G,B),2,8);
        //    }
        //
        imshow("image", src);
        waitKey(10);
        //
        
        rc.push_back(p11);
        rc.push_back(p12);
        rc.push_back(p21);
        rc.push_back(p22);
        
        
        //    quad = Mat::zeros(x_size*((float)src1.cols/(float)src.cols), y_size*((float)src1.cols/(float)src.cols), CV_8UC3);
        //    Mat zquad = Mat::zeros(x_size, y_size, CV_8UC3);
        //    vector<cv::Point2f> corner,quad_pts;
        //    corner.push_back(Point(p11.x*((float)src1.cols/(float)src.cols),p11.y*((float)src1.cols/(float)src.cols)));
        //    corner.push_back(Point(p21.x*((float)src1.cols/(float)src.cols),p21.y*((float)src1.cols/(float)src.cols)));
        //    corner.push_back(Point(p12.x*((float)src1.cols/(float)src.cols),p12.y*((float)src1.cols/(float)src.cols)));
        //    corner.push_back(Point(p22.x*((float)src1.cols/(float)src.cols),p22.y*((float)src1.cols/(float)src.cols)));
        //
        //    quad_pts.push_back(cv::Point2f(0,0));
        //    quad_pts.push_back(cv::Point2f(line_size(p11,p21)*((float)src1.cols/(float)src.cols),0));
        //    quad_pts.push_back(cv::Point2f(0,line_size(p11,p12)*((float)src1.cols/(float)src.cols)));
        //    quad_pts.push_back(cv::Point2f(line_size(p12,p22)*((float)src1.cols/(float)src.cols),line_size(p21,p22)*((float)src1.cols/(float)src.cols)));
        //
        //    Mat transmtx = getPerspectiveTransform(corner, quad_pts);
        //
        //    warpPerspective(src1, quad, transmtx,quad.size());
        
        return 1;
    }
    
    
    int ls::warf1(Mat &src,Mat &src1,Mat &angle,LSEG &line1,LSEG &line2,vector<Point> &rc  ,int k)
    {
        Point2f v1,fv1,vt1,fvt1,v2,fv2,vt2,fvt2;
        cv::Point p1,p2,cp,tp1,tp2,p3,p4,tp3,tp4;
        cv::Point fp1_old,lp1_old;
        cv::Point fp2_old,lp2_old;
        cv::Point mid1,mid2;
        cv::Point u1,d1,u2,d2;
        
        u1 = line1[0];
        d1 = line1[1];
        u2 = line2[0];
        d2 = line2[1];
        
        mid1.x = (u1.x + d1.x)/2.0;
        mid1.y = (u1.y + d1.y)/2.0;
        
        mid2.x = (u2.x + d2.x)/2.0;
        mid2.y = (u2.y + d2.y)/2.0;
        
        
        double fl1 = line_size(u1,d1);
        double fl2 = line_size(u2,d2);
        float max = fl1 > fl2 ? fl1 : fl2;//最长直线
        
        v1.x = ((float)d1.x - (float)u1.x)/fl1;
        v1.y = ((float)d1.y - (float)u1.y)/fl1;
        vt1.x = -v1.y;
        vt1.y = v1.x;
        fv1.x = -v1.x;
        fv1.y = -v1.y;
        fvt1.x = -vt1.x;
        fvt1.y = -vt1.y;
        
        
        v2.x = ((float)d2.x - (float)u2.x)/fl2;
        v2.y = ((float)d2.y - (float)u2.y)/fl2;
        vt2.x = -v2.y;
        vt2.y = v2.x;
        fv2.x = -v2.x;
        fv2.y = -v2.y;
        fvt2.x = -vt2.x;
        fvt2.y = -vt2.y;
        
        fp1_old = prpoint(mid1, 50, v1);
        lp1_old = prpoint(mid1, 50, fv1);
        
        fp2_old = prpoint(mid2, 50, v2);
        lp2_old = prpoint(mid2, 50, fv2);
        
        
        p1 = fp1_old;
        p2 = lp1_old;
        p3 = fp2_old;
        p4 = lp2_old;
        
        //    cp.x = (line2[0].x + line2[1].x)/2.0;
        //    cp.y = (line2[0].y + line2[1].y)/2.0;
        
        float dd1 = abs(v2.y*p1.x - v2.x*p1.y + v2.x*p3.y -v2.y*p3.x);//p1到v2的距离
        float dd2 = abs(v2.y*p2.x - v2.x*p2.y + v2.x*p3.y -v2.y*p3.x);//p2到v2的距离
        float dd3 = abs(v1.y*p3.x - v1.x*p3.y + v1.x*p1.y -v1.y*p1.x);//p3到v1的距离
        float dd4 = abs(v1.y*p4.x - v1.x*p4.y + v1.x*p1.y -v1.y*p1.x);//p4到v1的距离
        //
        tp1 = prpoint(p1, dd1, vt2);
        tp2 = prpoint(p2, dd2, vt2);
        tp3 = prpoint(p3, dd3, fvt1);
        tp4 = prpoint(p4, dd4, fvt1);
        
        
        int a = point_line(p3, p4, tp1);
        int b = point_line(p3, p4, tp2);
        // int c = point_line(p1, p2, tp3);
        // int d = point_line(p1, p2, tp4);
        //int a = pers_point_inside(p3, p4, tp1);
        //int b = pers_point_inside(p3, p4, tp2);
        //
        /////////////////////////////////////////
        //        Mat oo;
        //        oo = Mat::zeros(src.rows, src.cols, CV_8UC1);
        //        line(oo, line1[0], line1[1], Scalar(255,255,255),3,8);
        //        line(oo, line2[0], line2[1], Scalar(255,255,255),3,8);
        //        circle(oo, tp1, 3, Scalar(255,255,255));
        //        circle(oo, tp2, 3, Scalar(255,255,255));
        //        circle(oo, tp3, 3, Scalar(255,255,255));
        //        circle(oo, tp4, 3, Scalar(255,255,255));
        //        circle(oo, Point(3,krows*0.7), 3, Scalar(255,255,255));
        //        line(oo, Point(3,krows*0.7), Point(600,krows*0.4), Scalar(255,255,255));
        //        imshow("image1", oo);
        //        waitKey(10);
        //////////////////////////////////////////////////////
        
        
        if(a && b  && a == b)//l1的投影完全落在l2外
        {
            return 0;
        }
        
        if(abs(acos(v1.x*v2.x+v1.y*v2.y)*(180.0/CV_PI)) > 8)//放弃相交程度过大的直线2.8,2.7 2.2
        {
            return 3;
        }
        
        Point P1,P2,P3,P4;
        P1 = a?tp3:p1;
        P2 = b?tp4:p2;
        P3 = a?p3:tp1;
        P4 = b?p4:tp2;
        
        Point p11,p12,p21,p22;
        float del1 = 0,del2 = 0;
        double e_min1=0,e_min2=0;
        bool active_d1 = true,active_d2 = true;//生长停止的标志，true表示继续生长，反之停止生长
        bool location_p1 = true,location_p2 = true,location_p3 = true,location_p4 = true;
        
        float min_x = 0,max_x = 0,min_y = 0,max_y = 0;
        while((active_d1 && location_p1 && location_p3) || (active_d2 && location_p2 && location_p4))
        {
            Mat image2 = src.clone();
            
            Point op1,op2,op3,op4;
            p11 = prpoint(P1, del1, v1);
            p12 = prpoint(P2, del2, fv1);
            p21 = prpoint(P3, del1, v2);
            p22 = prpoint(P4, del2, fv2);
            double len1 = line_size(p11,p21);
            double len2 = line_size(p12,p22);
            double len3 = line_size(p11,p12);
            double len4 = line_size(p21,p22);
            min_x = len1 < len2 ? len1 : len2;
            max_x = len1 > len2 ? len1 : len2;
            min_y = len3 < len4 ? len3 : len4;
            max_y = len3 > len4 ? len3 : len4;
            
            if(min_x < 7.0 || max_x > 130 || max_y > 4*max)/////////////
            {
                return 2;
            }
            //保证不出边界
            location_p1 = p11.x < angle.cols && p11.x > 0 && p11.y < angle.rows && p11.y > 0;
            location_p2 = p12.x < angle.cols && p12.x > 0 && p12.y < angle.rows && p12.y > 0;
            location_p3 = p21.x < angle.cols && p21.x > 0 && p21.y < angle.rows && p21.y > 0;
            location_p4 = p22.x < angle.cols && p22.x > 0 && p22.y < angle.rows && p22.y > 0;
            //计算两条直线的端点的能量值，一致性
            float power1 = power(src, angle, p11, p21);
            float power2 = power(src, angle, p12, p22);
            // cout<<"power1:"<<power1<<"power2:"<<power2<<endl;
            //向两边生长,第二个满足条件，大于一个阈值，表示停止生长，并且需要满足第三个条件
            cv::Point fp1 = prpoint(p11,5,v1);
            cv::Point lp1 = prpoint(p21,5,v2);
            ////////////
            circle(image2, p11, 3, Scalar(255,255,0));
            circle(image2, p21, 3, Scalar(255,255,0));
            circle(image2, fp1, 3, Scalar(255,255,0));
            circle(image2, lp1, 3, Scalar(255,255,0));
            circle(image2, p12, 3, Scalar(255,255,0));
            circle(image2, p22, 3, Scalar(255,255,0));
            /////////////////
            
            //            float zpower1_min = power(src, angle, p11, fp1) < power(src, angle, p21, lp1) ? power(src, angle, p11, fp1)
            //            : power(src, angle, p21, lp1);//第三个条件：取出能量最大值，小于一个阈值表示d不在增长
            float zpower1_max = power(src, angle, p11, fp1) > power(src, angle, p21, lp1) ? power(src, angle, p11, fp1)
            : power(src, angle, p21, lp1);
            //            float mean_power1 = 0.5 * (zpower1_min + zpower1_max);
            //float d_power1 =sqrt(0.5 * ((zpower1_max - mean_power1) * (zpower1_max - mean_power1) + (zpower1_min - mean_power1) * (zpower1_min - mean_power1))) ;
            // cout<<"mean_power1:"<<mean_power1<<"d_power1:"<<d_power1<<endl;
            
            //cout<<d_power1<<endl;
            //float zpower1 = 0.45 * zpower1_max + 0.4 * zpower1_min;//0.45  0.4
            float zpower1 = zpower1_max;
            
            //        cout<<"1:"<<power1<<"   "<<zpower1<<endl;
            //
            //
            //
            cv::Point fp2 = prpoint(p12,5,fv1);
            cv::Point lp2 = prpoint(p22,5,fv2);
            
            //            float zpower2_min = power(src, angle, p12, fp2) < power(src, angle, p22, lp2) ? power(src, angle, p12, fp2) : power(src, angle, p22, lp2);
            float zpower2_max = power(src, angle, p12, fp2) > power(src, angle, p22, lp2) ? power(src, angle, p12, fp2) : power(src, angle, p22, lp2);
            //            float mean_power2 = 0.5 * (zpower2_min + zpower2_max);
            //float d_power2 =sqrt(0.5 * ((zpower2_max - mean_power2) * (zpower2_max - mean_power2) + (zpower2_min - mean_power2) * (zpower2_min - mean_power2))) ;
            //cout<<"mean_power2:"<<mean_power2<<"d_power2:"<<d_power2<<endl;
            //cout<<"zpower1_max:"<<zpower1_max<<"zpower1_min:"<<zpower1_min<<endl;
            //cout<<"zpower2_max:"<<zpower2_max<<"zpower2_min:"<<zpower2_min<<endl;
            
            
            float zpower2 = zpower2_max;
            
            //            float tg1 = 0.80*mean_power1 +  0.20*mean_power2;//0.65  0.35
            
            if(active_d1 && location_p1 && location_p3)//上端或左端
            {
                if((power1 > 30  && zpower1 < 40))//|| abs(power1 - power2) < 10)//)//满足第二个条件和第三个条件，不再生长30,42
                {
                    active_d1 = false;
                    e_min1 = abs(power1);
                }
                else//否则继续生长
                {
                    del1 += 1;//每次增加1个像素
                    
                }
            }
            else
            {
            }
            
            //        cout<<"2:"<<power2<<"   "<<zpower2<<endl;
            //            float tg2 = 0.20*mean_power1 + 0.80*mean_power2;
            if(active_d2 && location_p2 && location_p4)
            {
                if((power2 > 30 && zpower2 < 40))//|| abs(power1 - power2) < 10)//右端和下端
                {
                    active_d2 = false;
                    e_min2 = abs(power2);
                }
                else
                {
                    del2 += 1;
                }
            }
            
            else
            {
                //           if(power2>60)
                //            {
                //                active_d2 = true;
                //            }
            }
            /////////////////////////////
            //            line(image2, p11, p12, Scalar(0,0,255),2,8);
            //            line(image2, p11, p21, Scalar(0,0,255),2,8);
            //            line(image2, p22, p12, Scalar(0,0,255),2,8);
            //            line(image2, p22, p21, Scalar(0,0,255),2,8);
            //            imshow("image", image2);
            //            waitKey(10);
            ///////////////////////////////
        }
        
        //int x_size = line_size(p11,p12)<line_size(p21,p22)?line_size(p21,p22):line_size(p11,p12);
        //int y_size = line_size(p11,p21)<line_size(p12,p22)?line_size(p12,p22):line_size(p11,p21);
        //    {
        //////////////////////////////////////
        //        int R = ( rand() % (int) ( 255 + 1 ) );
        //        int G = ( rand() % (int) ( 255 + 1 ) );
        //        int B = ( rand() % (int) ( 255 + 1 ) );
        //        line(src, p11, p12, Scalar(R,G,B),2,8);
        //        line(src, p11, p21, Scalar(R,G,B),2,8);
        //        line(src, p22, p12, Scalar(R,G,B),2,8);
        //        line(src, p22, p21, Scalar(R,G,B),2,8);
        //        //    }
        //        //
        //        imshow("image", src);
        //        waitKey(10);
        //////////////////////////////////////////
        
        rc.push_back(p11);
        rc.push_back(p12);
        rc.push_back(p21);
        rc.push_back(p22);
        
        
        //    quad = Mat::zeros(x_size*((float)src1.cols/(float)src.cols), y_size*((float)src1.cols/(float)src.cols), CV_8UC3);
        //    Mat zquad = Mat::zeros(x_size, y_size, CV_8UC3);
        //    vector<cv::Point2f> corner,quad_pts;
        //    corner.push_back(Point(p11.x*((float)src1.cols/(float)src.cols),p11.y*((float)src1.cols/(float)src.cols)));
        //    corner.push_back(Point(p21.x*((float)src1.cols/(float)src.cols),p21.y*((float)src1.cols/(float)src.cols)));
        //    corner.push_back(Point(p12.x*((float)src1.cols/(float)src.cols),p12.y*((float)src1.cols/(float)src.cols)));
        //    corner.push_back(Point(p22.x*((float)src1.cols/(float)src.cols),p22.y*((float)src1.cols/(float)src.cols)));
        //
        //    quad_pts.push_back(cv::Point2f(0,0));
        //    quad_pts.push_back(cv::Point2f(line_size(p11,p21)*((float)src1.cols/(float)src.cols),0));
        //    quad_pts.push_back(cv::Point2f(0,line_size(p11,p12)*((float)src1.cols/(float)src.cols)));
        //    quad_pts.push_back(cv::Point2f(line_size(p12,p22)*((float)src1.cols/(float)src.cols),line_size(p21,p22)*((float)src1.cols/(float)src.cols)));
        //
        //    Mat transmtx = getPerspectiveTransform(corner, quad_pts);
        //
        //    warpPerspective(src1, quad, transmtx,quad.size());
        
        return 1;
    }
    
    
    //矫正所有切割后得到的图片，利用warpPerspective进行透视变换
    void ls::warp_change(vector<cv::Point>& in_prs,Mat& src1,Mat& src,Mat& out,vector<cv::Point2f> &corner)
    {
        vector<cv::Point> outpoints;
        double d1 = line_size(in_prs[0],in_prs[1]);
        double d2 = line_size(in_prs[2],in_prs[3]);
        double d3 = line_size(in_prs[0],in_prs[2]);
        double d4 = line_size(in_prs[1],in_prs[3]);
        bool lx = d1 < d2;
        bool ly = d3 < d4;
        
        int x_size = lx?d2:d1;//取最大
        int y_size = ly?d4:d3;
        
        cv::Point2f v,vt,fv,fvt;
        
        if(lx&&ly)//矫正成平行四边形
        {
            v.x = (in_prs[0].x - in_prs[1].x)/d1;
            v.y = (in_prs[0].y - in_prs[1].y)/d1;
            vt.x = (in_prs[0].x - in_prs[2].x)/d3;
            vt.y = (in_prs[0].y - in_prs[2].y)/d3;
            cv::Point a = prpoint(in_prs[0], x_size, v);
            outpoints.push_back(in_prs[0]);
            outpoints.push_back(a);
            outpoints.push_back(prpoint(in_prs[0], y_size, vt));
            outpoints.push_back(prpoint(a, y_size, vt));
        }
        else if(lx&&!ly)
        {
            v.x = (in_prs[1].x - in_prs[0].x)/d1;
            v.y = (in_prs[1].y - in_prs[0].y)/d1;
            vt.x = (in_prs[1].x - in_prs[3].x)/d4;
            vt.y = (in_prs[1].y - in_prs[3].y)/d4;
            cv::Point a = prpoint(in_prs[1], x_size, v);
            outpoints.push_back(a);
            outpoints.push_back(in_prs[1]);
            
            outpoints.push_back(prpoint(a, y_size, vt));
            outpoints.push_back(prpoint(in_prs[1], y_size, vt));
        }
        else if(!lx&&ly)
        {
            v.x = (in_prs[2].x - in_prs[3].x)/d2;
            v.y = (in_prs[2].y - in_prs[3].y)/d2;
            vt.x = (in_prs[2].x - in_prs[0].x)/d3;
            vt.y = (in_prs[2].y - in_prs[0].y)/d3;
            cv::Point a = prpoint(in_prs[2], y_size, vt);
            outpoints.push_back(a);
            outpoints.push_back(prpoint(a, x_size, v));
            outpoints.push_back(in_prs[2]);
            outpoints.push_back(prpoint(in_prs[2], x_size, v));
        }
        else
        {
            v.x = (in_prs[3].x - in_prs[2].x)/d2;
            v.y = (in_prs[3].y - in_prs[2].y)/d2;
            vt.x = (in_prs[3].x - in_prs[1].x)/d4;
            vt.y = (in_prs[3].y - in_prs[1].y)/d4;
            cv::Point a = prpoint(in_prs[3], x_size, v);
            outpoints.push_back(prpoint(a, y_size, vt));
            outpoints.push_back(prpoint(in_prs[3], y_size, vt));
            outpoints.push_back(a);
            outpoints.push_back(in_prs[3]);
        }
        
        out = Mat(x_size*((float)src1.cols/(float)src.cols), y_size*((float)src1.cols/(float)src.cols), CV_8UC3,Scalar(125));
        //    out = Mat::zeros(x_size, y_size, CV_8UC3);
        vector<cv::Point2f>  quad_pts;
        //映射到原图上
        corner.push_back(cv::Point(outpoints[0].x*((float)src1.cols/(float)src.cols),outpoints[0].y*((float)src1.cols/(float)src.cols)));
        corner.push_back(cv::Point(outpoints[2].x*((float)src1.cols/(float)src.cols),outpoints[2].y*((float)src1.cols/(float)src.cols)));
        corner.push_back(cv::Point(outpoints[1].x*((float)src1.cols/(float)src.cols),outpoints[1].y*((float)src1.cols/(float)src.cols)));
        corner.push_back(cv::Point(outpoints[3].x*((float)src1.cols/(float)src.cols),outpoints[3].y*((float)src1.cols/(float)src.cols)));
        //以原点为顶点的矩形
        quad_pts.push_back(cv::Point2f(0,0));
        quad_pts.push_back(cv::Point2f(line_size(outpoints[1],outpoints[3])*((float)src1.cols/(float)src.cols),0));
        quad_pts.push_back(cv::Point2f(0,line_size(outpoints[0],outpoints[1])*((float)src1.cols/(float)src.cols)));
        quad_pts.push_back(cv::Point2f(line_size(outpoints[1],outpoints[3])*((float)src1.cols/(float)src.cols),
                                       line_size(outpoints[2],outpoints[3])*((float)src1.cols/(float)src.cols)));
        
        
        //    获取透视变换的变换矩阵
        Mat transmtx = getPerspectiveTransform(corner, quad_pts);
        
        warpPerspective(src1, out, transmtx,out.size());
    }
    
    
    
    
    float ls::area(Point &pt,Point &p1,Point &p2)//计算三个点组成的三角形的面积
    {
        Point2f v;
        double d = line_size(p1,p2);
        v.x = ((float)p1.x - (float)p2.x)/d;
        v.y = ((float)p1.y - (float)p2.y)/d;
        float dd = abs(v.y*pt.x - v.x*pt.y + v.x*p1.y -v.y*p1.x);
        return 0.5*dd*d;
    }
    
    
    bool ls::isInRectangle(Point& pt,vector<Point>& pr)
    {
        float S01,S12,S23,S30;
        S01 = area(pt, pr[0], pr[1]);
        S12 = area(pt, pr[1], pr[2]);
        S23 = area(pt, pr[2], pr[3]);
        S30 = area(pt, pr[3], pr[0]);
        float ss = S01 + S12 + S23 + S30;
        float s012,s123;
        // float s102,s302;
        s012 = area(pr[0], pr[1], pr[2]);
        s123 = area(pr[3], pr[1], pr[2]);
        //s102 = area(pr[1], pr[0], pr[2]);
        // s302 = area(pr[3], pr[0], pr[2]);
        if(abs(ss - s012-s123) < 100)
            //if ((s102 + s302) == ss)
            //float sr = line_size(pr[0], pr[1]) * line_size(pr[1], pr[2]);
            //if (ss == sr)
        {
            return true;
        }
        else
            return false;
    }
    
    
    Rect get_spine_pr(cv::Point )
    {
        Rect rect;
        return rect;
        
    }
    
    ///------------alg 3------------
    //求两个点的直线方程
    LINE ls::makeline(Point p1,Point p2) {
        LINE tl;
        tl.start = p1;
        tl.end = p2;
        int sign = 1;
        tl.a=p2.y-p1.y;
        if (tl.a<0) {
            sign = -1;
            tl.a=sign*tl.a;
        }
        tl.b=sign*(p1.x-p2.x);
        tl.c=sign*(p1.y*p2.x-p1.x*p2.y);
        return tl;
    }
    
    // 如果两条直线 l1(a1*x+b1*y+c1 = 0), l2(a2*x+b2*y+c2 = 0)相交，返回true，且返回交点p
    bool ls::lineintersect(LINE l1,LINE l2,Point &p)
    {
        double EP = 0.03;
        double d=l1.a*l2.b-l2.a*l1.b;
        if (abs(d)<EP) return false;// 不相交
        p.x = (l2.c*l1.b-l1.c*l2.b)/d;
        p.y = (l2.a*l1.c-l1.a*l2.c)/d;
        return true;
    }
    //计算叉积
    double multiply(Point sp,Point ep,Point op) {
        return ((sp.x-op.x)*(ep.y-op.y)-(ep.x-op.x)*(sp.y-op.y));
    }
    bool online(LINE l,Point p)
    {
        return ((((p.x-l.start.x)*(p.x-l.end.x)<=0)&&((p.y-l.start.y)*(p.y-l.end.y)<=0 )));
    }
    
    Point vector_g(Point a,Point b)
    {
        Point c;
        c.x = b.x - a.x;
        c.y = b.y - a.y;
        return c;
        
    }
    
    double cross_product(Point a,Point b)
    {
        double cp,d1,d2,m;
        double cosr;
        m=a.x*b.x+a.y*b.y;
        d1 = sqrt(a.x * a.x + a.y * a.y);
        d2 = sqrt(b.x * b.x + b.y * b.y);
        cosr = m/d1/d2;
        cp = d1 * d2 * sin(acos(cosr));
        return cp;
    }
    
    double area_vertice_intersection(vector<Point> points)
    {
        vector<int> hull;
        if(points.size() < 3)
            return 0;
        convexHull(points, hull, true);
        Point poi = points[hull[hull.size()-1]];
        vector<Point> vec;
        for(int i=0;i < hull.size();i++)
        {
            Point p_new = points[hull[i]];
            Point v;
            v = vector_g(poi, p_new);
            if(v.x != 0 || v.y != 0)
            {
                vec.push_back(v);
            }
            poi = p_new;
        }
        double sum = 0.0;
        int i = 0;
        while(i <= (int)vec.size()-2)
        {
            double css;
            css = cross_product(vec[i],vec[i+1]);
            sum = sum + css;
            i=i+2;
        }
        return sum/2;
    }
    
    
#if 0
    //确定重叠区域
    void ls::DecideOberlap(vector<Point> &pr1,vector<Point> &pr2,Size imgsize,int& k1,int & k2)
    {
        Mat img1 = Mat::zeros(imgsize.height,imgsize.width,CV_8UC1);
        Mat img2 = Mat::zeros(imgsize.height,imgsize.width,CV_8UC1);
        line(img1, pr1[0], pr1[1], Scalar(125),2,8);
        line(img1, pr1[0], pr1[2], Scalar(125),2,8);
        line(img1, pr1[3], pr1[1], Scalar(125),2,8);
        line(img1, pr1[3], pr1[2], Scalar(125),2,8);
        line(img2, pr2[0], pr2[1], Scalar(125),2,8);
        line(img2, pr2[0], pr2[2], Scalar(125),2,8);
        line(img2, pr2[3], pr2[1], Scalar(125),2,8);
        line(img2, pr2[3], pr2[2], Scalar(125),2,8);
        Point seed1,seed2;
        seed1 = Point((pr1[0].x+pr1[1].x+pr1[2].x+pr1[3].x)*0.25,(pr1[0].y+pr1[1].y+pr1[2].y+pr1[3].y)*0.25);
        seed2 = Point((pr2[0].x+pr2[1].x+pr2[2].x+pr2[3].x)*0.25,(pr2[0].y+pr2[1].y+pr2[2].y+pr2[3].y)*0.25);
        floodFill(img1, seed1, Scalar(125));
        floodFill(img2, seed2, Scalar(125));
        Mat img = img1 +img2;
        threshold(img, img, 125, 255, CV_THRESH_BINARY);
        vector<vector<Point> > contours;
        findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        if (contours.size()==0) {
            k1 = 0;k2 = 0;
        }
        else
        {
            // int x_size1 = line_size(pr1[0],pr1[1])<line_size(pr1[2],pr1[3])?line_size(pr1[2],pr1[3]):line_size(pr1[0],pr1[1]);
            int x_size1 = MAX(line_size(pr1[2],pr1[3]),line_size(pr1[0],pr1[1]));
            //int y_size1 = line_size(pr1[0],pr1[2])<line_size(pr1[1],pr1[3])?line_size(pr1[1],pr1[3]):line_size(pr1[0],pr1[2]);
            int y_size1 = MAX(line_size(pr1[1],pr1[3]), line_size(pr1[0],pr1[2]));
            //int x_size2 = line_size(pr2[0],pr2[1])<line_size(pr2[2],pr2[3])?line_size(pr2[2],pr2[3]):line_size(pr2[0],pr2[1]);
            //int y_size2 = line_size(pr2[0],pr2[2])<line_size(pr2[1],pr2[3])?line_size(pr2[1],pr2[3]):line_size(pr2[0],pr2[2]);
            int x_size2 = MAX(line_size(pr2[2],pr2[3]), line_size(pr2[0],pr2[1]));
            int y_size2 = MAX(line_size(pr2[1],pr2[3]), line_size(pr2[0],pr2[2]));
            float area,area1,area2;
            
            area = contourArea(contours[0]);
            area1 = x_size1*y_size1;
            area2 = x_size2*y_size2;
            float rate = 0.4;
            if (area/area1 >= rate && area/area2 >= rate)
            {
                k1 = 1;k2 =1;
            }
            else if(area/area1 >= rate && area/area2 <= rate)
            {k1 = 1;k2 = 0;}
            else if(area/area1 <= rate && area/area2 >= rate)
            {k1 = 0;k2 = 1;}
            else
            {k1 = 0;k2 = 0;}
        }
    }
#endif
    
    void ls::DecideOberlap(vector<Point> &pr1,vector<Point> &pr2,Size imgsize,int& k1,int & k2)
    {
        vector<cv::Point> p1,p2;
        p1.push_back(pr1[0]);
        p1.push_back(pr1[2]);
        p1.push_back(pr1[3]);
        p1.push_back(pr1[1]);
        p2.push_back(pr2[0]);
        p2.push_back(pr2[2]);
        p2.push_back(pr2[3]);
        p2.push_back(pr2[1]);
        vector<Point> points;
        for (int m = 0; m < 4; m++)
        {
            if(isInRectangle(p1[m], p2))
            {
                points.push_back(p1[m]);
            }
            if(isInRectangle(p2[m], p1))
            {
                points.push_back(p2[m]);
            }
            for(int n = 0;n < 4;n++)
            {
                Point pt;
                LINE l1,l2;
                l1 = makeline(p1[m%4], p1[(m+1)%4]);
                l2 = makeline(p2[n%4], p2[(n+1)%4]);
                if(lineintersect(l1,l2,pt))//互相交叉
                {
                    if(online(l1, pt)&&online(l2, pt))
                    {
                        points.push_back(pt);
                    }
                }
                else
                {//互相包含
                    if(online(l1, l2.start))
                        points.push_back(l2.start);
                    else if(online(l1, l2.end))
                        points.push_back(l2.end);
                    else if(online(l2, l1.start))
                        points.push_back(l1.start);
                    else if(online(l2, l1.end))
                        points.push_back(l1.end);
                    //                    if(online(l1, l2.start))
                    //                        points.push_back(l1.start);
                    //                    else if(online(l1, l2.end))
                    //                        points.push_back(l1.end);
                    //                    else if(online(l2, l1.start))
                    //                        points.push_back(l2.start);
                    //                    else if(online(l2, l1.end))
                    //                        points.push_back(l2.end);
                }
            }
        }
        double area = area_vertice_intersection(points);
        if (area == 0) {
            k1 = 0;k2 = 0;
        }
        else
        {
            double area1,area2;
            area1 = area_vertice_intersection(pr1);
            area2 = area_vertice_intersection(pr2);
            float rate = 0.4;
            double b1,b2;
            b1 = area/area1;
            b2 = area/area2;
            if((b1 >= rate || b2 >= rate) && b1 > b2)
            {
                k1 = 1;k2 = 0;
            }
            else if((b1 >= rate || b2 >= rate) && b2 > b1)
            {
                k1 = 0;k2 = 1;
            }
            else
            {k1 = 0;k2 = 0;}
        }
    }
    
    
#if 1
    void ls::DecideOberlap1(vector<cv::Point> &pr1,vector<cv::Point> &pr2,Size imgsize,int& k1,int & k2)
    {
        vector<cv::Point> p1,p2;
        p1.push_back(pr1[0]);//顺时针
        p1.push_back(pr1[2]);
        p1.push_back(pr1[3]);
        p1.push_back(pr1[1]);
        p2.push_back(pr2[0]);
        p2.push_back(pr2[2]);
        p2.push_back(pr2[3]);
        p2.push_back(pr2[1]);
        vector<cv::Point> points;
        //寻找两个pr相交区域的顶点，保存到points
        for (int m = 0; m < 4; m++)
        {
            if(isInRectangle(p1[m], p2))
            {
                points.push_back(p1[m]);
            }
            if(isInRectangle(p2[m], p1))
            {
                points.push_back(p2[m]);
            }
            for(int n = 0;n < 4;n++)
            {
                cv::Point pt;
                LINE l1,l2;
                l1 = makeline(p1[m%4], p1[(m+1)%4]);//生成线段
                l2 = makeline(p2[n%4], p2[(n+1)%4]);
                if(lineintersect(l1,l2,pt))//求交点
                {
                    if(online(l1, pt) && online(l2, pt))//判断交点是否在两条线段上
                    {
                        points.push_back(pt);
                    }
                }
                else
                {
                    if(online(l1, l2.start))
                        points.push_back(l2.start);
                    else if(online(l1, l2.end))
                        points.push_back(l2.end);
                    else if(online(l2, l1.start))
                        points.push_back(l1.start);
                    else if(online(l2, l1.end))
                        points.push_back(l1.end);
                }
            }
        }
        //    Mat img1 = Mat::zeros(imgsize.height,imgsize.width,CV_8UC1);
        //    Mat img2 = Mat::zeros(imgsize.height,imgsize.width,CV_8UC1);
        //    line(img1, pr1[0], pr1[1], Scalar(125),2,8);
        //    line(img1, pr1[0], pr1[2], Scalar(125),2,8);
        //    line(img1, pr1[3], pr1[1], Scalar(125),2,8);
        //    line(img1, pr1[3], pr1[2], Scalar(125),2,8);
        //    line(img1, pr2[0], pr2[1], Scalar(125),2,8);
        //    line(img1, pr2[0], pr2[2], Scalar(125),2,8);
        //    line(img1, pr2[3], pr2[1], Scalar(125),2,8);
        //    line(img1, pr2[3], pr2[2], Scalar(125),2,8);
        //    for (int i =0; i<points.size(); i++) {
        //        circle(img1, points[i], 10, Scalar(255));
        //    }
        //    imshow("img1", img1);
        //    waitKey(30);
        //计算相交区域面积
        double area = area_vertice_intersection(points);
        
        if (area == 0)
        {
            k1 = 0;
            k2 = 0;//互不包含
        }
        else
        {
            //int x_size1 = line_size(pr1[0],pr1[1]) < line_size(pr1[2],pr1[3]) ? line_size(pr1[2],pr1[3]) : line_size(pr1[0],pr1[1]);//求pr1的最长水平边
            // int y_size1 = line_size(pr1[0],pr1[2]) < line_size(pr1[1],pr1[3]) ? line_size(pr1[1],pr1[3]) : line_size(pr1[0],pr1[2]);//求pr1的最长竖直边
            //int x_size2 = line_size(pr2[0],pr2[1]) < line_size(pr2[2],pr2[3]) ? line_size(pr2[2],pr2[3]) : line_size(pr2[0],pr2[1]);//求pr2的最长水平边
            //int y_size2 = line_size(pr2[0],pr2[2]) < line_size(pr2[1],pr2[3]) ? line_size(pr2[1],pr2[3]) : line_size(pr2[0],pr2[2]);//求pr2的最长竖直边
            //vector<cv::Point> pr1_4,pr2_4;
            //pr1_4.push
            float area1,area2;
            //area1 = x_size1 * y_size1;
            //area2 = x_size2 * y_size2;
            area1 = area_vertice_intersection(pr1);
            area2 = area_vertice_intersection(pr2);
            float rate = 0.8;
            double b1,b2;
            b1 = area/area1;
            b2 = area/area2;
            if((b1 >= rate || b2 >= rate) && b1 > b2)
                //if (b1 > b2 && abs(b1 - 1) < 0.01)
            {
                k1 = -1;
                k2 = 1;//pr1在pr2内
            }
            else if((b1 >= rate || b2 >= rate) && b2 > b1)
                //else if (b1 < b2 && abs(b2 - 1) < 0.01)
            {
                k1 = 1;
                k2 = -1;//pr2在pr1内
            }
            //        else if(b1 >= rate && b2 <= rate)
            //        {k1 = -1;k2 = 1;}
            //        else if(b1 <= rate && b2 >= rate)
            //        {k1 = 1;k2 = -1;}
            else
            {
                k1 = -1;
                k2 = -1;
            }//相交部分区域
        }
    }
#endif
#if 0
    //计算重叠区域面积
    float ls::pr_area(Mat &angle,vector<Point>& pr,float& height)
    {
        int x_size = line_size(pr[0],pr[1])<line_size(pr[2],pr[3])?line_size(pr[2],pr[3]):line_size(pr[0],pr[1]);
        int y_size = line_size(pr[0],pr[2])<line_size(pr[1],pr[3])?line_size(pr[1],pr[3]):line_size(pr[0],pr[2]);
        //int x_size = MAX(line_size(pr[2],pr[3]), line_size(pr[0],pr[1]));
        //int y_size = MAX(line_size(pr[1],pr[3]), line_size(pr[0],pr[2]));
        height = x_size < y_size ? y_size : x_size;
        Mat src;double a,b;
        if(x_size > y_size)
        {
            a = 180 - power(src, angle, pr[0], pr[1]);
            b = 180 - power(src, angle, pr[2], pr[3]);
        }
        else
        {
            a = 180 - power(src, angle, pr[0], pr[2]);
            b = 180 - power(src, angle, pr[1], pr[2]);
        }
        
        return height*(1 - (a+b)/(2*30));
    }
#endif
    
    
    int ls ::is_parallel(LSEG l1,LSEG l2)
    {
        Point2d v1,v2;
        v1.x = (l1[0].x - l1[1].x)/line_size(l1[0], l1[1]);
        v1.y = (l1[0].y - l1[1].y)/line_size(l1[0], l1[1]);
        
        v2.x = (l2[0].x - l2[1].x)/line_size(l2[0], l2[1]);
        v2.y = (l2[0].y - l2[1].y)/line_size(l2[0], l2[1]);
        double dd;
        dd = v1.y * v2.x - v2.y * v1.x;
        if (abs(dd) < 0.03) return 1;
        else return 0;
        
        
    }
    
    
    
    
    vector<vector<LSEG> > ls::intersect_trunk(vector<LSEG> &oolines,int count)
    {
        vector<LSEG> oolines_rest,res_tmp;
        vector<vector<LSEG> > oolines_same;//oolines_one;//,lines_tmp;
        
        
        if (1 == count)
        {
            lines_same.push_back(oolines);
            return lines_same;
        }
        else
        {
            for (int i = 0; i <(count - 1); i ++)
            {
                int flag = is_parallel(oolines[i], oolines[i + 1]);
                if (flag)
                {
                    res_tmp.push_back(oolines[i]);
                    if (i == (oolines.size() - 2))
                    {
                        res_tmp.push_back(oolines[i + 1]);
                        lines_same.push_back(res_tmp);
                        return lines_same;
                    }
                }
                else
                {
                    res_tmp.push_back(oolines[i]);
                    lines_same.push_back(res_tmp);
                    res_tmp.clear();
                    if (i == (oolines.size() - 1))
                    {
                        
                        return lines_same;
                    }
                    else
                    {
                        for (int oo = i + 1; oo < oolines.size(); oo ++)
                        {
                            oolines_rest.push_back(oolines[oo]);
                            
                        }
                        i = 0;
                        lines_same.push_back(oolines_same[i]);
                        oolines_same[i].clear();
                        
                        return intersect_trunk(oolines_rest,(int)oolines_rest.size());
                        
                        
                    }
                    
                }
                
            }
            
        }
        return lines_same;
        
        
    }
    
    void ls::getlines2(vector<LINE> &ls2,vector<LSEG> les,vector<int> &k)
    {
        
        for (int jj = 0; jj < ls2.size(); jj ++)
        {
            for (int kk = jj+1; kk < ls2.size() && kk < jj + 4 ; kk ++)
            {
                cv::Point inter;
                bool fag = lineintersect(ls2[jj], ls2[kk], inter);
                bool is1 = online(ls2[jj], ls2[kk].end);
                //            bool is2 = online(ls2[kk], ls2[jj].end);
                if (!fag && is1)
                {
                    LSEG tmp;
                    tmp.push_back(ls2[jj].start);
                    tmp.push_back(ls2[kk].end);
                    les.push_back(tmp);
                    k.push_back(jj);
                    
                    
                }
                
                
            }
            
        }
    }
    
    void ls::exchange(vector<LSEG> &l,Mat & src)
    {
        for (int i = 0; i < l.size() - 1; i ++)
        {
            Point2d middle,v1,v2;
            middle.x = (l[i][0].x + l[i][1].x)/2;
            middle.y = (l[i][0].y + l[i][1].y)/2;
            v1.x = l[i+1][0].x - middle.x;
            v1.y = l[i+1][0].y - middle.y;
            v2.x = l[i+1][1].x - middle.x;
            v2.y = l[i+1][1].y - middle.y;
            double s1 = line_size(l[i][0], l[i][1]);
            double s2 = line_size(l[i+1][0], l[i+1][1]);
            int f = is_parallel(l[i], l[i+1]);
            //int j = i + 1;
            if (!f && v1.x > 0 && v1.y > 0 && v2.x > 0 && v2.y > 0 && (s1 - s2) > 30)
            {
                line(src, l[i][0], l[i][1], Scalar(0,0,255));
                line(src, l[i+1][0], l[i+1][1], Scalar(255,0,0));
                LSEG iTemp;
                iTemp = l[i];
                l[i] = l[i+1];
                l[i+1] = iTemp;
                cout<<"i: "<<i<<endl;
                
            }
            line(src, l[i][0], l[i][1], Scalar(255,0,0));
            line(src, l[i+1][0], l[i+1][1], Scalar(0,255,0));
            
        }
        // imshow("exchange", src);
        
        
    }
    
    
#define alpha  2
    
    double mwisComputeObjFuncX( double* w, double* A, int n, unsigned char* x)
    {
        int i,j;
        //int alpha = 2;
        double goal = 0;
        double val = 0;
        
        for(i=0;i<n;i++)
            for(j=0;j<n;j++)
                val += A[i*n+j]*x[i]*x[j];
        goal = - 0.5*alpha* val;
        
        for(i=0;i<n;i++)
            goal += w[i]*x[i];
        
        return goal;
    }
    
    double mwisComputeObjFuncY( double* w, double* A, int n, double* y)
    {
        int i,j;
        double goal = 0,val = 0;
        
        for(i=0;i<n;i++)
            for(j=0;j<n;j++)
                val += A[i*n+j]*y[i]*y[j];
        goal = - 0.5*alpha* val;
        
        for(i=0;i<n;i++)
            goal += w[i]*y[i];
        
        return goal;
    }
    
    double mwisComputeConvexObjFuncY( double* w, double c, int n, double* y)
    {
        int i;
        double goal = 0,val = 0;
        
        for(i=0;i<n;i++)
            val += y[i]*y[i];
        goal = - 0.5*c*alpha* val;
        
        for(i=0;i<n;i++)
            goal += w[i]*y[i];
        
        return goal;
    }
    
    double mwisComputeConvexObjFuncX( double* w, double c, int n, unsigned char* x)
    {
        int i;
        double goal = 0,val = 0;
        
        for(i=0;i<n;i++)
            val += x[i]*x[i];
        goal = - 0.5*c*alpha* val;
        
        for(i=0;i<n;i++)
            goal += w[i]*x[i];
        
        return goal;
    }
    
    double mwisConstraint(double* A, int n, double* y)
    {
        int i,j;
        double val = 0;
        
        for(i=0;i<n;i++)
            for(j=0;j<n;j++)
                val += A[i*n+j]*y[i]*y[j];
        return val;
    }
    
    /* MWIS based on integer fixed point */
    unsigned char* mwisGetX1( double* w, double* A, int n, unsigned char* x)
    {
        int i,j = 0,maxIter = 1000;
        double maxw = -1e20,minw = 1e20,rangew;
        double norm2 = 1e20;
        double threshold = 1e-16;
        double eta;
        double c,d,val,maxval;
        int maxi;
        unsigned char flag;
        
        double objf,objftemp;
        
        double* z = (double*)malloc(n*sizeof(double));
        double* y = (double*)malloc(n*sizeof(double));
        unsigned char* xtilde = (unsigned char*)malloc(n*sizeof(unsigned char));
        
        /* allocate memory if needed */
        if(x<=NULL) x = (unsigned char*)malloc(n*sizeof(unsigned char));
        
        /* init y,x and rescale w */
        for(i=0;i<n;i++)
        {
            if(maxw < w[i]){ maxw = w[i];j=i;}
            if(minw > w[i]) minw = w[i];
        }
        rangew = maxw - minw;
        srand((unsigned int)time(0));
        for(i=0;i<n;i++)
        {
            w[i] = (w[i]-minw)/rangew;
            y[i] = /*w[i]*(rand()%2);*/0.5*w[i] + 0.01;/**/
        }
        
        memset(x,0,n*sizeof(unsigned char));
        x[j] = 1;
        //memset(y,0,n*sizeof(MAT_TYPE));
        //y[j] = 1;
        
        /* compute objective function */
        objf = mwisComputeObjFuncY( w, A, n, y);
        printf("objf: %f\n",objf);
        
        /* iterate */
        for(;(maxIter--) &&  norm2 >threshold; )
        {
            //printf("\niter: %d\n",maxIter);
            /* find local solution xtilde */
            maxval = -1e20;
            maxi = -1;
            flag = 0;
            for(i=0;i<n;i++)
            {
                val = 0;
                for(j=0;j<n;j++)
                {
                    val += A[i*n+j]*y[j];
                }
                val = w[i] - alpha*val;
                xtilde[i] = (val>=0);
                flag |= xtilde[i];
                
                if(maxval<val)
                {
                    maxval = val;
                    maxi = i;
                }
            }
            
            if(!flag) xtilde[maxi] = 1;
            
            /* compute objective function with xtilde */
            objftemp = mwisComputeObjFuncX( w, A, n, xtilde);
            
            /* check if objective function increases */
            if(objftemp>=objf)
            {
                /* compute norm */
                norm2 = 0;
                for(i=0;i<n;i++) norm2 += (y[i] - xtilde[i])*(y[i] - xtilde[i]);
                
                /* update y and x */
                for(i=0;i<n;i++)
                {
                    //if(xtilde[i])
                    //	printf("i: %d y: %lf xtilde: %d x: %d\n",i,y[i],xtilde[i],x[i]);
                    y[i] = xtilde[i];
                }
                memcpy(x,xtilde,n*sizeof(unsigned char));
                objf = objftemp;
            }
            else
            {
                /* compute z = xtilde - y */
                for(i=0;i<n;i++) z[i] = xtilde[i] - y[i];
                
                /* compute eta */
                c = d = 0;
                
                /* (w - alpha*Ay)z */
                for(i=0;i<n;i++)
                {
                    val = 0;
                    for(j=0;j<n;j++)
                    {
                        val += A[i*n+j]*y[j];
                    }
                    c += (w[i] - alpha*val)*z[i];
                }
                
                /* zAz */
                for(i=0;i<n;i++)
                    for(j=0;j<n;j++)
                        d += A[i*n+j]*z[i]*z[j];
                
                eta = d>0 ? MAX(c/(alpha*d),0) : MIN(c/(alpha*d),1);
                
                /* compute norm */
                norm2 = 0;
                for(i=0;i<n;i++) norm2 += eta*z[i]*eta*z[i];
                
                /* compute y */
                for(i=0;i<n;i++) y[i] += eta*z[i];
                
                /* compute objective function with y */
                objf = mwisComputeObjFuncY( w, A, n, y);
            }
            //printf("objf: %f\n",objf);
        }
        
        printf("objf: %f\n",objf);
        
        /* free memory */
        free(z);
        free(y);
        free(xtilde);
        
        return x;
    }
    
    /* MWIS based on gradient descent */
    unsigned char* mwisGetX2( double* w, double* A, int n, unsigned char* x)
    {
        int i,j = 0,maxIter = 1000;
        double maxw = -1e20,minw = 1e20,rangew;
        double norm2 = 1e20;
        double threshold = 1e-16;
        double eta;
        double c,d,val,dt = 0.0;
        double objf,objftemp,objfX;
        
        double* ytemp;
        double* y = (double*)malloc(n*sizeof(double));
        double* dy = (double*)malloc(n*sizeof(double));
        double* ynext = (double*)malloc(n*sizeof(double));
        double* z = (double*)malloc(n*sizeof(double));
        
        /* allocate memory if needed */
        if(x<=NULL) x = (unsigned char*)malloc(n*sizeof(unsigned char));
        
        /* init y,x, and rescale w */
        for(i=0;i<n;i++) //找出最小最大权重
        {
            if(maxw < w[i]){ maxw = w[i];j=i;}
            if(minw > w[i]) minw = w[i];
        }
        rangew = maxw - minw;//差
        srand((unsigned int)time(0));
        for(i=0;i<n;i++)
        {
            w[i] = (w[i]-minw)/rangew;//将权值归一化（0，1）
            y[i] = w[i]*(rand()%2)/**/;/**//*0.5*w[i] + 0.01;*///0或1
        }
        memset(x,0,n*sizeof(unsigned char));
        x[j] = 1;
        
        /* compute objective function */
        objf = mwisComputeObjFuncY( w, A, n, y);
        printf("objf: %f\n",objf);
        
        /* iterate */
        for( ;(maxIter--) && norm2 >threshold; )
        {
            /* find gradient dy */
            for(i=0;i<n;i++)
            {
                val = 0;
                for(j=0;j<n;j++)
                {
                    val += A[i*n+j]*y[j];
                }
                dy[i] = w[i] - alpha*val;
            }
            
            /* find time step dt */
            dt = 0;
            for(i=0;i<n;i++) dt+= dy[i]*dy[i];
            val = 0;
            for(i=0;i<n;i++)
                for(j=0;j<n;j++)
                    val += A[i*n+j]*dy[i]*dy[j];
            if(val)
                dt = dt/(alpha*fabs(val));
            
            /* find local solution ynext */
            for(i=0;i<n;i++)
            {
                ynext[i] = y[i] + dt*dy[i];
                if(ynext[i]>1) ynext[i] = 1;
                if(ynext[i]<0) ynext[i] = 0;
            }
            
            /* update x and compute objective function with x */
            for(i=0;i<n;i++)x[i] = (ynext[i]>1e-16);
            objfX = mwisComputeObjFuncX( w, A, n, x);
            
            /* compute objective function with ynext */
            objftemp = mwisComputeObjFuncY( w, A, n, ynext);
            
            if(objfX>objftemp)
            {
                for(i=0;i<n;i++) ynext[i] = x[i];
                objftemp = objfX;
            }
            
            if(objftemp<objf)
            {
                /* compute z = ynext - y */
                for(i=0;i<n;i++) z[i] = ynext[i] - y[i];
                
                c = d = 0;
                
                /* (w - alpha*Ay)z */
                for(i=0;i<n;i++)
                    c += dy[i]*z[i];
                
                /* zAz */
                for(i=0;i<n;i++)
                    for(j=0;j<n;j++)
                        d += A[i*n+j]*z[i]*z[j];
                
                /* compute eta */
                eta = d>0? MAX(c/(alpha*d),0):MIN(c/(alpha*d),1);
                
                /* compute ynext */
                for(i=0;i<n;i++) ynext[i] = y[i] + eta*z[i];
                
                /* compute objective function with ynext */
                objftemp = mwisComputeObjFuncY( w, A, n, ynext);
            }
            
            /* compute norm */
            norm2 = 0;
            /*for(i=0;i<n;i++) norm2 += (y[i] - ynext[i])*(y[i] - ynext[i]);
             norm2/=n;*/
            for(i=0;i<n;i++) norm2 = max(norm2,fabs(y[i] - ynext[i]));
            
            /* info */
            //printf("objf: %f norm2: %g dt: %g iter: %d yAy: %g\n",objf,norm2,dt,maxIter,mwisConstraint(A, n, y));
            
            /* update for next iteration */
            objf = objftemp;
            ytemp = y;
            y = ynext;
            ynext = ytemp;
        }
        
        /* update x */
        for(i=0;i<n;i++)x[i] = (y[i]>=1e-16);
        
        /* info */
        printf("objf: %f norm2: %g dt: %g iter: %d yAy: %g\n",objf,norm2,dt,maxIter,mwisConstraint(A, n, y));
        
        /* free memory */
        free(y);
        free(ynext);
        free(dy);
        free(z);
        
        return x;
    }
    
    unsigned char* mwisGetX( double* w, double* A, int n, unsigned char* x)
    {
        unsigned char option = 1;
        
        switch(option)
        {
            default:
                return mwisGetX2( w, A, n, x);
        }
    }
    
    
    
    
    
    //****************************************
    //************************************
#if 0
    vector<vector<LSEG> > clines;
    vector<LSEG> zlines;
    QuickSort(liness, (int)liness.size(), -1);
    for(int i = 0; i < liness.size(); i++) {
        if(zlines.size() == 0)
        {
            zlines.push_back(liness[i]);
        }
        else if (i == liness.size()-1 && line_jiao(liness[i][0], liness[i][1])-line_jiao(zlines[zlines.size()-1][0],zlines[zlines.size()-1][1])>2)
        {
            clines.push_back(zlines);
            zlines.clear();
            zlines.push_back(liness[i]);
            clines.push_back(zlines);
        }
        else if(i == liness.size()-1 && line_jiao(liness[i][0], liness[i][1])-line_jiao(zlines[zlines.size()-1][0],zlines[zlines.size()-1][1])<2)
        {
            zlines.push_back(liness[i]);
            clines.push_back(zlines);
        }
        else if (line_jiao(liness[i][0], liness[i][1])-line_jiao(zlines[zlines.size()-1][0],zlines[zlines.size()-1][1])>2) {
            clines.push_back(zlines);
            zlines.clear();
            zlines.push_back(liness[i]);
        }
        else
        {
            zlines.push_back(liness[i]);
        }
    }
    
    for (int i = 0; i < clines.size(); i++) {
        vector<vector<LSEG> > clines1;
        vector<LSEG> outlines,zlines1;
        Mat a = Mat::zeros(image.rows, image.cols, CV_8UC1);
        aa = image.clone();
        for(int j = 0;j<clines[i].size();j++)
        {
            line(a, clines[i][j][0], clines[i][j][1], Scalar(255,255,255),2,8);
            
        }
        imshow("a", a);
        findline(a, angle, outlines, 0);
        QuickSort(outlines, (int)outlines.size(), 1);
        for(int j = 0;j < outlines.size();j++)
        {
            line(aa, outlines[j][0], outlines[j][1], Scalar(255,255,255),2,8);
            if(zlines1.size() == 0)
            {
                zlines1.push_back(outlines[j]);
            }
            else if (j == outlines.size()-1 && (ca(outlines[j],1)-ca(zlines1[zlines1.size()-1],1) > 3 || abs(line_jiao(outlines[j][0], outlines[j][1])-line_jiao(zlines1[zlines1.size()-1][0],zlines1[zlines1.size()-1][1])) > 1))
            {
                clines1.push_back(zlines1);
                zlines1.clear();
                zlines1.push_back(outlines[j]);
                clines1.push_back(zlines1);
            }
            else if(j == outlines.size()-1 && ca(outlines[j],1)-ca(zlines1[zlines1.size()-1],1) <= 3 && abs(line_jiao(outlines[j][0], outlines[j][1])-line_jiao(zlines1[zlines1.size()-1][0],zlines1[zlines1.size()-1][1])) <= 1)
            {
                zlines1.push_back(outlines[j]);
                clines1.push_back(zlines1);
            }
            else if (ca(outlines[j], 1)-ca(zlines1[zlines1.size()-1],1)>3 || abs(line_jiao(outlines[j][0], outlines[j][1])-line_jiao(zlines1[zlines1.size()-1][0],zlines1[zlines1.size()-1][1])) > 1) {
                clines1.push_back(zlines1);
                zlines1.clear();
                zlines1.push_back(outlines[j]);
            }
            else
            {
                zlines1.push_back(outlines[j]);
            }
        }
        imshow("aaaa", aa);
        aa = image.clone();
        for(int m = 0;m < clines1.size();m++)
        {
            if(clines1[m].size() == 1)
            {
                line(aa, clines1[m][0][0], clines1[m][0][1], Scalar(255,255,255),2,8);
            }
            else
            {
                vector<Point> pts;
                for (int n = 0; n < clines1[m].size(); n++) {
                    pts.push_back(clines1[m][n][0]);
                    pts.push_back(clines1[m][n][1]);
                }
                BubbleSort1(pts, (int)pts.size());
                line(aa, pts[0], pts[(int)pts.size()-1], Scalar(255,255,255),2,8);
            }
        }
        imshow("a",aa);
        waitKey(0);
    }
    
#endif
    
    //****************************************
    //************************************
#if 1
    //检测书脊矩形框，并能量生长
    bool ls::pr_detect(Mat &src,Mat &src1,Mat &angle,vector<LSEG> &oolines,vector<vector<Point> > &out_result,vector<vector<Point> > &rest,int k,
                       LSEG &baseline,vector<cv::Mat> &book_cordinates)
    {
        vector<LSEG> result;
        float thresh_low = 2.0;
        float tg1 = 0.0;
        float tg2 = 0.0;
        //float thresh_high = 7.5;
        Mat cordinates_tmp(4,2,CV_32FC1);
        
        if (oolines.size()>2)
        {
            QuickSort(oolines, (int)oolines.size(),k,baseline);//按照书籍排列方向进行相应排序，快速排序
            //        for (int kk = 0; kk < oolines.size(); kk++)
            //        {
            //            //LINE l = makeline(oolines[kk][0], oolines[kk][1]);
            //            //cout<<l.a<<","<<l.b<<","<<l.c<<","<<l.start.x<<","<<l.start.y<<","<<l.end.x<<","<<l.end.y<<endl;
            //            line(src, oolines[kk][0], oolines[kk][1], Scalar(255,0,0));
            //            circle(src, oolines[kk][0], 3, Scalar(0,255,0));
            //            circle(src, oolines[kk][1], 3, Scalar(0,0,255));
            //            imshow("sort", src);
            //            waitKey(0);
            //
            //       }
            //
            //        vector<LINE> l;
            //        for (int mm = 0; mm < oolines.size(); mm ++)
            //        {
            //            l.push_back(makeline(oolines[mm][0], oolines[mm][1]));
            //        }
            //        vector<LSEG> les;
            //        vector<int> index;
            //        getlines2(l, les,index);
            //        if (les.size())
            //        {
            //            for (int nn = 0; nn < oolines.size() - 2; nn ++)
            //            {
            //                for (int ll = 0; ll < index.size(); ll ++)
            //                {
            //                    if (nn == index[ll])
            //                    {
            //                        oolines[nn] = les[nn];
            //                        oolines[nn + 1] = oolines[nn + 2];
            //                    }
            //                }
            //            }
            //        }
            //
            ////
            //        for (int kk = 0; kk < oolines.size(); kk++)
            //        {
            //            line(src, oolines[kk][0], oolines[kk][1], Scalar(255,0,0));
            //            circle(src, oolines[kk][0], 3, Scalar(0,255,0));
            //            circle(src, oolines[kk][1], 3, Scalar(0,0,255));
            //            imshow("connect", src);
            //            waitKey(0);
            //
            //        }
            ////
            
            int i=0,j=i+1;
            //int count = 0;
            while(i < oolines.size()-1)//i从0开始遍历
                //for (int i = 0; i < oolines.size() - 1; i ++)
            {
                // Mat quad;
                vector<cv::Point> rc;
                //根据能量函数确定最终的书脊矩形框，返回1表示找到一个框
                int bl = warf(src, src1, angle, oolines[i], oolines[j], rc,k,tg1,tg2);
                //cout<<bl<<endl;
                if(1 == bl)
                {
                    int x_size = line_size(rc[0],rc[1])<line_size(rc[2],rc[3])?line_size(rc[2],rc[3]):line_size(rc[0],rc[1]);
                    int y_size = line_size(rc[0],rc[2])<line_size(rc[1],rc[3])?line_size(rc[1],rc[3]):line_size(rc[0],rc[2]);
                    int l_size = x_size < y_size ? y_size : x_size;//max最长边
                    int s_size = x_size < y_size ? x_size : y_size;//min最短边
                    if((float)l_size/(float)s_size > thresh_low)//比值大于阈值，则认为这四个点是一本书籍
                    {
                        //                    if ((float)l_size/(float)s_size > thresh_high)
                        //                    {
                        result.push_back(rc);
                        //                    }
                        //                    else
                        //                    {
                        //                        //float thresh_mid = (thresh_low + thresh_high)/2;
                        //                        if ((float)l_size/(float)s_size < 6.3)//6
                        //                        {
                        //                            result.push_back(rc);
                        //                        }
                        //                        else
                        //
                        //                            rest.push_back(rc);
                        //                    }
                        //
                        
                        //                        int R = ( rand() % (int) ( 255 + 1 ) );
                        //                        int G = ( rand() % (int) ( 255 + 1 ) );
                        //                        int B = ( rand() % (int) ( 255 + 1 ) );
                        //                        line(src, rc[0], rc[1], Scalar(R,G,B),2,8);
                        //                        line(src, rc[2], rc[0], Scalar(R,G,B),2,8);
                        //                        line(src, rc[1], rc[3], Scalar(R,G,B),2,8);
                        //                        line(src, rc[2], rc[3], Scalar(R,G,B),2,8);
                        //
                        //
                        //                 imshow("image", src);
                        //waitKey(10);
                        i = i + 1;
                        j = i + 1;
                        
                    }
                    else//否则下一个
                    {
                        rest.push_back(rc);
                        j = j + 1;
                        
                    }
                }
                else if(3 == bl|| 0 == bl || 2 == bl)//下一个
                {
                    j = j + 1;
                }
                if(j == oolines.size())//下一次遍历
                {
                    i = i + 1;
                    j = i + 1;
                }
            }
            
            
            if(0 == result.size())
                return false;//没找到书籍，为空
            
            for (int i = 0; i < result.size(); i ++)
            {
                out_result.push_back(result[i]);
            }
#if 0
            else
            {
                int ok1 = 0,ok2 = 0;
                double *a = new double [((int)result.size())*((int)result.size())];
                double *w = new double [((int)result.size())];
                unsigned char *x = new unsigned char [((uchar)result.size())];
                float tg = 0.5*(tg1 + tg2);
                cout<<"tg:"<<tg<<endl;
                for(int n = 0;n<(int)result.size();n++)
                {
                    a[n] = 0;
                    x[n] = 0;
                    w[n] = power1(result[n], src, angle, tg);
                }
                
                for (int m = 0; m < result.size(); m++) {
                    for (int n = m; n < result.size(); n++) {
                        if(m != n)
                        {
                            DecideOberlap1(result[m],result[n],Size(angle.cols,angle.rows),ok1,ok2);
                            a[m*((int)result.size())+n] = ok1;
                            a[n*((int)result.size())+m] = ok2;
                        }
                        else if(m == n)
                        {
                            a[m*((int)result.size())+n] = 0;
                        }
                    }
                }
                
                for(int i = 0;i<(int)result.size()*(int)result.size();i++)
                {
                    
                }
                //x[0] = 1;
                x = mwisGetX(w, a, (int)result.size(), x);
                
                for (int i = 0; i < (int)result.size(); i ++) {
                    if(x[i] == 1) out_result.push_back(result[i]);
                }
                
                free(a);
                return 1;
            }
            //}
#endif
            
#if 0
            else
            {
                //            for (int m = 0; m < result.size(); m ++)
                //            {
                //
                //                out_result.push_back(result[m]);
                //                if (result.size() >= 2)
                //                {
                //                    //if (k == 0 || k == 2 || k == 3)
                //                    //{
                //                        //float max_y = result[0][0].y > result[result.size() - 1][2].y ? result[0][0].y : result[result.size() - 1][2].y;
                //                        float min_y = result[0][0].y < result[result.size() - 1][2].y ? result[0][0].y : result[result.size() - 1][2].y;
                //                       // float min_y1 = result[0][1].y < result[result.size() - 1][2].y ? result[0][0].y : result[result.size() - 1][2].y;
                //                   // float max_y1 = result[0][1].y > result[result.size() - 1][2].y ? result[0][0].y : result[result.size() - 1][2].y;
                //                        cordinates_tmp.at<float>(0, 0) = result[0][0].x;
                //                    cordinates_tmp.at<float>(0, 1) = min_y;//result[0][0].y;
                //                        cordinates_tmp.at<float>(1, 0) = result[0][1].x;
                //                        cordinates_tmp.at<float>(1, 1) = result[0][1].y;
                //                        cordinates_tmp.at<float>(2, 0) = result[result.size() - 1][2].x;
                //                    cordinates_tmp.at<float>(2, 1) = min_y;//result[result.size() - 1][2].y;
                //                        cordinates_tmp.at<float>(3, 0) = result[result.size() - 1][3].x;
                //                        cordinates_tmp.at<float>(3, 1) = result[result.size() - 1][3].y;
                //
                //                   // }
                //                }
                //                else
                //                {
                //
                //                    cordinates_tmp.at<float>(0, 0) = result[m][0].x;
                //                    cordinates_tmp.at<float>(0, 1) = result[m][0].y;
                //                    cordinates_tmp.at<float>(1, 0) = result[m][1].x;
                //                    cordinates_tmp.at<float>(1, 1) = result[m][1].y;
                //                    cordinates_tmp.at<float>(2, 0) = result[m][2].x;
                //                    cordinates_tmp.at<float>(2, 1) = result[m][2].y;
                //                    cordinates_tmp.at<float>(3, 0) = result[m][3].x;
                //                    cordinates_tmp.at<float>(3, 1) = result[m][3].y;
                //
                //
                //                //cordinates_tmp.free();
                //                //}
                //                book_cordinates.push_back(cordinates_tmp);
                int ok1 = 0;int ok2 = 0;
                int **G;
                G = new int *[(int)result.size()];
                for(int i =0;i<result.size();i++)
                {
                    G[i] = new int[(int)result.size()];
                }
                
                for(int i =0;i<result.size();i++)
                {
                    for(int j =0;j < result.size();j++)
                    {
                        G[i][j] = 0;
                    }
                }
                
                for(int i =0;i<result.size();i++)
                {
                    for (int j=0; j<result.size(); j++) {
                        
                        //                        if(i == j)
                        //                        {
                        //                            G[i][j] = 0;
                        //                        }
                        //                        else
                        //                        {
                        DecideOberlap(result[i],result[j],Size(angle.cols,angle.rows),ok1,ok2);
                        G[i][j] = ok1;
                        G[j][i] = ok2;
                        //                        }
                    }
                }
                
                //            vector<int> flag;
                for(int i = 0;i<result.size();i++)
                {
                    bool isleaf = true;
                    for (int j = 0; j<result.size(); j++)
                    {
                        if(G[i][j] != 0)
                        {
                            isleaf = false;
                        }
                    }
                    if(isleaf) out_result.push_back(result[i]);
                    //                    else
                    //                        out_result.push_back(result[k]);
                }
                return 1;
            }
            //            }
            
#endif
            
        }
        
        return 0;
    }
#endif
#if 1
    //检测书脊矩形框，并能量生长
    bool ls::pr_detect1(Mat &src,Mat &src1,Mat &angle,vector<LSEG> &oolines,vector<vector<Point> > &out_result,int k)
    {
        vector<LSEG> result;
        float thresh_low = 2.0;
        Mat cordinates_tmp(4,2,CV_32FC1);
        if (oolines.size()>2)
        {
            QuickSort1(oolines, (int)oolines.size(),k);//按照书籍排列方向进行相应排序，快速排序
            //            int i=0,j=i+1;
            //            while(i < oolines.size()-1)//i从0开始遍历
            for (int i = 0; i < oolines.size() - 1; i ++)
            {
                vector<cv::Point> rc;
                //根据能量函数确定最终的书脊矩形框，返回1表示找到一个框
                int bl = warf1(src, src1, angle, oolines[i], oolines[i+1], rc,k);
                //cout<<bl<<endl;
                if(1 == bl)
                {
                    int x_size = line_size(rc[0],rc[1])<line_size(rc[2],rc[3])?line_size(rc[2],rc[3]):line_size(rc[0],rc[1]);
                    int y_size = line_size(rc[0],rc[2])<line_size(rc[1],rc[3])?line_size(rc[1],rc[3]):line_size(rc[0],rc[2]);
                    int l_size = x_size < y_size ? y_size : x_size;//max最长边
                    int s_size = x_size < y_size ? x_size : y_size;//min最短边
                    if((float)l_size/(float)s_size > thresh_low)//比值大于阈值，则认为这四个点是一本书籍
                    {
                        result.push_back(rc);
                        //                        i = i + 1;
                        //                        j = i + 1;
                        
                    }
                    //                    else//否则下一个
                    //                    {
                    ////                        rest.push_back(rc);
                    //                        j = j + 1;
                    //
                    //                    }
                }
                //                else if(3 == bl|| 0 == bl || 2 == bl)//下一个
                //                {
                //                    j = j + 1;
                //                }
                //                if(j == oolines.size())//下一次遍历
                //                {
                //                    i = i + 1;
                //                    j = i + 1;
                //                }
            }
            
            
            if(0 == result.size())
                return false;//没找到书籍，为空
#if 1
            else
            {
                int ok1 = 0;int ok2 = 0;
                int **G;
                G = new int *[(int)result.size()];
                for(int i =0;i<result.size();i++)
                {
                    G[i] = new int[(int)result.size()];
                }
                
                for(int i =0;i<result.size();i++)
                {
                    for(int j =0;j < result.size();j++)
                    {
                        G[i][j] = 0;
                    }
                }
                
                for(int i =0;i<result.size();i++)
                {
                    for (int j=0; j<result.size(); j++) {
                        
                        //                        if(i == j)
                        //                        {
                        //                            G[i][j] = 0;
                        //                        }
                        //                        else
                        //                        {
                        DecideOberlap(result[i],result[j],Size(angle.cols,angle.rows),ok1,ok2);
                        G[i][j] = ok1;
                        G[j][i] = ok2;
                        //                        }
                    }
                }
                
                //            vector<int> flag;
                for(int i = 0;i<result.size();i++)
                {
                    bool isleaf = true;
                    for (int j = 0; j<result.size(); j++)
                    {
                        if(G[i][j] != 0)
                        {
                            isleaf = false;
                        }
                    }
                    if(isleaf) out_result.push_back(result[i]);
                    //                    else
                    //                        out_result.push_back(result[k]);
                }
                return 1;
                
            }
#endif
            //            for (int i = 0; i < result.size(); i ++)
            //            {
            //                out_result.push_back(result[i]);
            //            }
            //
            
        }
        
        
        return 0;
    }
    
#endif
    
    
#if 0
    void graphFind(vector<LSEG> &result)
    {
        int size = (int)result.size();
        Mat adjaceencyMatrix(size,size,CV_8U);
        for (int i = 0; i < size; i ++)
        {
            adjaceencyMatrix.at<int>(i,i) = 1;
        }
        for (int i = 0; i < size; i ++)
        {
            for (int j = 0; j < size; j ++)
            {
                <#statements#>
            }
        }
        
        
        
    }
#endif
    double ls::power1(vector<Point> &in_prs,Mat &src,Mat &angle,double tg)
    {
        bool ly = line_size(in_prs[0],in_prs[2])<line_size(in_prs[1],in_prs[3]);
        int y_size = ly?line_size(in_prs[1],in_prs[3]):line_size(in_prs[0],in_prs[2]);
        
        double w = y_size*(1-(power(src, angle, in_prs[1], in_prs[3])+power(src, angle, in_prs[0], in_prs[2]))/(2*tg));
        
        return w;
    }
    
#if 1
    void ls::select_right_spine(Mat & angle,vector<LSEG> &res,vector<LSEG>res_out,int k)
    {
        int pr_in = 0,pr_out = 0;
        vector<int> a,b;
        vector<LSEG> res_tmp;
        for (int i = 0; i < res.size(); i ++)
        {
            double d_y1 = line_size(res[i][0], res[i][1]) < line_size(res[i][2], res[i][3]) ? line_size(res[i][0], res[i][1]) : line_size(res[i][2], res[i][3]);
            double d_x1 = line_size(res[i][0], res[i][2]) < line_size(res[i][1], res[i][3]) ? line_size(res[i][0], res[i][2]) : line_size(res[i][1], res[i][3]);
            if ((d_y1 / d_x1) >= 3)
            {
                res_tmp.push_back(res[i]);
            }
            for (int j = i + 1; j < res.size() && j < i + 6; j ++)
            {
                DecideOberlap(res[i], res[j], Size(angle.cols,angle.rows), pr_in, pr_out);
                
                if (pr_in == -1 && pr_out == 1)
                {
                    
                }
            }
        }
        
        
        
    }
#endif
    
    
    //###################################################################
    
    
    
    void ls::findline(Mat &flood,vector<LSEG> &lines,int k)
    {
        vector<vector<Point> > contours;
        //轮廓查找,每一条直线
        findContours(flood, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        //imshow("findcontours", flood);
        
        vector<LSEG> oolines;
        //    vector<cv::Point2f> x;
        //    vector<cv::Point2f> y;
        for (int m = 0; m < contours.size(); m++)
        {
            Vec4f ssline;//拟合直线的参数
            //vector<cv::Point> zcontours;
            int max_py = 0,min_py = 10000,max_px = 0,min_px = 10000;//最大和最小点的初值
            for(int n = 0;n < contours[m].size();n++)
            {
                //            circle(flood, contours[m][n], 5, Scalar(100,255,100));
                //            imshow("point", flood);
                //            waitKey();
                //从返回的轮廓中找到最大点和最小点
                if(max_py < contours[m][n].y) max_py = contours[m][n].y;
                if(min_py > contours[m][n].y) min_py = contours[m][n].y;
                if(max_px < contours[m][n].x) max_px = contours[m][n].x;
                if(min_px > contours[m][n].x) min_px = contours[m][n].x;
                //cout<<"x:"<<contours[m][n].x<<"  "<<"y:"<<contours[m][n].y<<endl;
            }
            //拟合和连接轮廓线
            fitLine(contours[m], ssline, CV_DIST_L2, 0,0.1,0.1);
            //cout<<"0:"<<ssline[0]<<endl<<"1:"<<ssline[1]<<endl<<"2:"<<ssline[2]<<endl<<"3:"<<ssline[3]<<endl;
            
            Point p0,p1;
            if(abs(max_px - min_px) <= 3)//width
            {
                p0.x = ssline[2];
                p0.y = min_py;
                p1.x = ssline[2];
                p1.y = max_py;
            }
            else if(abs(max_py - min_py) <= 3)
            {
                p0.x = min_px;
                p0.y = ssline[3];
                p1.x = max_px;
                p1.y = ssline[3];
            }
            else if(ssline[0] > sqrt(2)*0.5)//45度
            {
                int ay = (ssline[1]/ssline[0]) * (min_px - ssline[2]) + ssline[3];
                int ax = min_px;
                int by = (ssline[1]/ssline[0]) * (max_px - ssline[2]) + ssline[3];
                int bx = max_px;
                p0 = bx>ax?Point(ax,ay):Point(bx,by);
                p1 = bx<ax?Point(ax,ay):Point(bx,by);
            }
            else
            {
                p0.y = min_py;
                p0.x = ssline[2]+((double)ssline[0]/(double)ssline[1])*(min_py - ssline[3]);
                p1.y = max_py;
                p1.x = ssline[2]+((double)ssline[0]/(double)ssline[1])*(max_py - ssline[3]);
            }
            
            //        cv::Point2f v,fv;
            //        v.x = ((float)p1.x - (float)p0.x)/line_size(p0,p1);
            //        v.y = ((float)p1.y - (float)p0.y)/line_size(p0,p1);
            //        fv.x = -v.x;
            //        fv.y = -v.y;
            //        cv::Point px,py;
            //        px = prpoint(p0, 0, v);
            //        py = prpoint(p1, 0, fv);
            LSEG outline;
            outline.push_back(p0);
            outline.push_back(p1);
            
            lines.push_back(outline);
        }
    }
    
    
    
    vector<LSEG> ls::line_preprocess(vector<LSEG> &lines,int k)
    {
        vector<LSEG> oolines,lines_tmp;
        //    vector<cv::Point2f> x;
        //    vector<cv::Point2f> y;
        
        for (int m = 0; m < lines.size(); m++)
        {
            
            // double d = line_size(lines[m][0], lines[m][1]);
            //if (d > 100)
            //{
            
            Point p0,p1;
            if (1 == k)
            {
                p1 = lines[m][0].x > lines[m][1].x ? lines[m][0] : lines[m][1];
                p0 = lines[m][0].x < lines[m][1].x ? lines[m][0] : lines[m][1];
            }
            else
            {
                p1 = lines[m][0].y > lines[m][1].y ? lines[m][0] : lines[m][1];
                p0 = lines[m][0].y < lines[m][1].y ? lines[m][0] : lines[m][1];
            }
            
            cv::Point2f v,fv;
            v.x = ((float)p1.x - (float)p0.x)/line_size(p0,p1);
            v.y = ((float)p1.y - (float)p0.y)/line_size(p0,p1);
            fv.x = -v.x;
            fv.y = -v.y;
            cv::Point px,py;
            px = prpoint(p0, -5, v);
            py = prpoint(p1, -5, fv);
            LSEG outline;
            outline.push_back(px);
            outline.push_back(py);
            oolines.push_back(outline);
            // }
        }
        
        return oolines;
        
    }
    
    //void ls::parallelines(vector<LSEG> &lines,vector<LSEG> &outlines)
    //{
    //    QuickSort(lines, (int)lines.size(), 4);
    //
    //}
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#if 1
    void get_vertical_vertice(LINE l1,LINE l2,Point corners,Point corners_true,Point corners_other);
    void preprocess_corners(vector<Point> &corners,int cnt_xy,int cnt_rl,LSEG &pts_xy,LSEG &pts_rl);
    void getL(LINE &l1,LINE &l2,LSEG &lp);
    unsigned char* AdaptiveThreshold(unsigned char* input, int width, int height);
    
    void book_segment(Mat &src,vector<Mat> &out_result)
    {
        Mat img_resize;
        Mat src1 = src.clone();
        //缩放图片
        resize(src, img_resize, Size(((float)src.cols/(float)src.rows)*640,640));
        Mat img_gray;
        //灰度化
        cvtColor(img_resize, img_gray, CV_RGB2GRAY);
        equalizeHist(img_gray, img_gray);
        medianBlur(img_gray, img_gray, 5);
        GaussianBlur(img_gray, img_gray, Size(5,5), 0);
        //imshow("binary", img_gray);
        
        //锐化，突出边界轮廓
        // Laplacian(img, img, img.depth());
        Mat kernel(3,3,CV_32F,Scalar(-1));
        kernel.at<float>(1,1) = 8;
        filter2D(img_gray, img_gray, img_gray.depth(), kernel);
        imshow("sharp", img_gray);
        imwrite("/Users/brdev/Desktop/test_out/sharp.jpg", img_gray);
        
        //边缘检测
        Canny(img_gray, img_gray, 30, 200);
        imshow("canny", img_gray);
        //    Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3,3));
        //    erode(img_gray, img_gray, kernel1);
        //    imshow("erode", img_gray);
        vector<LSEG> linesy,linesx,linesr,linesl;
        vector<Point> corners;
        //获取角点,一堆杂乱的点
        //cornerHarris(src1, dst, 2, 3, 0.01);
        Mat out_rect;
        goodFeaturesToTrack(img_gray, corners, 400, 0.01, 18);
        
        for (int i = 0; i < corners.size(); i ++)
        {
            circle(img_gray, corners[i], 3, Scalar(255,0,0));
        }
        imshow("corners", img_gray);
        imwrite("/Users/brdev/Desktop/test_out/corners.jpg", img_gray);
        
        
        
    }
#endif
    
    
    
    
    
    
    //判断点是否在直线上
    int ls::is_onLine(Point2f p, LSEG lines)
    {
        Point2f v;
        v.x = (lines[0].x - lines[1].x)/line_size(lines[0], lines[1]);
        v.y = (lines[0].y - lines[1].x)/line_size(lines[0], lines[1]);
        int k;
        k = v.x * (p.y - lines[0].y) - v.y * (p.x - lines[0].x);
        
        if(0 == k)
            return 1;
        else
            return 0;
        
        
    }
#if 0
    //****
    void get_vertical_vertice(LINE l1,LINE l2,Point corners,Point corners_true,Point corners_other)//保留起点和末点相同，或者起点和起点相同的直线对，其他另外保存，在做后续处理
    {//重新改写，思路理清
        Point vertice;//corners_true,corners_other;
        bool inter = lineintersect(l1, l2, vertice);
        int v = l1.a * l2.b + l1.b * l2.a;
        if (inter && !v && (l1.start == corners) && (l1.start == l2.start || l1.start == l2.end))
        {
            corners_true = l1.start;
            
        }
        else if (inter && !v && (l1.end == corners) && (l1.end == l2.end || l1.end == l2.start))
        {
            corners_true = l1.end;
        }
        else if (inter && !v && vertice == corners)
            //else
        {
            corners_other = corners;
        }
        
        //return corners_true;
        
    }
    //get the "L"
    void getL(LINE &l1,LINE &l2,LSEG &lp)
    {
        Point ver;
        bool ii = lineintersect(l1, l2, ver);
        if (!ii) return;
        else
        {
            if (ver == l1.start)
            {
                if (l1.start == l2.start)
                {
                    lp.push_back(l2.end);
                    lp.push_back(ver);
                    lp.push_back(l1.end);
                }
                else if(l1.start == l2.end)
                {
                    lp.push_back(l1.end);
                    lp.push_back(ver);
                    lp.push_back(l2.start);
                    
                }
                
                
            }
            else if (ver == l1.end)
            {
                if (l1.end == l2.start)
                {
                    lp.push_back(l1.start);
                    lp.push_back(ver);
                    lp.push_back(l2.end);
                }
                else if(l1.end == l2.end)
                {
                    lp.push_back(l2.start);
                    lp.push_back(ver);
                    lp.push_back(l1.start);
                    
                }
                
                
                
            }
        }
        
        
        
    }
#endif
    
    //    int ls::ImageAdjust(Mat &src, Mat &dst,
    //                    double low, double high,   // X方向：low and high are the intensities of src
    //                    double bottom, double top, // Y方向：mapped to bottom and top of dst
    //                    double gamma )
    //    {
    //        double low2 = low*255;
    //        double high2 = high*255;
    //        double bottom2 = bottom*255;
    //        double top2 = top*255;
    //        double err_in = high2 - low2;
    //        double err_out = top2 - bottom2;
    //        int x,y;
    //        double val;
    //        if(low<0 && low>1 && high <0 && high>1 &&
    //           bottom<0 && bottom>1 && top<0 && top>1 && low>high)
    //            return -1;
    //        // intensity transform
    //        for( y = 0; y < src.rows; y++)
    //        {
    //            for (x = 0; x < src.cols; x ++)
    //            {
    //                //val = ((uchar*)(src.data + src.step*y))[x];
    //
    //                val = src.at<float>(x,y);
    //                val=pow((val - low2)/err_in, gamma)*err_out+bottom2;
    //                if(val > 255)
    //                    val = 255.0;
    //                if(val < 0)
    //                    val = 0; // Make sure src is in the range [low,high]
    //                //((uchar*)(dst.data + dst.step*y))[x] = (uchar) val;
    //                dst.at<float>(x, y) = val;
    //            }
    //        }
    //        return 0;
    //    }
    
    void ls::gammaCorrection(Mat &src, Mat &dst, float gamma)
    {
        CV_Assert(src.data);
        // accept only char type matrices
        CV_Assert(src.depth() != sizeof(uchar));
        // build look up table
        uchar lut[256];
        for( int i = 0; i < 256; i++ )
        {
            lut[i] = saturate_cast<uchar>(pow((float)(i/255.0), gamma) * 255.0f);//防止数据溢出
        }
        dst = src.clone();
        const int channels = dst.channels();
        switch(channels)
        {
            case 1://grayscale
            {
                MatIterator_<uchar> it = dst.begin<uchar>();
                MatIterator_<uchar> end = dst.end<uchar>();
                for( ; it != end; it++ )
                    //*it = pow((float)(((*it))/255.0), fGamma) * 255.0;
                    *it = lut[(*it)];
                break;
            }
            case 3://BGR
            {
                MatIterator_<Vec3b> it = dst.begin<Vec3b>();
                MatIterator_<Vec3b> end = dst.end<Vec3b>();
                for( ; it != end; it++ )
                {
                    (*it)[0] = lut[((*it)[0])];
                    (*it)[1] = lut[((*it)[1])];
                    (*it)[2] = lut[((*it)[2])];
                }
                break;
            }
        }
    }
    
    
    
    //初始化start
#define TEST 1
    
    
    
    //void crop(Mat & src,Mat &dst)
    //{
    //    int width = src.cols;
    //    int height = src.rows;
    //    dst = Mat::zeros(height-10, width-4, src.type());
    //
    //    for (int i = 2; i < width - 2; i ++)
    //    {
    //        for (int j = 5; j < height - 5; j ++)
    //        {
    //            dst.push_back(src.at<uchar>(j, i));
    //        }
    //    }
    //}
    
    
    void ls::scanNoteFilter(Mat &src,Mat &dst,int flagProcess)
    {
        Mat image;
        resize(src, image, Size(((float)src.cols/(float)src.rows)*krows,krows));
        int nl = image.rows;//行数
        int nc = image.cols * image.channels();//每行的元素个数
        if (image.isContinuous())
        {
            nc = nc * nl;
            nl = 1;
        }
        int count_white = 0,count_black = 0;
        //if (1 == image.channels())
        // {
        for (int j = 0; j < nl; j ++)
        {
            uchar *data = image.ptr<uchar>(j);//每行的首地址
            for (int i = 0; i < nc; i ++)
            {
                //                    cout<<*data++<<endl;
                // cout<<(int)(*data ++)<<endl;
                float normVal = (*data ++) / 255.0;
                //cout<<normVal<<endl;
                if (normVal > 0.75 )
                {
                    count_white ++;
                }
                else if (normVal <= 0.25)
                    count_black ++;
                //else
                
            }
        }
        float num_w,num_b;
        num_w = count_white / (float)nc;
        num_b = count_black / (float)nc;
        cout<<num_b<<endl;
        cout<<num_w<<endl;
        if (num_b < 0.03)
            gammaCorrection(image, dst, 0.62);
        else
            gammaCorrection(image, dst, 0.46);
        
    }
    
    
    bool cmp_x(const Line &p1, const Line &p2) {
        return p1._center.x < p2._center.x;
    }
    
    void ls::scan1(Mat &img, float out_point[8])
    {
        
        // resize input image to img_proc to reduce computation
        Mat img_proc;
        //ls sc;
        int w = img.size().width, h = img.size().height, min_w = 200;
        double scale = min(10.0, w * 1.0 / min_w);
        int w_proc = w * 1.0 / scale, h_proc = h * 1.0 / scale;
        resize(img, img_proc, Size(w_proc, h_proc));
        Mat img_dis = img_proc.clone();
        Mat gray,canny;
        //    GaussianBlur(img_proc, img_proc, Size(3,3), 0,0);
        //    Mat x,y;
        //    Sobel(img_proc, x, CV_32F, 0, 1,3);
        //    Sobel(img_proc, y, CV_32F, 1, 0,3);
        //    Mat magnitude,angle;//幅度和角度
        //    cartToPolar(y, x, magnitude, angle);//计算二维向量的长度、角度或者两者， magnitude：存储向量长度输出数组
        //    //cartToPolar(out, out,magnitude, angle);
        //    vector<Vec4i> lSegs,outlines;
        //    //线检测
        //    Ptr<LineSegmentDetector> ld = createLineSegmentDetector();
        //    ld->detect(img_proc, lSegs);
        //    Mat edgey = Mat::zeros(img_proc.rows, img_proc.cols, CV_8U);//保存竖直摆放的书籍边界点
        //    Mat edgex = Mat::zeros(img_proc.rows, img_proc.cols, CV_8U);//保存水平摆放的书籍边界点
        //    vector<>
        
        cvtColor(img_proc, gray, CV_RGB2GRAY);
        // Mat img_dis = img_proc.clone();
        GaussianBlur(gray, gray, Size(3,3), 0,0);
        Mat x,y;
        Sobel(gray, x, CV_32F, 0, 1,3);
        Sobel(gray, y, CV_32F, 1, 0,3);
        Mat magnitude,angle;//幅度和角度
        cartToPolar(y, x, magnitude, angle);//计算二维向量的长度、角度或者两者， magnitude：存储向量长度输出数组
        //cartToPolar(out, out,magnitude, angle);
        vector<Vec4i> Segs;
        //线检测
        Ptr<LineSegmentDetector> ld = createLineSegmentDetector();
        ld->detect(gray, Segs);
        Mat edgey = Mat::zeros(gray.rows, gray.cols, CV_8U);
        Mat edgex = Mat::zeros(gray.rows, gray.cols, CV_8U);
        Mat edge = Mat::zeros(gray.rows, gray.cols, CV_8U);
        vector<Line> horizontals, verticals;
        for (int i = 0; i < Segs.size(); i ++)
        {
            cv::Point pt1 = cv::Point(Segs[i][0],Segs[i][1]);//将Segs中的每个元素的前两个赋给pt1
            cv::Point pt2 = cv::Point(Segs[i][2],Segs[i][3]);////将Segs中的每个元素的后两个赋给pt2
            if (line_size(pt1, pt2) > 86)
            {
                cv::Point p1,p2;
                if (pt1.y > pt2.y)
                {
                    p1 = pt2;
                    p2 = pt1;
                }
                else
                {
                    p1 = pt1;
                    p2 = pt2;
                    
                }
                Line tmp(p1,p2);
                float jiao = line_jiao(p1, p2);
                //            if (jiao < 2 || jiao > 178)
                //            {
                //                //circle(img_proc, pt1, 3, Scalar(0,255,0));
                //                //circle(img_proc, pt2, 3, Scalar(0,0,255));
                //                line(edgex, pt1, pt2, Scalar(255,255,255),2,3);
                //                horizontals.push_back(tmp);
                //            }
                if ((abs(jiao) > 70 && abs(jiao) < 120) )//|| ((180-abs(jiao)) > 80 && (180-abs(jiao)) < 100))
                {
                    //circle(img_proc, p1, 3, Scalar(0,255,0));
                    // circle(img_proc, p2, 3, Scalar(0,0,255));
                    line(edgey,p1,p2,Scalar(255,255,255),2,3);
                    
                    verticals.push_back(tmp);
                    //imshow("updown", img_proc);
                    //waitKey(10);
                    
                }
                line(img_proc, p1, p2, Scalar(255,0,0),2,3);
            }
        }
        //imshow("x", edgex);
        imshow("y", edgey);
        imshow("image", img_proc);
        
        
        
        /* get four outline edges of the document */
        // get edges of the image
        // Mat gray, canny;
        //cvtColor(img_proc, gray, CV_BGR2GRAY);
        //getCanny(gray, canny);
        //imshow("canny", canny);
        //morphologyEx(canny, canny, MORPH_DILATE, Mat(7,7,CV_8U),Point(-1,-1),1);
        // imshow("dilate", canny);
        //    vector<KeyPoint> corners;
        //    Ptr<ORB> orb = ORB::create();
        //    orb->detect(edge, corners);
        //    vector<Point2f> pts;
        //    cv::KeyPoint::convert(corners, pts);
        //    for (int i = 0; i < corners.size(); i ++)
        //    {
        //        circle(img_proc, pts[i], 3, Scalar(0,255,0));
        //    }
        //    imshow("jiaodian", img_proc);
        
        // extract lines from the edge image
        // vector<Vec4i> lines;
        
        // HoughLinesP(canny, lines, 1, CV_PI / 180, w_proc / 3, w_proc / 3, 20);
        //Rect rect = find_max_area_contour(img_proc,edge);
        // Rect rect(0,0,w_proc,h_proc);
        //    for (size_t i = 0; i < lines.size(); i++)
        //    {
        //        Vec4i v = lines[i];
        //        double delta_x = v[0] - v[2], delta_y = v[1] - v[3];
        //        Line l(Point(v[0], v[1]), Point(v[2], v[3]));
        //        // get horizontal lines and vertical lines respectively
        //        if (fabs(delta_x) > fabs(delta_y))
        //        {
        //            horizontals.push_back(l);
        //        }
        //        else
        //        {
        //            verticals.push_back(l);
        //        }
        //        // for visualization only
        //        line(img_proc, Point(v[0], v[1]), Point(v[2], v[3]), Scalar(0, 0, 255), 1, CV_AA);
        //    }
        //    imshow("lines", img_proc);
        
        
        // edge cases when not enough lines are detected
#if 0
        if (horizontals.size() < 2)
        {
            if ( 0 == horizontals.size() )
            {
                horizontals.push_back(Line(Point(0, 0), Point(w_proc - 1, 0)));
                horizontals.push_back(Line(Point(0, h_proc - 1), Point(w_proc - 1, h_proc - 1)));
            }
            if ( 1 == horizontals.size() && horizontals[0]._center.y > h_proc / 2 )
            {
                horizontals.push_back(Line(Point(0, 0), Point(w_proc - 1, 0)));
            }
            else//if (1 == horizontals.size() && horizontals[0]._center.y <= h_proc / 2 )
            {
                horizontals.push_back(Line(Point(0, h_proc - 1), Point(w_proc - 1, h_proc - 1)));
            }
            // for(int i = 0;i < )
        }
        else if (horizontals.size() >= 2)
        {
            
            sort(horizontals.begin(), horizontals.end(), cmp_y);
            for(int num = 0; num < horizontals.size();num ++)
            {
                for(int denum = num + 1; denum < horizontals.size();denum ++)
                {
                    double d = abs(horizontals[num]._center.y - horizontals[denum]._center.y);
                    if(d == rect.height)///////
                    {
                        horizontals.push_back(horizontals[num]);
                        horizontals.push_back(horizontals[denum]);
                    }
                }
                
            }
            
        }
#endif
        
        if (verticals.size() < 2)
        {
            if (0 == verticals.size() )
            {
                verticals.push_back(Line(Point(0, 0), Point(0, h_proc - 1)));
                verticals.push_back(Line(Point(w_proc - 1, 0), Point(w_proc - 1, h_proc - 1)));
            }
            if (1 == verticals.size() && verticals[0]._center.x > w_proc / 2)
            {
                
                Line tmp = verticals[0];
                verticals.clear();
                verticals.push_back(Line(Point(0, 0), Point(0, h_proc - 1)));
                verticals.push_back(tmp);
                
            }
            else//if (1 == verticals.size() && verticals[0]._center.x <= w_proc / 2)
            {
                //            circle(img_dis, verticals[0]._p1, 3, Scalar(0,0,255),2,3);
                //           // imshow("_1", img_dis);
                //            circle(img_dis, verticals[0]._p2, 3, Scalar(0,255,0),2,3);
                //imshow("_2", img_dis);
                
                //            Line tmp = verticals[0];
                //            verticals.clear();
                verticals.push_back(Line(Point(w_proc - 1, 0), Point(w_proc - 1, h_proc - 1)));
            }
            
        }
        else if (verticals.size() > 2)
        {
            sort(verticals.begin(), verticals.end(), cmp_x);//按照中点横坐标的升序排列
            vector<Line> linesReal;
            for (int i = 0; i < verticals.size(); i ++)
                linesReal.push_back(verticals[i]);
            verticals.clear();
            //Rect rect = find_max_area_contour(canny);
            int num = 0,denum = num + 1;
            while (num < linesReal.size() - 1)
            {
                double d = abs(linesReal[num]._center.x - linesReal[denum]._center.x);
                if (d > 0.5*w_proc)
                {
                    verticals.push_back(linesReal[num]);
                    verticals.push_back(linesReal[denum]);
                    num = num + 1;
                    denum = num + 1;
                }
                else
                {
                    denum = denum + 1;
                }
                if (denum == linesReal.size())
                {
                    num = num + 1;
                    denum = num + 1;
                }
            }
            
            
        }
        else
        {//=2
            if ( verticals[0]._center.x > w_proc / 2 && verticals[1]._center.x > w_proc / 2)
            {
                Line tmp = verticals[0];
                verticals.clear();
                verticals.push_back(Line(Point(0, 0), Point(0, h_proc - 1)));
                verticals.push_back(tmp);
            }
            else if (verticals[1]._center.x <= w_proc / 2 && verticals[0]._center.x <= w_proc / 2)
            {
                Line tmp = verticals[1];
                verticals.clear();
                verticals.push_back(tmp);
                verticals.push_back(Line(Point(w_proc - 1, 0), Point(w_proc - 1, h_proc - 1)));
            }
        }
        
        sort(verticals.begin(), verticals.end(), cmp_x);
        
        //circle(img_dis, cv::Point(0,0), 3, Scalar(0,255,0),2,3);
        //imshow("_0", img_dis);
        //circle(img_dis, cv::Point(0,h_proc-1), 3, Scalar(0,255,0),2,3);
        //imshow("_h", img_dis);
        //    circle(img_dis, verticals[0]._p1, 3, Scalar(0,255,0),2,3);
        //    imshow("_1", img_dis);
        //    circle(img_dis, verticals[0]._p2, 3, Scalar(0,255,0),2,3);
        //    imshow("_2", img_dis);
        //    circle(img_dis, verticals[1]._p1, 3, Scalar(0,255,0),2,3);
        //    imshow("_3", img_dis);
        //    circle(img_dis, verticals[1]._p2, 3, Scalar(0,255,0),2,3);
        //    imshow("_4", img_dis);
        
        
        
        Point2f v1,fv1,vt1,fvt1,v2,fv2,vt2,fvt2;
        cv::Point p1,p2,p3,p4,tp1,tp2,tp3,tp4;
        Point fp1 = verticals[0]._p1,lp1 = verticals[0]._p2;
        Point fp2 = verticals[1]._p1,lp2 = verticals[1]._p2;
        vector<Point2f> points;
        double l0 = line_size(fp1,lp1);
        double l1 = line_size(fp2,lp2);
        //float max = d0 < d1 ? d1 : d0;
        v1.x = ((float)lp1.x - (float)fp1.x)/l0;
        v1.y = ((float)lp1.y - (float)fp1.y)/l0;
        vt1.x = -v1.y;
        vt1.y = v1.x;
        fv1.x = -v1.x;
        fv1.y = -v1.y;
        fvt1.x = -vt1.x;
        fvt1.y = -vt1.y;
        
        
        v2.x = ((float)lp2.x - (float)fp2.x)/l1;
        v2.y = ((float)lp2.y - (float)fp2.y)/l1;
        vt2.x = -v2.y;
        vt2.y = v2.x;
        fv2.x = -v2.x;
        fv2.y = -v2.y;
        fvt2.x = -vt2.x;
        fvt2.y = -vt2.y;
        p1 = fp1;
        p2 = lp1;
        p3 = fp2;
        p4 = lp2;
        
        float d1 = abs(v2.y*p1.x - v2.x*p1.y + v2.x*p3.y -v2.y*p3.x);//p1到v2的距离
        float d2 = abs(v2.y*p2.x - v2.x*p2.y + v2.x*p3.y -v2.y*p3.x);//p2到v2的距离
        float d3 = abs(v1.y*p3.x - v1.x*p3.y + v1.x*p1.y -v1.y*p1.x);//p3到v1的距离
        float d4 = abs(v1.y*p4.x - v1.x*p4.y + v1.x*p1.y -v1.y*p1.x);//p4到v1的距离
        if (l1 - l0 > 10)
        {
            tp3 = prpoint(p3, d3, fvt1);
            tp4 = prpoint(p4, d4, fvt1);
            if (tp3.y < 0 || tp4.x < 0) {
                cv::Point p3_tmp,p4_tmp;
                tp3 = cv::Point(tp3.x,0);
                tp4 = cv::Point(0,tp4.y);
                // tp3 =
            }
            points.push_back(tp3);
            points.push_back(fp2);
            points.push_back(tp4);
            points.push_back(lp2);
        }
        else if (l0 - l1 > 10)
        {
            tp1 = prpoint(p1, d1, vt2);
            tp2 = prpoint(p2, d2, vt2);
            if (tp1.y < 0) {
                tp1 = cv::Point(tp1.x,0);
            }
            points.push_back(fp1);
            points.push_back(tp1);
            points.push_back(lp1);
            points.push_back(tp2);
        }
        else
        {
            points.push_back(fp1);
            points.push_back(fp2);
            points.push_back(lp1);
            points.push_back(lp2);
        }
        
        //}
        
        
        // sort lines according to their center point
        //sort(horizontals.begin(), horizontals.end(), cmp_y);
        //    sort(verticals.begin(), verticals.end(), cmp_x);
        
        
        //    points.push_back(computeIntersect(horizontals[0], verticals[0]));
        //    points.push_back(computeIntersect(horizontals[0], verticals[1]));
        //    points.push_back(computeIntersect(horizontals[1], verticals[1]));
        //    points.push_back(computeIntersect(horizontals[1], verticals[0]));
        //    points.push_back(verticals[0]._p1);
        //    points.push_back(verticals[1]._p1);
        //    points.push_back(verticals[0]._p2);
        //    points.push_back(verticals[1]._p2);
        
        for(int i = 0;i < points.size();i ++)
        {
            circle(img_dis, points[i], 3, Scalar(0,255,0));
            //        imshow("corners", img_proc);
            //        waitKey(10);
            
        }
        imshow("corners", img_dis);
        
        
        out_point[0] = points[0].x*scale;
        out_point[1] = points[0].y*scale;
        out_point[2] = points[1].x*scale;
        out_point[3] = points[1].y*scale;
        out_point[4] = points[2].x*scale;
        out_point[5] = points[2].y*scale;
        out_point[6] = points[3].x*scale;
        out_point[7] = points[3].y*scale;
        
        
        
        
        
    }
    
    
    void ls::perspective(Mat &src, float in_point[8], Mat &dst)
    {
        float w_a4 = sqrt(pow(in_point[0] - in_point[2], 2) + pow(in_point[1] - in_point[3] ,2 ));
        float h_a4 = sqrt(pow(in_point[0] - in_point[4], 2) + pow(in_point[1] - in_point[5] ,2));
        dst = Mat::zeros(h_a4, w_a4, CV_8UC3);
        
        
        
        // corners of destination image with the sequence [tl, tr, bl, br]
        vector<Point2f> dst_pts, img_pts;
        dst_pts.push_back(Point(0, 0));
        dst_pts.push_back(Point(w_a4 - 1, 0));
        dst_pts.push_back(Point(0, h_a4 - 1));
        dst_pts.push_back(Point(w_a4 - 1, h_a4 - 1));
        
        // corners of source image with the sequence [tl, tr, bl, br]
        img_pts.push_back(Point(in_point[0], in_point[1]));
        img_pts.push_back(Point(in_point[2],in_point[3]));
        img_pts.push_back(Point(in_point[4],in_point[5]));
        img_pts.push_back(Point(in_point[6], in_point[7]));
        
        
        // get transformation matrix
        Mat transmtx = getPerspectiveTransform(img_pts, dst_pts);
        
        // apply perspective transformation
        warpPerspective(src, dst, transmtx, dst.size());
    }

    
    void getROI(cv::Mat &src,cv::Mat &out,cv::Rect rect)
    {
        out = cv::Mat(rect.width, rect.height, CV_8UC3,Scalar(125));
        vector<cv::Point2f>  quad_pts;
       //映射到原图上
        vector<cv::Point2f> pointss;
        pointss.push_back(rect.tl());
        pointss.push_back(rect.tl() + cv::Point(rect.width,0));
        pointss.push_back(rect.tl() + cv::Point(0,rect.height));
        pointss.push_back(rect.br());

        //以原点为顶点的矩形
        quad_pts.push_back(cv::Point2f(0,0));
        quad_pts.push_back(cv::Point2f(rect.height,0));
        quad_pts.push_back(cv::Point2f(0,rect.width));
        quad_pts.push_back(cv::Point2f(rect.height,
                                       rect.width));
        
        //    获取透视变换的变换矩阵
        cv::Mat transmtx = getPerspectiveTransform(pointss, quad_pts);
        
        warpPerspective(src, out, transmtx,out.size());

    }
    
    
    void ls::bookShelfSegment(cv::Mat &src, vector<cv::Mat> &results,vector<cv::Mat> &corner)
    {
        Mat image;
        //image = src1.clone();
        resize(src,image,Size(((float)src.cols/(float)src.rows)*krows,krows));
        //    imwrite("/Users/brdev/Documents/bookSegment Algorithms/Book-Identifier/code/main/test.jpg", image);
        //    imshow("resize", image);
        //    detailEnhance(image, image);
        //    imshow("detail", image);
        Mat img = image.clone();
        Mat gray;
        cvtColor(image, gray, CV_BGR2GRAY);
        cv::Mat gray2 = gray.clone();
        int numPixInTile = gray.rows*gray.cols/(18*18);
        int minClipLimit = ceil(numPixInTile/256.0);
        double normClipLimit = 0.01;
        double cliplimit = minClipLimit + floor(normClipLimit*(numPixInTile - minClipLimit) + 0.5);
        adaptiveHistEqual(gray, gray,cliplimit);
        
        cv::Mat edges;
        //    double high = 300;
        blur(gray, gray, Size(3,3));
        
        
        /////////
        cv::Mat ox,oy,mag,ang;
        Sobel(gray, ox, CV_32F, 0, 1 ,3);
        Sobel(gray, oy, CV_32F, 1, 0 ,3);
        blur(ox, ox, Size(3,3));
        blur(oy, oy, Size(3,3));
        cartToPolar(oy, ox, mag, ang);
        
        ///////////
        
        double low = 8;
        
        Canny(gray, edges, low,8*low,3);//150,800
//        imshow("canny", edges);
        //    WriteData("/Users/brdev/Desktop/edge.txt", edges);
        double dtheta = 0.2;
        double drho = 1.0;
        double theta_in = 20;
        //    src1 = src1/255.0;
        edges.convertTo(edges, CV_64F);
        //    WriteData("/Users/brdev/Desktop/edge.txt", edges);
        //    imshow("doublecanny", edges);
        //    imwrite("/Users/brdev/Desktop/edge.jpg", edges);
        vector<double> rho,theta;
        cv::Mat H;
        getThetaRho(edges, theta, rho, theta_in, dtheta, drho);
        vector<double> theta_new;
        for (int i = 0; i < theta.size(); i ++)
        {
            theta_new.push_back(theta[i]*CV_PI/180);
        }
        myHoughMex(edges, H, theta_new, rho);
        //    WriteData("/Users/brdev/Desktop/h.txt", H);
        int hood_size[] = {51,51};
        int numpeaks = 50;
        double H_min,H_max;
        minMaxLoc(H, &H_min,&H_max);
        double thresh = ceil(0.5*H_max);
        vector<double> r,c;
        houghPeaks(H, numpeaks, thresh, hood_size, r, c);
        vector<LSEG> spine_lines;
        double fillgap = gray2.cols/60.0;
        double minlength = gray2.rows/1.5;
        vector<double> nonzero_x,nonzero_y,val;
        findNonZeroIdx(gray2 , nonzero_y, nonzero_x, val);
        spine_lines = myHoughLines(gray2, theta, rho, r, c,nonzero_x,nonzero_y, fillgap, minlength);
//        for (int i = 0; i < spine_lines.size(); i ++)
//        {
//            cv::Point2f p1(spine_lines[i][0]);
//            cv::Point2f p2(spine_lines[i][1]);
//            line(image, p1, p2, Scalar(0,255,0),1.5);
//            //        imshow("spine", image);
//            //        waitKey(100);
//        }
//        imshow("spines_line", image);
        //    imshow("houghline", linesx);
        //    LSEG baseline;
        //    QuickSort(spine_lines, (int)spine_lines.size(), 1, baseline);
        //    for (int i = 0; i < spine_lines.size(); i ++)
        //    {
        //        cv::Point2f p1(spine_lines[i][0]);
        //        cv::Point2f p2(spine_lines[i][1]);
        //        line(image, p1, p2, Scalar(0,255,0),1.5);
        ////        imshow("spine", image);
        ////        waitKey(100);
        //    }
        vector<LSEG> spines;
        LSEG first,last;
        first.push_back(cv::Point(1,0));
        first.push_back(cv::Point(1,image.rows-1));
        last.push_back(cv::Point(image.cols-2,0));
        last.push_back(cv::Point(image.cols-2,image.rows-2));
        spines.push_back(first);
        for (int num = 0; num < spine_lines.size(); num ++)
        {
            spines.push_back(spine_lines[num]);
        }
        spines.push_back(last);
        vector<LSEG> res,other;
        //    vector<cv::Mat> book_cor;
        pr_detect1(img,src,ang,spines,res,1);
        //    for (int i = 0; i < spines.size(); i ++)
        //    {
        //        cv::Point2f p1(spines[i][0]);
        //        cv::Point2f p2(spines[i][1]);
        //        line(image, p1, p2, Scalar(0,255,0),1.5);
        ////        imshow("spine", image);
        ////        waitKey(1000);
        //    }
        for (int i = 0; i < res.size(); i++)
        {
            //cv::Point center = get_gravity_center(result[i]);
            //        cv::Point2f uls = get_up_left(res[i], 3.0);
            //        cv::Point2f offsets = Point(3,3);
            //        cv::Point2f pcs = uls - offsets;
            //        circle(image, uls, 7, Scalar(0,255,0),-1,5,0);
            //        char *texts = new char[100];
            //        sprintf(texts, "%d",i+1);
            //        putText(image, texts, pcs, FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(0,255,255),0.2);
            //        imshow("marker", image);
            // imwrite("/Users/brdev/Desktop/test_out/marker.jpg", image);
            //waitKey(100);
            Mat as(500,500,CV_8U);
            Mat coordinates(4,2,CV_32FC1);
            vector<Point2f> pointss;
            //矫正
            warp_change(res[i],src,img,as,pointss);
            coordinates.at<float>(0,0) = pointss[0].x;
            coordinates.at<float>(0,1) = pointss[0].y;
            coordinates.at<float>(1,0) = pointss[1].x;
            coordinates.at<float>(1,1) = pointss[1].y;
            coordinates.at<float>(2,0) = pointss[2].x;
            coordinates.at<float>(2,1) = pointss[2].y;
            coordinates.at<float>(3,0) = pointss[3].x;
            coordinates.at<float>(3,1) = pointss[3].y;
            
            results.push_back(as);
            corner.push_back(coordinates);
            //        imshow("a", out_result[i]);
            
            //        delete [] texts;
        }
        //    imshow("dian", image);
        
        
        
    }
    //    void ls::selectThresh(cv::Mat &Gx,cv::Mat &Gy,double percent,double thresh_ratio,double &thresh_high,double &thresh_low)
    //    {
    //        cv::Mat mag;
    ////        double t1 = 0.0,t2 = 0.0;
    //        cv::Mat dst1,dst2;
    //        multiply(Gx, Gx, dst1);
    //        multiply(Gy, Gy, dst2);
    //        cv::Mat dst = dst1 + dst2;
    //        dst.convertTo(dst, CV_32F);
    //        sqrt(dst, mag);
    //
    //        double min,max;
    //        minMaxLoc(mag, &min, &max);
    //        if (max > 0)
    //            mag = mag/max;
    //
    //        int w = mag.cols;
    //        int h = mag.rows;
    ////        thresh_high = w * h * percent / (1024*2*2);
    //        Histogrom1D h1;
    //        MatND hist = h1.getHistogram(mag);
    //        double maxVal = 0,minVal = 0;
    //        minMaxLoc(hist, &minVal, &maxVal,0,0);
    //        cv::Mat histImg(256,256,CV_8U,Scalar(255));
    //        //设置最高点为nbins的90%
    //        long sum = 0;
    //        int hpt = static_cast<int>(0.9 * 256);
    //        for(int h = 0;h < 256;h ++)
    //        {
    //            thresh_high = 0.0;
    //            float binVal = hist.at<float>(h);
    ////            int intensity = static_cast<int>(binVal*hpt/maxVal);
    //            sum = sum + binVal;
    //            if (sum > (w * h * percent))
    //            {
    //                thresh_high = h*255 ;/// 256.0;
    //            }
    //            if(thresh_high > 0)break;
    //        }
    //
    //        thresh_low = thresh_ratio * thresh_high;
    //
    //    }
    
    void ls::smoothGradient(cv::Mat &binaryImage,double sigma,cv::Mat &Gx,cv::Mat &Gy)
    {
        int filter_size = 8 * ceil(sigma);
        int n = (filter_size - 1) / 2;
        //create 1-D Gaussian Kernel
        double c = 1 / (sqrt(2 * CV_PI) * sigma);
        double sum = 0.0;
        vector<double> gauss,gauss_kernel;
        //        cv::Mat gauss_kernel;
        for (int i = -n; i <= n; i ++)
        {
            double temp = c * exp(-(i*i) / (2 * sigma * sigma));
            sum = sum + temp;
            gauss.push_back(temp);
            //            gauss_kernel.at<double>()
            
        }
        
        double tmp;
        for (int j = 0; j < gauss.size(); j ++)
        {
            //            double gauss_kernel = c * exp(-(i*i) / (2 * sigma * sigma));
            tmp  = gauss[j] / sum;
            gauss_kernel.push_back(tmp);// tmp;
            //            double deriv = g
            
        }
        vector<double> derivg;
        morphologyEx(gauss_kernel, derivg, MORPH_GRADIENT, Mat());
        vector<double> negvals,posvals;
        for (int num = 0; num < derivg.size(); num++) {
            if(derivg[num] > 0)posvals.push_back(derivg[num]);
            else if(derivg[num] < 0)negvals.push_back(derivg[num]);
        }
        
        double sum1 = 0.0,sum2 = 0.0;
        for (int x = 0; x < posvals.size(); x ++) {
            sum1 = sum1 + derivg[posvals[x]];
        }
        for (int y = 0; y < negvals.size(); y ++) {
            sum2 = sum2 + derivg[negvals[y]];
        }
        
        
        for (int i = 0; i < derivg.size(); i ++)
        {
            if(derivg[i] > 0) derivg[i] = derivg[i]/sum1;
            else if (derivg[i] < 0) derivg[i] = derivg[i] / abs(sum2);
        }
        vector<double> gauss_t,derivg_t;
        transpose(gauss_kernel, gauss_t);
        transpose(derivg, derivg_t);
        
        filter2D(binaryImage, Gx, binaryImage.depth(), gauss_t);
        filter2D(Gx, Gx, Gx.depth(), derivg);
        
        filter2D(binaryImage, Gy, binaryImage.depth(), gauss_kernel);
        filter2D(Gy, Gy, Gy.depth(), derivg_t);
        
        
    }
    
    void ls::adaptiveHistEqual(cv::Mat &src,cv::Mat &dst,double clipLimit)
    {
        Ptr<cv::CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(clipLimit);
        clahe->apply(src, dst);
    }
    vector<double> ls::linspace(double d1,double d2,int n)
    {
        int n1 = n - 1;
        //        double c = (d2 - d1) * (n1 - 1);
        vector<double> y;
        for (int i = 0; i < n1+1; i ++)
        {
            double tmp = d1 + i*(d2/n1) - (d1/n1)*i;
            y.push_back(tmp);
        }
        return y;
    }
    cv::Mat ls::repmat(cv::Mat &mat,int m,int n)
    {
        cv::Mat out(mat.rows*m,mat.cols*n,CV_8U);
        for (int i = 0; i < m; i ++)
        {
            for (int j = 0; j < n; j ++)
            {
                //                outx.push_back(mat);
                mat.copyTo(out);
            }
        }
        return out;
        
    }
    
    void ls::sub_vector(vector<double> &vec, int idx1,int idx2,vector<double> &out )
    {
        for (int i = 0; i < vec.size(); i ++)
        {
            if (i >= idx1 && i <= idx2)
                out.push_back(vec[i]);
        }
    }
    
    void substring( char *s, char ch1, char ch2, char *substr )
    {
        while( *s && *s++!=ch1 ) ;
        while( *s && *s!=ch2 ) *substr++=*s++ ;
        *substr='\0';
    }
    
    
    void ls::multiplyWithSameSize(cv::Mat &mat1,cv::Mat &mat2,cv::Mat &dst)
    {
        cv::Mat out(mat1.rows,mat1.cols,mat1.type());
        for (int i = 0; i < mat1.rows; i ++)
        {
            for (int j = 0; j < mat1.cols; j ++)
            {
                out.at<double>(i, j) = (mat1.at<double>(i, j) * mat2.at<double>(i,j));
            }
        }
        dst = out;
        //        out.release();
        
    }
    
    //    void ls::houghT(cv::Mat &src,cv::Mat &H,int theta_in,vector<double> &theta,vector<double> &rho)
    //    {
    //        double dtheta = 0.2;
    //        double drho = 1;
    //        int m = src.rows;
    //        int n = src.cols;
    //        int dnum = ceil(theta_in/dtheta) + 1;
    //        vector<double> theta_tmp,rho_t;
    //
    //        theta = linspace(-theta_in, 0, dnum);
    //        sub_vector(theta, 1, (int)theta.size()-2, theta_tmp);
    //        flip(theta_tmp, theta_tmp, 1);
    //        for (int j = 0; j < theta_tmp.size(); j ++)
    //        {
    //            theta.push_back((-1) * theta_tmp[j]);
    //        }
    //
    //        int ntheta = (int)theta.size();
    //        double D = sqrt((m -1) * (m -1) + (n - 1) * (n - 1));
    //        int q = ceil(D / drho);
    //        int nrho = 2 * q + 1;
    //        rho_t = linspace((-1)*q*drho, q*drho, nrho);
    //        for (int k = 0; k < rho_t.size(); k ++)
    //            rho.push_back(rho_t[k]);
    ////        vector<Point2i> locations;
    ////        int cn = countNonZero(src);
    ////        cv::Mat src1;
    ////        src.convertTo(src1, CV_8UC1);
    ////        findNonZero(src, locations);
    ////        vector<int>x,y;
    //        vector<double> x,y,val;
    //        findNonZeroIdx(src, x, y, val);
    ////        for (int len = 0; len < locations.size(); len ++)
    ////        {
    ////            x.push_back(locations[len].x );//、、、、、、、、、、、、、、、、、、
    ////            y.push_back(locations[len].y );
    ////            val.push_back((double)src.at<uchar>(locations[len].x, locations[len].y));
    ////        }
    //        for (int i = 0; i < x.size(); i ++)
    //        {
    //            x[i] = x[i] - 1;
    //            y[i] = y[i] - 1;
    //        }
    //
    //        //initialize
    //        H = cv::Mat::zeros(nrho, ntheta, CV_64F);
    //        cv::Mat h = cv::Mat::zeros(nrho,ntheta, CV_64F);
    //        cv::Mat val_matrix,rho_matrix,theta_matrix,x_matrix,y_matrix;
    //        cv::Mat cos_theta,sin_theta,mul_x_cos,mul_y_sin;//(theta_matrix.rows,theta_matrix.cols,CV_8U)
    //        cv::Mat x_fl_mat,y_fl_mat,val_fl_mat,theta_mat;
    //        vector<double> x_first_last,y_first_last,val_first_last;
    //        cv::Mat rho_bin_index(1000,ntheta,CV_64F);
    //        double first = 0.0,last = 0.0;
    //        double slope;
    //        int rho_bin;
    //        vector<double> tbi;
    //        cv::Mat tbi_mat,theta1,theta2;
    //        cv::Mat theta_bin_index ;
    //        vector<double> theta_col,rho_col,val_col;
    //
    //
    //        //to avoid excessive memory usage,process 1000 nonzero pixel values at a time
    //        for (int ii = 0; ii < ceil(val.size()/1000); ii ++)
    //        {
    //            first = ii*1000;
    //            last = MIN(first + 999, x.size());
    //
    //
    //            sub_vector(x, first, last, x_first_last);
    //            x_fl_mat = Mat(x_first_last);
    //            repeat(x_fl_mat, 1,ntheta,x_matrix);
    //
    //
    //            sub_vector(y, first, last, y_first_last);
    ////            cv::Mat y_fl_mat;//(1,1000,CV_8U);
    //            y_fl_mat = Mat(y_first_last);
    //            repeat(y_fl_mat,1, ntheta,y_matrix);
    //
    //
    //            sub_vector(val, first, last, val_first_last);
    ////            cv::Mat val_fl_mat;//(1,1000,CV_8U);
    //            val_fl_mat = Mat(val_first_last);
    //            repeat(val_fl_mat, 1, ntheta,val_matrix);
    //
    ////            for (int kk = 0; kk < val_matrix.rows; kk ++) {
    ////                for (int ll = 0; ll < val_matrix.cols; ll ++) {
    ////                    cout<<val_matrix.at<double>(kk, ll);
    ////                }
    ////            }
    //
    //
    ////            cv::Mat theta_mat;//(1,ntheta,CV_8U);
    //            theta_mat = Mat(theta);
    //            transpose(theta_mat, theta_mat);
    //            repeat(theta_mat,x_matrix.rows,1,theta_matrix);
    //
    //
    //            theta_matrix = CV_PI/180 * theta_matrix;
    //            theta1 = theta_matrix.clone();
    //            theta2 = theta_matrix.clone();
    //
    //            for (int num1 = 0; num1 < theta1.rows; num1 ++)
    //            {
    //                for (int num2 = 0 ; num2 < theta1.cols; num2 ++)
    //                {
    //                    theta1.at<double>(num1, num2) = cos((double)theta1.at<uchar>(num1,num2));
    //                }
    //            }
    //            cos_theta = theta1.clone();
    //
    //
    //            for (int num3 = 0; num3 < theta2.rows; num3 ++)
    //            {
    //                for (int num4 = 0 ; num4 < theta2.cols; num4 ++)
    //                {
    //                    theta2.at<double>(num3, num4) = sin((double)theta2.at<uchar>(num3,num4));
    //                }
    //            }
    //            sin_theta = theta2.clone();
    //
    //
    //
    //
    //            multiply(x_matrix, cos_theta, mul_x_cos);
    //            multiply(y_matrix, sin_theta, mul_y_sin);
    //            rho_matrix = mul_x_cos + mul_y_sin;
    //
    //
    //
    //            slope = (nrho - 0)/(rho[rho.size() - 1] - rho[0]);
    //
    //            for (int row = 0; row < rho_matrix.rows; row ++)
    //            {
    //                for (int col = 0; col < rho_matrix.cols; col ++)
    //                {
    //                    rho_bin = round(slope*((double)rho_matrix.at<uchar>(row,col) - rho[0])+1);
    //                    rho_bin_index.at<double>(row, col) = rho_bin;
    //                }
    //
    //            }
    //
    //
    ////            tbi = linspace(1, ntheta, ntheta);
    //            tbi_mat = Mat(theta);
    //            transpose(tbi_mat, tbi_mat);
    //            repeat(tbi_mat,x_matrix.rows,1,theta_bin_index);
    //
    //
    ////            val_col = getVector(val_matrix);
    ////            theta_col = getVector(theta_bin_index);
    ////            rho_col = getVector(rho_bin_index);
    //
    //            mat2Cvector(val_matrix, val_col);
    //            mat2Cvector(theta_bin_index, theta_col);
    //            mat2Cvector(rho_bin_index, rho_col);
    //
    //
    //            for (int num = 0; num < val_col.size(); num ++)
    //            {
    ////                cout<<rho_col[num]<<","<<theta_col[num]<<endl;
    ////                cout<<val_col[num]<<endl;
    //                h.at<double>((int)rho_col[num], (int)theta_col[num]) = val_col[num];
    //            }
    //
    //            H = H + h;
    //
    //            theta_col.clear();
    //            rho_col.clear();
    //            val_col.clear();
    //            x_first_last.clear();
    //            y_first_last.clear();
    //            val_first_last.clear();
    ////            locations.clear();
    //            tbi.clear();
    //
    //
    //        }
    ////        H = H / 255.0;
    //        x_fl_mat.release();
    //        val_matrix.release();
    //        theta_bin_index.release();
    //        rho_bin_index.release();
    //        tbi_mat.release();
    //        cos_theta.release();
    //        sin_theta.release();
    //        mul_x_cos.release();
    //        mul_y_sin.release();
    //        theta2.release();
    //        theta1.release();
    //        y_fl_mat.release();
    //        val_fl_mat.release();
    //        theta_mat.release();
    //        h.release();
    //        theta_tmp.clear();
    //        rho_t.clear();
    ////        locations.clear();
    //        x.clear();
    //        y.clear();
    //        val.clear();
    //
    //
    //
    //    }
    void ls::myHoughMex(cv::Mat &edge,cv::Mat &H, vector<double> theta,vector<double> rho)
    {
        int n = edge.cols;
        int m = edge.rows;
        
        H = cv::Mat::zeros((int)rho.size(), (int)theta.size(), CV_64F);
        int rl = floor((rho.size() - 1)/2+0.5);
        double r;
        int cc;
        for (int i = 0; i < m; i ++)
        {
            for (int j = 0; j < n; j ++)
            {
                //                cout<<edge.at<double>(i, j)<<endl;
                if(0 == edge.at<double>(i, j))
                    continue;
                for (int k = 0; k < theta.size(); k ++)
                {
                    r = (j+1)*cos(theta[k]) + (i+1)*sin(theta[k]);
                    //cc = round(r+rl+1);
                    cc=floor(r+rl+1+0.5)-1;
                    
                    H.at<double>(cc, k) = H.at<double>(cc, k) + 1;
                }
            }
        }
        
    }
    
    
    //convert the matrix to column vector
    vector<double> ls::getVector(const Mat &_t1f)
    {
        Mat t1f;
        _t1f.convertTo(t1f, CV_8U);
        return (vector<double>)(t1f.reshape(1, 1));
    }
    
    void ls::mat2Cvector(cv::Mat &mat,vector<double> &vec)
    {
        
        for (int i = 0; i < mat.cols; i ++)
        {
            for (int j = 0; j < mat.rows; j ++)
            {
                //                cout<<mat.at<double>(j, i)<<endl;
                vec.push_back(mat.at<double>(j, i));
            }
        }
    }
    //convert the matrix to the row vector
    void ls::mat2Rvector(cv::Mat &mat,vector<double> &vec)
    {
        for (int i = 0; i < mat.rows; i ++)
        {
            for (int j = 0; j < mat.cols; j ++)
            {
                vec.push_back(mat.at<double>(i, j));
            }
        }
    }
    
    int ls::sub2ind(cv::Mat &A,double pp,double qq)
    {
        vector<double> A_vec;
        //        mat2Cvector(A, A_vec);
        //        A_vec = getVector(A);
        A_vec = A.reshape(0,1);
        int ans;
        vector<double> com;
        if (A.dims == 2)
        {
            double value = (double)A.at<double>(pp, qq);
            for (int num = 0; num < A_vec.size(); num ++)
            {
                if(A_vec[num] == value)
                    ans = num;
            }
            
        }
        else if (3 == A.dims)
        {
        }
        return ans;
    }
    
    void ls::findNonZeroIdx(cv::Mat &src,vector<double> &x,vector<double> &y,vector<double> &val)
    {
        double tmp ;
        for (int i = 0; i < src.rows; i ++)
        {
            for (int j = 0; j < src.cols; j ++)
            {
                tmp = (double)src.at<uchar>(i, j);
                //                cout<<tmp<<endl;
                if (tmp > 0)
                {
                    x.push_back(i);
                    y.push_back(j);
                    val.push_back(tmp);
                    
                }
            }
        }
        
        
        //        int nl = src.rows;
        //        int nc = src.cols*src.channels();
        //        if (src.isContinuous()) {
        //            nc = nc*nl;
        //            nl = 1;
        //        }
        //        for (int j = 0; j < nl; j ++)
        //        {
        //            uchar *data = src.ptr<uchar>(j);
        //            for (int i = 0; i < nc; i ++)
        //            {
        //                *data++ = *data;
        //                if (data[i]) {
        //                    x.push_back(i);
        //                    y.push_back(j);
        //                    val.push_back(data[i]);
        //
        //                }
        //            }
        //        }
        
    }
    
    //    void ls::houghPixels(cv::Mat &f,vector<double> &theta,vector<double> &rho,double rbin,double cbin,vector<double> &r,vector<double> &c)
    //    {
    ////        findNonZero(f, locs);
    ////        vector<int>x,y;
    //        vector<double>x,y, val;
    //        findNonZeroIdx(f, x, y, val);
    ////        for (int len = 0; len < locs.size(); len ++)
    ////        {
    ////            x.push_back(locs[len].x );
    ////            y.push_back(locs[len].y );
    ////            val.push_back(f.at<int>(locs[len].x,locs[len].y));
    ////        }
    //        for (int i = 0; i < x.size(); i ++)
    //        {
    ////            val.push_back((double)f.at<uchar>(x[i], y[i]));
    //            x[i] = x[i] - 1;
    //            y[i] = y[i] - 1;
    //        }
    //
    ////        vector<double> rho_vec,theta_vec;
    ////        mat2vector(rho, rho_vec);
    ////        mat2vector(theta, theta_vec);
    //        double theta_c = theta[cbin] * CV_PI/180;
    //        vector<double> rho_xy;
    //        double xy;
    //        for (int i = 0; i < x.size(); i ++)
    //        {
    ////            cout<<x[i]<<endl;
    ////            cout<<y[i]<<endl;
    ////            cout<<cos(theta_c)<<","<<sin(theta_c)<<endl;
    //            xy = x[i]*cos(theta_c) + y[i]*sin(theta_c);
    //            rho_xy.push_back(xy);
    //        }
    //        int nrho = (int)rho.size();
    //        double slope = (nrho - 1)/(rho[nrho-1] - rho[0]);
    //        vector<int> rho_bin_idx;
    //        double slo;
    //        for (int i = 0; i < rho_xy.size(); i ++)
    //        {
    //            slo = slope * (rho_xy[i]- rho[0]) + 1;
    //            rho_bin_idx.push_back(round(slo));
    //
    //        }
    //        vector<int> idx;
    //        for (int i = 0; i < rho_bin_idx.size(); i ++)
    //        {
    //            if (rho_bin_idx[i] == rbin)
    //                    idx.push_back(i);
    //        }
    //        vector<double> rold,cold;
    //        for (int i = 0; i < idx.size(); i ++)
    //        {
    //            rold.push_back(x[idx[i]]+1);
    //            cold.push_back(y[idx[i]]+1);
    //        }
    //
    //        reSortHoughPixels(rold, cold, r, c);
    //
    //        x.clear();
    //        y.clear();
    //        val.clear();
    //        rho_xy.clear();
    //        rho_bin_idx.clear();
    //        idx.clear();
    //        rold.clear();
    //        cold.clear();
    //
    //    }
    void ls::matrixMul(cv::Mat &A,cv::Mat &B,cv::Mat &C)
    {
        
        int An = A.rows;
        if(An==0) return;
        int Am = A.cols;
        int Bn = B.rows;
        if(Bn==0) return;
        int Bm = B.cols;
        C = cv::Mat::zeros(An, Bm, A.type());
        if(Bm==0 || Am==0 || Am!=Bn) return;
        
        //        C.resize(An, vector<int>(Bm,0));
        for(int i=0;i<An;i++)
        {
            for(int j=0;j<Bm;j++)
            {
                for(int k=0;k<Am;k++)
                {
                    C.at<double>(i, j) += A.at<double>(i, k) * B.at<double>(k, j);//A[i][k] * B[k][j];
                }
            }
        }
    }
    
    void ls::reSortHoughPixels(vector<double> &r,vector<double> &c,vector<double> &rnew,vector<double> &cnew)
    {
        if (!r.size())
        {
            rnew = r;
            cnew = c;
        }
        
        else
        {
            double r_max,r_min,c_max,c_min;
            minMaxLoc(r, &r_min, &r_max);
            minMaxLoc(c, &c_min, &c_max);
            double r_range = r_max - r_min;
            double c_range = c_max - c_min;
            int sort_order;
            if(r_range > c_range)
            {
                sort_order = 1;
            }
            else
                sort_order = 2;
            vector<Point2f> peaks;
            Point2f pt;
            for (int i = 0; i < r.size(); i ++)
            {
                pt = cv::Point2f(r[i],c[i]);
                peaks.push_back(pt);
            }
            sortP(peaks, sort_order);
            for (int i = 0; i < peaks.size(); i ++)
            {
                rnew.push_back(peaks[i].x);
                cnew.push_back(peaks[i].y);
            }
            
        }
    }
    
    
    bool cmp_py(const Point2f &p1, const Point2f &p2);
    bool cmp_px(const Point2f &p1, const Point2f &p2);
    
    void ls::sortP(vector<Point2f> &pts,int sort_order)
    {
        if (1 == sort_order) {
            sort(pts.begin(), pts.end(), cmp_px);
        }
        else
            sort(pts.begin(), pts.end(), cmp_py);
        
    }
    
    bool cmp_py(const Point2f &p1, const Point2f &p2) {
        return p1.y < p2.y;
    }
    
    bool cmp_px(const Point2f &p1, const Point2f &p2) {
        return p1.x < p2.x;
    }
    
    //    vector<SpineLines> ls::houghlines_rc(cv::Mat &gray_img,vector<double> &theta,vector<double> &rho,vector<double> &rr,vector<double> &cc,double fillgap,double minlength)
    //    {
    //        vector<SpineLines> spine_lines;
    //        vector<double>r,c,x,diff_x,idx;
    //        double x1,x2,linelength,Inf;
    //        double dou = 0.0;
    //        cv::Mat point1(1,2,CV_64F),point2(1,2,CV_64F);
    //        double rbin,cbin;
    //        cv::Mat T(2,2,CV_64F);
    //        cv::Mat Tinv(2,2,CV_64F);
    //        double det;
    //        for (int i = 0; i < rr.size(); i ++)
    //        {
    //            rbin = rr[i];
    //            cbin = cc[i];
    //
    //            //get all pixels associated with HT cell
    //            houghPixels(gray_img, theta, rho, rbin, cbin, r, c);
    //            if (!r.size()) continue;
    //
    ////            vector<Point2f> pts;
    //
    //
    //            //rotate the pixel locations about (1,1) so that they lie approximately alone a vertical line
    //            double omega = (90 - theta[cbin]) * CV_PI/180;
    ////            cv::Mat T(2,2,CV_64F);
    //            T.at<double>(0, 0) = cos(omega);
    //            T.at<double>(0, 1) = sin(omega);
    //            T.at<double>(1, 0) = -sin(omega);
    //            T.at<double>(1, 1) = cos(omega);
    //
    //            det = determinant(T);
    //            for (int kk = 0; kk < T.rows; kk ++) {
    //                for (int ll = 0; ll < T.cols; ll ++) {
    //
    //                    cout<<T.at<double>(kk, ll)<<endl;
    //                }
    //            }
    //
    //
    //            Tinv.at<double>(0, 0) = T.at<double>(1, 1)/det;
    //            Tinv.at<double>(0, 1) = (-1)*T.at<double>(1, 0)/det;
    //            Tinv.at<double>(1, 0) = (-1)*T.at<double>(0, 1)/det;
    //            Tinv.at<double>(1, 1) = T.at<double>(0, 0)/det;
    //            for (int kk = 0; kk < Tinv.rows; kk ++) {
    //                for (int ll = 0; ll < Tinv.cols; ll ++) {
    //                    cout<<Tinv.at<double>(kk, ll)<<endl;
    //                }
    //            }
    //
    //            cv::Mat rc_mat((int)r.size(),2,CV_64F);
    ////            for (int j = 0; j < r.size(); j ++)
    ////            {
    ////                rc_mat.at<double>(j, 0) = r[j] - 1;
    ////                rc_mat.at<double>(j, 1) = c[j] - 1;
    ////            }
    //            for (int kk = 0; kk < rc_mat.cols; kk ++) {
    //                for (int ll = 0; ll < rc_mat.rows; ll ++) {
    //                    if(kk == 0)
    //                        rc_mat.at<double>(ll, kk) = r[ll] - 1;
    //                    else if (kk == 1)
    //                        rc_mat.at<double>(ll, kk) = c[ll] - 1;
    ////                     cout<<rc_mat.at<double>(ll, kk)<<endl;
    //                }
    //            }
    //
    //            cv::Mat xy;
    //
    //            matrixMul(rc_mat, T, xy);
    ////            for (int kk = 0; kk < xy.rows; kk ++) {
    ////                for (int ll = 0; ll < xy.cols; ll ++) {
    ////                    cout<<xy.at<double>(kk, ll)<<endl;
    ////                }
    ////            }
    //            for (int k = 0; k < xy.rows; k ++)
    //            {
    //                x.push_back(xy.at<double>(k, 0));
    //            }
    //            sort(x.begin(), x.end());
    ////            mat2Cvector(xy, x);
    ////            sort(x.begin(), x.end());
    //            rc_mat.release();
    ////            xy.release();
    //
    //
    //
    //            //find the gaps larger than the threshold
    //            for (int num = 0; num < x.size()-1; num++)
    //            {
    //                diff_x.push_back(x[num+1]-x[num]);
    //            }
    ////            flip(diff_x, diff_x, 1);
    //            Inf = 1.0/dou;
    //            diff_x.push_back(Inf);
    //            idx.push_back(0.0);
    //            for (int num1 = 0; num1 < diff_x.size(); num1 ++)
    //            {
    //                if(diff_x[num1] > fillgap)
    //                    idx.push_back(diff_x[num1]);
    //            }
    //            for (int num2 = 0; num2 < idx.size()-1; num2 ++)
    //            {
    //                x1 = x[idx[num2]+1];
    //                x2 = x[idx[num2+1]];
    //                linelength = x2 - x1;
    //                if (linelength >= minlength)
    //                {
    //                    point1.at<double>(0, 0) = x1;
    //                    point1.at<double>(0, 1) = rho[rbin];
    //                    point2.at<double>(0, 0) = x2;
    //                    point2.at<double>(0, 1) = rho[rbin];
    //
    //                    //rotate the end-point locations back to the original angle
    //
    //                    matrixMul(point1, Tinv, point1);
    //                    matrixMul(point2, Tinv, point2);
    //
    //                    SpineLines sl;
    //                    cout<<point1.at<double>(0, 0)<<endl;
    //                    sl.point1 = cv::Point2f(point1.at<double>(0, 0)+1,point1.at<double>(0, 1)+1);
    //                    sl.point2 = cv::Point2f(point2.at<double>(0, 0)+1,point2.at<double>(0, 1)+1);
    //                    sl.length = linelength;
    //                    sl.theta = theta[cbin];
    //                    sl.rho = rho[rbin];
    //                    spine_lines.push_back(sl);
    ////                    Tinv.release();
    ////                    T.release();
    //
    //                }
    //            }
    //
    //
    //            r.clear();
    //            c.clear();
    //            x.clear();
    //            diff_x.clear();
    //            idx.clear();
    //            xy.release();
    ////            point1.release();
    ////            point2.release();
    //
    //        }
    //        return spine_lines;
    //
    //    }
    
    
    vector<LSEG> ls::myHoughLines(cv::Mat &gray_img,vector<double> &theta,vector<double> &rho,vector<double> &rr,vector<double> &cc,
                                  vector<double> &nonzero_x,vector<double> &nonzero_y, double fillgap,double minlength)
    {
        double minlength_sq = minlength*minlength;
        double fillgap_sq = fillgap * fillgap;
        int rbin,cbin;
        cv::Point point1,point2;
        vector<LSEG> spine_lines;
        for (int i = 0; i < nonzero_x.size(); i ++)
        {
            nonzero_x[i] = nonzero_x[i] - 1;
            nonzero_y[i] = nonzero_y[i] - 1;
        }
        vector<double> r,c;
        for (int j = 0; j < rr.size(); j ++)
        {
            rbin = rr[j];
            cbin = cc[j];
            myHoughPixels(nonzero_x,nonzero_y,theta,rho,rbin,cbin,r,c);
            
            if(!r.size())
                continue;
            
            vector<Vec2i> xy;
            for (int k = 0; k < r.size(); k ++)
            {
                Vec2i tmp;
                tmp[0] = c[k];
                tmp[1] = r[k];
                xy.push_back(tmp);
            }
            vector<int> dist_sq;
            for (int num1 = 0; num1 < xy.size()-1; num1 ++)
            {
                Vec2i tmp;
                tmp[0] = (xy[num1+1][0] - xy[num1][0]) * (xy[num1+1][0] - xy[num1][0]);
                tmp[1] = (xy[num1+1][1] - xy[num1][1]) * (xy[num1+1][1] - xy[num1][1]);
                int sum = tmp[0] + tmp[1];
                dist_sq.push_back(sum);
                
            }
            vector<int> idx;
            idx.push_back(0);
            for (int num2 = 0; num2 < dist_sq.size(); num2 ++)
            {
                if(dist_sq[num2] > fillgap_sq)
                    idx.push_back(num2);
                
            }
            idx.push_back((int)xy.size());
            for (int p = 0; p < idx.size()-1; p ++)
            {
                // 			point1 = cv::Point(xy[idx[p]+1][0],xy[idx[p]+1][1]);
                // 			point2 = cv::Point(xy[idx[p+1]][0],xy[idx[p+1]][1]);
                
                point1 = cv::Point(xy[idx[p]][0],xy[idx[p]][1]);
                point2 = cv::Point(xy[idx[p+1]-1][0],xy[idx[p+1]-1][1]);
                
                double linelength = line_size(point1, point2);
                if((linelength*linelength) >= minlength_sq)
                {
                    LSEG spl;
                    spl.push_back(point1);
                    spl.push_back(point2);
                    //                    spl.point1 = point1;
                    //                    spl.point2 = point2;
                    //                    spl.length = linelength;
                    //                    spl.theta = theta[cbin];
                    //                    spl.rho = rho[rbin];
                    spine_lines.push_back(spl);
                }
            }
            r.clear();
            c.clear();
            xy.clear();
            dist_sq.clear();
            idx.clear();
            
        }
        return spine_lines;
        
        
    }
    
    
    void ls::myHoughPixels(vector<double> &nonzero_x,vector<double> &nonzero_y,vector<double> &theta,vector<double> &rho,int rbin,int cbin,vector<double> &r,vector<double> &c)
    {
        double theta_c = theta[cbin] * CV_PI/180;
        vector<double> rho_xy;
        double xy;
        for (int i = 0; i < nonzero_x.size(); i ++)
        {
            //            cout<<x[i]<<endl;
            //            cout<<y[i]<<endl;
            //            cout<<cos(theta_c)<<","<<sin(theta_c)<<endl;
            xy = nonzero_x[i]*cos(theta_c) + nonzero_y[i]*sin(theta_c);
            rho_xy.push_back(xy);
        }
        int nrho = (int)rho.size();
        double slope = (nrho - 1)/(rho[nrho-1] - rho[0]);
        vector<int> rho_bin_idx;
        double slo;
        for (int i = 0; i < rho_xy.size(); i ++)
        {
            slo = slope * (rho_xy[i]- rho[0]) + 1;
            rho_bin_idx.push_back(floor(slo+0.5));
            
        }
        vector<int> idx;
        for (int i = 0; i < rho_bin_idx.size(); i ++)
        {
            if (rho_bin_idx[i] == rbin)
                idx.push_back(i);
        }
        vector<double> rold,cold;
        for (int i = 0; i < idx.size(); i ++)
        {
            rold.push_back(nonzero_y[idx[i]]+1);
            cold.push_back(nonzero_x[idx[i]]+1);
        }
        
        reSortHoughPixels(rold, cold, r, c);
        
        rho_xy.clear();
        rho_bin_idx.clear();
        idx.clear();
        rold.clear();
        cold.clear();
        
        
    }
    
    
    
    void findMaxIdx(cv::Mat &mat,vector<cv::Point> &idx,double max)
    {
        for (int i = 0; i < mat.rows; i ++)
        {
            for (int j =  0; j < mat.cols; j ++)
            {
                if (max == mat.at<int>(i, j))
                {
                    cv::Point p = cv::Point(i,j);
                    idx.push_back(p);
                    
                }
            }
        }
        
    }
    void ls::houghPeaks(cv::Mat &H,int numpeaks,double thresh,const int *nhood_size,vector<double> &r,vector<double> &c)
    {
        bool done = false;
        cv::Mat hnew = H.clone();
        double max,min;
        vector<cv::Point> pts_;
        while (!done)
        {
            max = 0.0,min = 0.0;
            minMaxIdx(hnew, &min,&max);
            int nl = hnew.rows;
            int nc = hnew.cols;
            
            for (int i = 0; i < nl; i ++)
            {
                //                uchar *data = hnew.ptr<uchar>(i);
                for (int j = 0; j < nc; j ++)
                {
                    if (hnew.at<double>(i, j) == max)
                    {
                        pts_.push_back(cv::Point(i,j));
                    }
                }
            }
            int p = pts_[0].x;// x[0];
            int q = pts_[0].y;//y[0];
            
            if (hnew.at<double>(p,q) >= thresh)
            {
                //                r[r.size()-1] = p;
                //                c[c.size()-1] = q;
                r.push_back(p);
                c.push_back(q);
                //suppress this maximum and its close neighbours
                int p1 = p - (nhood_size[0]-1)/2;
                int p2 = p + (nhood_size[0]-1)/2;
                int q1 = q - (nhood_size[1]-1)/2;
                int q2 = q + (nhood_size[1]-1)/2;
                int p11 = MAX(p1, 1);
                int p22 = MIN(p2, H.rows);
                cv::Mat pp;//(rr,cc,CV_8S);
                cv::Mat qq;//(rr,cc,CV_8S);
                myMeshgrid(q1, q2, p11, p22, qq, pp);
                vector<double> pp_v,qq_v;
                mat2Cvector(pp, pp_v);
                mat2Cvector(qq, qq_v);
                
                //throw away neighbor coordinates that are out of bounds in the rho direction
                //                vector<double> po;
                
                
                //                vector<double>::iterator it;
                //                for (it = pp_v.begin();it != pp_v.end();)
                //                {
                //
                //                    if (*it < 0 || *it > H.rows)
                //                    {
                //                        it = pp_v.erase(it);
                //
                //                    }
                //                    else
                //                        ++it;
                //
                //                }
                //                vector<double>::iterator it1;
                //                for (it1 = qq_v.begin();it1 != qq_v.end();)
                //                {
                //
                //                    if (*it1 < 0 || *it1 > H.rows)
                //                    {
                //                        it1 = qq_v.erase(it1);
                //
                //                    }
                //                    else
                //                        ++it1;
                //
                //                }
                
                
                for (int j = 0; j < qq_v.size(); j ++)
                {
                    if(qq_v[j] < 0)
                    {
                        qq_v[j] = H.cols + qq_v[j];
                        pp_v[j] = H.rows - pp_v[j] + 1;
                        
                    }
                    if (qq_v[j] > H.cols)
                    {
                        qq_v[j] = qq_v[j] - H.cols;
                        pp_v[j] = H.rows - pp_v[j] + 1;
                    }
                }
                
                
                for (int num = 0; num < pp_v.size(); num ++)
                {
                    int ppp=floor(pp_v[num]+0.5);
                    int qqq=floor(qq_v[num]+0.5);
                    hnew.at<double>(ppp,qqq) = 0;
                }
                
                done = (r.size() == numpeaks);
                pts_.clear();
                
            }
            else
                done = true;
            
        }
        
    }
    
    void ls::myMeshgrid(int l1,int l2,int r1,int r2,cv::Mat &left,cv::Mat &right)
    {
        int cols = r2 - r1 + 1;
        int rows = l2 - l1 + 1;
        vector<double> lef = linspace(l1,l2,rows);
        vector<double> rig = linspace(r1,r2,cols);
        cv::Mat mat_lef = cv::Mat(lef);
        cv::Mat mat_rig = cv::Mat(rig);
        transpose(mat_rig, mat_rig);
        repeat(mat_lef, 1, cols, left);
        repeat(mat_rig, rows, 1, right);
        transpose(left, left);
        transpose(right, right);
        
    }
    
    
    void ls::getThetaRho(cv::Mat &edges,vector<double> &theta,vector<double> &rho,double theta_in,double dtheta,double drho)
    {
        
        int m = edges.rows;
        int n = edges.cols;
        
        int dnum = ceil(theta_in/dtheta) + 1;
        vector<double> theta_tmp,rho_t;
        
        
        theta = linspace(-theta_in, 0, dnum);
        sub_vector(theta, 0, (int)theta.size()-2, theta_tmp);
        flip(theta_tmp, theta_tmp, 1);
        for (int j = 0; j < theta_tmp.size(); j ++)
        {
            theta.push_back((-1) * theta_tmp[j]);
        }
        
        //    int ntheta = (int)theta.size();
        double D = sqrt((m -1) * (m -1) + (n - 1) * (n - 1));
        int q = ceil(D / drho);
        int nrho = 2 * q + 1;
        rho_t = linspace((-1)*q*drho, q*drho, nrho);
        for (int k = 0; k < rho_t.size(); k ++)
            rho.push_back(rho_t[k]);
        
        
    }
    
    int ls::WriteData(string fileName, cv::Mat& matData)
    {
        int retVal = 0;
        
        // ¥Úø™Œƒº˛
        ofstream outFile(fileName.c_str(), ios_base::out);  //∞¥–¬Ω®ªÚ∏≤∏«∑Ω Ω–¥»Î
        if (!outFile.is_open())
        {
            cout << "¥Úø™Œƒº˛ ß∞‹" << endl;
            retVal = -1;
            return (retVal);
        }
        
        // ºÏ≤Èæÿ’Û «∑ÒŒ™ø’
        if (matData.empty())
        {
            cout << "æÿ’ÛŒ™ø’" << endl;
            retVal = 1;
            return (retVal);
        }
        
        // –¥»Î ˝æ›
        for (int r = 0; r < matData.rows; r++)
        {
            for (int c = 0; c < matData.cols; c++)
            {
                //uchar data = matData.at<uchar>(r,c);  //∂¡»° ˝æ›£¨at<type> - type  «æÿ’Û‘™ÀÿµƒæﬂÃÂ ˝æ›∏Ò Ω
                double data=matData.at<double>(r,c);
                
                outFile << data << "\t" ;   //√ø¡– ˝æ›”√ tab ∏Ùø™
            }
            //outFile<<";";
            outFile << endl;  //ªª––
        }
        
        return (retVal);
    }
    
    void ls::bookSegmentStart(Mat &src1,vector<cv::Mat> &out_result,vector<cv::Mat> &corner)
    {
        Mat image;
        //image = src1.clone();
        resize(src1,image,Size(((float)src1.cols/(float)src1.rows)*krows,krows));
        //    imwrite("/Users/brdev/Documents/bookSegment Algorithms/Book-Identifier/code/main/test.jpg", image);
        ////    imshow("resize", image);
        ////    detailEnhance(image, image);
        ////    imshow("detail", image);
        //    Mat img = image.clone();
        //    Mat gray;
        //    cvtColor(image, gray, CV_BGR2GRAY);
        //    cv::Mat gray2 = gray.clone();
        //    adaptiveHistEqual(gray, gray,2.5);
        //
        ////    imshow("equalhist", gray);
        ////    cv::Mat gx,gy;
        ////    smoothGradient(gray, 0.6, gx, gy);
        ////    double high,low;
        //////    high = threshold(gray, gray, 1, 255, THRESH_OTSU);
        ////    low = 0.4*high;
        //////    selectThresh(gx, gy, 0.7, 0.4, high, low);
        //    cv::Mat edges;
        ////    double high = 300;
        //    blur(gray, gray, Size(5,5));
        ///////////
        //    cv::Mat ox,oy,mag,ang;
        //    Sobel(gray, ox, CV_32F, 0, 1 ,3);
        //    Sobel(gray, oy, CV_32F, 1, 0 ,3);
        //    blur(ox, ox, Size(3,3));
        //    blur(oy, oy, Size(3,3));
        //    cartToPolar(oy, ox, mag, ang);
        //
        /////////////
        //
        //    double low = 15;
        //
        //    Canny(gray, edges, low,8*low,3);//150,800
        //    imshow("canny", edges);
        ////    WriteData("/Users/brdev/Desktop/edge.txt", edges);
        //    vector<Vec4i> houghlines;
        //    double dtheta = 0.2;
        //    double drho = 1.0;
        //    double theta_in = 20;
        ////    src1 = src1/255.0;
        //    edges.convertTo(edges, CV_64F);
        //    WriteData("/Users/brdev/Desktop/edge.txt", edges);
        //    imshow("doublecanny", edges);
        //    imwrite("/Users/brdev/Desktop/edge.jpg", edges);
        //    vector<double> rho,theta;
        //    cv::Mat H;
        //    getThetaRho(edges, theta, rho, theta_in, dtheta, drho);
        //    vector<double> theta_new;
        //    for (int i = 0; i < theta.size(); i ++)
        //    {
        //        theta_new.push_back(theta[i]*CV_PI/180);
        //    }
        //    myHoughMex(edges, H, theta_new, rho);
        //    WriteData("/Users/brdev/Desktop/h.txt", H);
        //    int hood_size[] = {51,51};
        //    int numpeaks = 50;
        //    double H_min,H_max;
        //    minMaxLoc(H, &H_min,&H_max);
        //    double thresh = ceil(0.5*H_max);
        //    vector<double> r,c;
        //    houghPeaks(H, numpeaks, thresh, hood_size, r, c);
        //    vector<LSEG> spine_lines;
        //    double fillgap = gray2.cols/60.0;
        //    double minlength = gray2.rows/1.5;
        //    vector<double> nonzero_x,nonzero_y,val;
        //    findNonZeroIdx(gray, nonzero_y, nonzero_x, val);
        //    spine_lines = myHoughLines(gray2, theta, rho, r, c,nonzero_x,nonzero_y, fillgap, minlength);
        ////    for (int i = 0; i < spine_lines.size(); i ++)
        ////    {
        ////        cv::Point2f p1(spine_lines[i][0]);
        ////        cv::Point2f p2(spine_lines[i][1]);
        ////        line(image, p1, p2, Scalar(0,255,0),1.5);
        //////        imshow("spine", image);
        //////        waitKey(100);
        ////    }
        ////    imshow("spines_line", image);
        ////    imshow("houghline", linesx);
        //    LSEG baseline;
        ////    QuickSort(spine_lines, (int)spine_lines.size(), 1, baseline);
        ////    for (int i = 0; i < spine_lines.size(); i ++)
        ////    {
        ////        cv::Point2f p1(spine_lines[i][0]);
        ////        cv::Point2f p2(spine_lines[i][1]);
        ////        line(image, p1, p2, Scalar(0,255,0),1.5);
        //////        imshow("spine", image);
        //////        waitKey(100);
        ////    }
        //    vector<LSEG> spines;
        //    LSEG first,last;
        //    first.push_back(cv::Point(1,0));
        //    first.push_back(cv::Point(1,image.rows-1));
        //    last.push_back(cv::Point(image.cols-2,0));
        //    last.push_back(cv::Point(image.cols-2,image.rows-2));
        //    spines.push_back(first);
        //    for (int num = 0; num < spine_lines.size(); num ++)
        //    {
        //        spines.push_back(spine_lines[num]);
        //    }
        //    spines.push_back(last);
        //    vector<LSEG> res,other;
        //    vector<cv::Mat> book_cor;
        //    pr_detect(img,src1,ang,spines,res,other,1,baseline,book_cor);
        ////    for (int i = 0; i < spines.size(); i ++)
        ////    {
        ////        cv::Point2f p1(spines[i][0]);
        ////        cv::Point2f p2(spines[i][1]);
        ////        line(image, p1, p2, Scalar(0,255,0),1.5);
        //////        imshow("spine", image);
        //////        waitKey(1000);
        ////    }
        //    for (int i = 0; i < res.size(); i++)
        //    {
        //        //cv::Point center = get_gravity_center(result[i]);
        //        cv::Point2f uls = get_up_left(res[i], 3.0);
        //        cv::Point2f offsets = Point(3,3);
        //        cv::Point2f pcs = uls - offsets;
        //        circle(image, uls, 7, Scalar(0,255,0),-1,5,0);
        //        char *texts = new char[100];
        //        sprintf(texts, "%d",i+1);
        //        putText(image, texts, pcs, FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(0,255,255),0.2);
        //        //        imshow("marker", image);
        //        // imwrite("/Users/brdev/Desktop/test_out/marker.jpg", image);
        //        //waitKey(100);
        //        Mat as(500,500,CV_8U);
        //        Mat coordinates(4,2,CV_32FC1);
        //        vector<Point2f> pointss;
        //        //矫正
        //        warp_change(res[i],src1,img,as,pointss);
        //        coordinates.at<float>(0,0) = pointss[0].x;
        //        coordinates.at<float>(0,1) = pointss[0].y;
        //        coordinates.at<float>(1,0) = pointss[1].x;
        //        coordinates.at<float>(1,1) = pointss[1].y;
        //        coordinates.at<float>(2,0) = pointss[2].x;
        //        coordinates.at<float>(2,1) = pointss[2].y;
        //        coordinates.at<float>(3,0) = pointss[3].x;
        //        coordinates.at<float>(3,1) = pointss[3].y;
        //
        //        out_result.push_back(as);
        //        corner.push_back(coordinates);
        ////        imshow("a", out_result[i]);
        //
        //        delete [] texts;
        //    }
        ////    imshow("dian", image);
        
        
        
        
        
        
        
        
#if 1
        
        
        
        
        //#if 1
        //    Mat hsi;
        //    Histogrom1D h1;
        //    cvtColor(image, hsi, CV_BGR2HSV_FULL);
        //    imshow("hsi", hsi);
        //    vector<Mat> bgr;
        //    split(image, bgr);
        //    imshow("b", bgr[0]);
        //    imshow("g", bgr[1]);
        //    imshow("r", bgr[2]);
        //    int dim(256);
        //    Mat lut(1,&dim,CV_8U);
        //    for (int i = 0; i < 256; i ++) {
        //        lut.at<uchar>(i) = 1/(1+pow(2.62, i));
        //    }
        //    h1.applyLookUp(bgr[0], lut);
        //    h1.applyLookUp(bgr[1], lut);
        //    h1.applyLookUp(bgr[2], lut);
        //    imshow("b", bgr[0]);
        //    imshow("g", bgr[1]);
        //    imshow("r", bgr[2]);
        ////    Mat dst;
        //    merge(bgr,  dst);
        //    imshow("dst", dst);
        //
        //
        //#endif
        //
        
        
        
        //    Histogrom1D h1;
        //    h1.stretch(image, 200);
        //    imshow("stretch", image);
        //    equalizeHist(image, image);
        //    imshow("equal", image);
        
        //                #if 0
        //
        //                    vector<Mat> bgr;
        //                    split(image, bgr);
        //                    imshow("b", bgr[0]);
        //                    imshow("g", bgr[1]);
        //                    imshow("r", bgr[2]);
        //                    Histogrom1D hig;
        //                    Mat hist0(256,256,CV_8U,Scalar(255));
        //                    Mat hist1(256,256,CV_8U,Scalar(255));
        //                    Mat hist2(256,256,CV_8U,Scalar(255));
        //                    hist0 = hig.getHistogramImage(bgr[0]);
        //                    imshow("hist0", hist0);
        //                    hist1 = hig.getHistogramImage(bgr[1]);
        //                    imshow("hist1", hist1);
        //                    hist2 = hig.getHistogramImage(bgr[2]);
        //                    imshow("hist2", hist2);
        //                    vector<MatND> h;
        //                    h.push_back(hig.getHistogram(bgr[0]));
        //                    h.push_back(hig.getHistogram(bgr[1]));
        //                    h.push_back(hig.getHistogram(bgr[2]));
        //                    double min[3],max[3];
        //                    for(int num = 0;num < h.size();num ++){
        //                        minMaxLoc(h[num], &min[num], &max[num]);
        //                    }
        //                //    minMaxLoc(h[0], &min[0], &max[0]);
        //                //    minMaxLoc(h[1], &min[1], &max[1]);
        //                //    minMaxLoc(h[2], &min[2], &max[2]);
        //                    double max_tmp = 0;
        //                    int k = 0;
        //
        //
        //                    for (int i = 0; i < 3; i ++) {
        //                        if (max_tmp < max[i])
        //                        {
        //                            max_tmp = max[i];
        //                            k = i;
        //                        }
        //                    }
        //                //    for (int j = 0; j < 256; j ++)
        //                //    {
        //                //        if (h[k].at<float>(j) < 65) {
        //                //            h[k].at<float>(j) = 255;
        //                //        }
        //                //    }
        //                //    imshow("new1", h[k]);
        //                //    imshow("new2", bgr[k]);
        //
        //                //    //normalize(hist2, hist2);
        //                //    //imshow("normalize", hist2);
        //                //    MatND h = hig.getHistogram(hist2);
        //                    hig.stretch(bgr[k],10);
        //                    imshow("stretch", bgr[k]);
        //                    Mat dst;
        //                    //ImageAdjust( bgr[2], dst, 0, 0.5, 0.5, 1, 1);
        //                      //  return -1;
        //                    gammaCorrection(bgr[k], dst, 0.5);
        //                    imshow("dst", dst);
        //
        //                //    ContentFinder cf;
        //                //    cf.setHistogram(h);
        //                //    imshow("new", hist2);
        //
        //                #endif
        
        
        //#if 0
        //    Mat hsv, h, s, v;
        //    cvtColor(image, hsv, CV_BGR2HSV);
        //    imshow("hsv", hsv);
        //    Mat mask;
        //    vector<Mat> vec;
        //    split(hsv, vec);
        ////    for (int i = 0; i < vec[0].total(); i ++) {
        ////        if (0 == vec[0].at<uchar>(i)) {
        ////            vec[0].at<uchar>(i) = NULL;
        ////            vec[1].at<uchar>(i) = 0;
        ////            vec[2].at<uchar>(i) = 1;
        ////        }
        ////    }
        ////    Mat res;
        ////    merge(vec, res);
        ////    imshow("result", res);
        //
        //
        //    //imshow("h", vec[0]);
        //    //imshow("s", vec[1]);
        //    imshow("v", vec[2]);
        //    Mat dst;
        //    Histogrom1D hig;
        //    threshold(vec[2], dst, 150, 255, THRESH_BINARY);
        //    imshow("dst", dst);
        ////    cvtColor(image, image, CV_BGR2GRAY);
        ////    threshold(image, image, 1, 255, THRESH_OTSU);
        ////    imshow("image", image);
        //
        //#endif
        //#if 0
        //    Rect rectangle(0,0,image.cols-1,image.rows-1);
        //    Mat res;
        //    Mat bgModel,fgModel;
        //    grabCut(image, res, rectangle, bgModel, fgModel, 1,GC_INIT_WITH_RECT);
        //    imshow("res", res);
        //    compare(res, GC_BGD, res, CMP_GE);
        //    Mat foreground(image.size(),CV_8UC3,Scalar(255,255,255));
        //    image.copyTo(foreground, res);
        //    //res = res & 1;
        //    imshow("fg", foreground);
        //#endif
        //   // cvtColor(image, image, CV_RGB2HSV);
        //   // imshow("change", image);
        //    //    int dim(256);
        ////    Mat lut(1,&dim,CV_8U);
        ////    for (int i = 0; i < 256; i ++) {
        ////        lut.at<uchar>(i) = 255 - i;
        ////    }
        ////    h1.applyLookUp(laplace, lut);
        ////    imshow("fan", laplace);
        //#if 0
        //    Mat lap;
        //    Laplacian(image, lap, image.depth());
        //    Mat E(image.size(), image.type());
        //    lap.convertTo(E, image.type());
        //    Mat contrast;
        ////    Mat kernel(3,3,CV_32F,Scalar(-1));
        ////    kernel.at<float>(0) = 0;
        ////    kernel.at<float>(1) = -1;
        ////    kernel.at<float>(2) = 0;
        ////    kernel.at<float>(3) = -1;
        ////    kernel.at<float>(4) = 5;
        ////    kernel.at<float>(5) = -1;
        ////    kernel.at<float>(6) = 0;
        ////    kernel.at<float>(7) = -1;
        ////    kernel.at<float>(8) = 1;
        ////    filter2D(image, contrast, image.depth(), kernel);
        //    subtract(image, E, contrast);
        ////    Mat ress(contrast.size(),contrast.type());
        ////    cvSmooth(&contrast, &ress);
        //   // scaleAdd(laplace, -1 ,image , contrast);
        //    imshow("contrast", contrast);
        //    // imshow("image", image);
        ////    GaussianBlur(contrast, contrast, Size(3,3),0,0);
        ////    imshow("smooth", contrast);
        //
        //
        //    Mat laplace;
        //    Histogrom1D h1;
        //    Laplacian(contrast, laplace, contrast.depth());
        //    imshow("laplace", laplace);
        //    GaussianBlur(laplace, laplace, Size(3,3), 0,0);
        //    //blur(laplace, laplace, Size(3,3));
        //   // morphologyEx(laplace, laplace, MORPH_OPEN, Mat(3,3,CV_8U),Point(-1,-1),1);
        //
        //    Mat bil;
        //    int d = 3;
        //    bilateralFilter(laplace, bil, d, 2*d, d/2);
        //    imshow("bilate", bil);
        //
        //    threshold(laplace, laplace, 70, 255, THRESH_BINARY_INV);//40-60  45
        //    imshow("thresh", laplace);
        //    //morphologyEx(laplace, laplace, MORPH_DILATE, Mat(3,3,CV_8U),Point(-1,-1),1);
        //    //imshow("morph", laplace);
        //#endif
        //
        
        
        
        //    Mat binary;
        //    threshold(image, binary, 180, 255, THRESH_BINARY);//180
        //    imshow("bin", binary);
        //    Mat k = getGaborKernel(Size(3,3), 1, 0, 1, 0.1);
        //    filter2D(image, image, image.depth(), k);
        //    imshow("gabor", image);
        //    Mat h = getGaussianKernel(3, 50);
        //        Mat kernel(3,3,CV_32F,Scalar(-1));
        //        kernel.at<float>(1,1) = 8;
        //        filter2D(image, image, image.depth(), h);
        //        imshow("sharp", image);
        //    Mat edges,corners;
        //    MorphoFeatures mf;
        //    edges = mf.getEdges(image);
        //    corners = mf.getCorners(image);
        //    mf.drawOnImage(corners, image);
        //    imshow("edges",edges);
        //    imshow("corners", image);
        //    WatershedSegment ws;
        //    Mat markers = ws.getMarkers(image);
        //    ws.setMarkers(markers);
        //    ws.process(image);
        //    imshow("seg", image);
        //    Rect rect(10,10,380,380);
        //    Mat res;
        //    Mat bg,fg;
        //    grabCut(image, res, rect, bg, fg, 5,GC_INIT_WITH_RECT);
        //    compare(res, GC_PR_FGD, res, CMP_EQ);
        //    Mat foreground(image.size(),CV_8UC3,Scalar(255,255,255));
        //    image.copyTo(foreground,res);
        //    res = res & 1;
        //    imshow("grabcut", bg);
        //cvSmooth(&image, &dst);
        // imshow("smooth", dst);
        //    AdjustContrast(image, dst, 200);
        //    imshow("contrast", dst);
        
        // bilateralFilter(image, dst, 9, 50, 2.0);
        //imshow("bilteral", dst);
        
        
        Mat blur_out(image.rows,image.cols,CV_32FC1);
        //gray
        cvtColor(image, blur_out, CV_BGR2GRAY);
        //medianBlur(blur_out, blur_out, 3);
        // imshow("midian", blur_out);
        
        //    MatND hist = h1.getHistogram(blur_out);
        //    imshow("hist", hist);
        //    normalize(hist, hist);
        //    threshold(blur_out, blur_out, 100, 180, THRESH_BINARY);
        //    imshow("threshold", blur_out);
        // normalize(blur_out, blur_out);
        // imshow("norm", blur_out);
        //equalizeHist(blur_out, blur_out);
        // imshow("hist", blur_out);
        
        
        
        
        
        
        //#if 0
        //
        //   // Histogrom1D hig;
        //
        //    Mat stretch = h1.stretch(laplace,100);
        //    imwrite("/Users/brdev/Desktop/180_.jpg",stretch);
        //    imshow("stretch", stretch);
        //    Mat binary,color;
        //    cvtColor(stretch, color, CV_GRAY2RGB);
        //    imshow("color", color);
        //    ColorHistogram colorhist;
        //#endif
        // colorhist.colorReduce6(img);
        // imshow("reduce", img);
        
        
        // binary = hig.getHistogramImage(stretch);
        // imshow("histh", binary);
        //    Mat edge;
        //    Canny(stretch, edge, 100, 300);
        //    imshow("canny", edge);
        // threshold(stretch, binary, 100, 255, THRESH_BINARY);
        // imshow("bin", binary);
        //    Mat dilate;
        //    morphologyEx(binary, dilate, MORPH_DILATE, Mat(5,5,CV_8U),Point(-1,-1),1);
        //    imshow("dilate", dilate);
        //imshow("gray", blur_out);
        //    Ptr<xfeatures2d::SIFT> sift = xfeatures2d::SIFT::create();
        //    Ptr<xfeatures2d::SURF> surf = xfeatures2d::SURF::create();
        //    Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
        //    Ptr<ORB> orb = ORB::create();
        //    Ptr<MSER> mser = MSER::create();
        //Ptr<h>
        //#if 1
        ////    vector<KeyPoint> keyPoints;
        ////    Mat cornerStrength;
        ////    vector<cv::Point> corners;
        //    //HarrisDetector harris;
        //    Mat dst;//(image.rows,image.cols,CV_32F);
        //    Canny(image, dst, 200, 600);//350
        //    imshow("canny", dst);
        //    morphologyEx(dst, dst, MORPH_DILATE, Mat(5,5,CV_8U),Point(-1,-1),1);
        //    imshow("dilate", dst);
        ////    harris.detect(dst);
        ////    harris.getCorners(corners, 0.0001);
        ////    harris.drawOnImage(dst, corners);
        ////    imshow("harris", dst);
        ////    mser->detect(dst, keyPoints);
        ////    Mat harris_out;
        ////    drawKeypoints(dst, keyPoints, harris_out);
        ////    imshow("harris", harris_out);
        //
        //    vector<vector<Point> > contours;
        //    //轮廓查找,每一条直线
        //    findContours(dst, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        //    int k = 0;
        //    int max_area = 0;
        //    for (int i = 0; i < contours.size(); i ++)
        //    {
        //#if 0
        //        vector<RotatedRect> minRect(contours.size());
        //       // Mat tmp(4,2,CV_32FC1);
        //        Point2f rect_points[4];
        //        minRect[i] = minAreaRect(Mat(contours[i]));
        //        minRect[i].points( rect_points );
        //        for (int j = 0; j < 4; j ++)
        //        {
        //            circle(image, rect_points[j], 3, Scalar(0,255,0));
        //        }
        //#endif
        //#if 1
        //
        //        if (max_area < contourArea(contours[i]))
        //        {
        //            max_area = contourArea(contours[i]);
        //            k = i;
        //        }
        //#endif
        
        
        //    }
        //#if 1
        //    int max_py = 0,min_py = 10000,max_px = 0,min_px = 10000;//最大和最小点的初值
        //    for(int j = 0;j < contours[k].size();j++)
        //    {
        //
        //        //从返回的轮廓中找到最大点和最小点
        //        if(max_py < contours[k][j].y) max_py = contours[k][j].y;
        //        if(min_py > contours[k][j].y) min_py = contours[k][j].y;
        //        if(max_px < contours[k][j].x) max_px = contours[k][j].x;
        //        if(min_px > contours[k][j].x) min_px = contours[k][j].x;
        //    }
        //    Rect rect;
        //    cv::Point a = cv::Point(min_px,min_py);
        //    cv::Point b = cv::Point(max_px,max_py);
        //    cv::Point c = cv::Point(max_px,min_py);
        //    cv::Point d = cv::Point(min_px,max_py);
        //    rect.x = min_px;
        //    rect.y = min_py;
        //    rect.width = b.x - a.x;
        //    rect.height = b.y -a.y;
        //   // s = rect.area();
        //    circle(image, a, 3, Scalar(0,255,0));
        //    circle(image, b, 3, Scalar(0,255,0));
        //    circle(image, c, 3, Scalar(0,255,0));
        //    circle(image, d, 3, Scalar(0,255,0));
        //#endif
        //
        //    imshow("corners", image);
        //#endif
        //cornerHarris(blur_out, cornerStrength, 3, 3, 0.01);
        //goodFeaturesToTrack(blur_out, corners, 20, 0.001, 30);
        
        // Mat harrisCorners;
        // threshold(blur_out, blur_out, 1, 255, THRESH_BINARY);
        //adaptiveThreshold(blur_out, blur_out, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 5);
        //imshow("binary", blur_out);
        equalizeHist(blur_out, blur_out);
        //imshow("hist", blur_out);
        //    normalize(blur_out, blur_out);
        //    imshow("norm", blur_out);
        //中值滤波，去噪
        medianBlur(blur_out, blur_out, 3);
        // imshow("median", blur_out);
        //高斯模糊
        GaussianBlur(blur_out, blur_out, Size(7,7),0,0);
        //imshow("gauss", blur_out);
        //    //锐化，突出边界轮廓
        //    // Laplacian(img, img, img.depth());
        //    Mat kernel(5,5,CV_32F,Scalar(-1));
        //    kernel.at<float>(1,1) = 24;
        //    filter2D(blur_out, blur_out, blur_out.depth(), kernel);
        //    imshow("sharp", blur_out);
        //imwrite("/Users/brdev/Desktop/test_out/sharp.jpg", blur_out);
        
        
        ////    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        ////    morphologyEx(blur_out, blur_out, MORPH_BLACKHAT, element);
        ////    imshow("morph", blur_out);
        ////    imwrite("/Users/brdev/Desktop/test_out/morph.jpg", blur_out);
        
        Mat out_x,out_y;
        //边缘检测，sobel算子分别对水平方向和竖直方向进行检测
        Sobel(blur_out, out_x, CV_32F, 0, 1 ,3);
        
        //int low_threshold = 100;
        //Canny(blur_out, out_x, low_threshold, 3*low_threshold,3);
        //Canny(blur_out, out_y, low_threshold, 3*low_threshold,3);
        //imshow("sobel_x", out_x);
        // dilate(out_x, out_x, Mat(5,5,CV_8U),Point(-1,-1),1);
        //morphologyEx(out_x, out_x, MORPH_CLOSE, Mat(5,5,CV_8U),Point(-1,-1),1);
        //src,dst,MORPH_OPEN,Mat(3,3,CV_8U),Point(-1,-1),1
        //imshow("erode", out_x);
        //imshow("canny", out_y);
        
        Sobel(blur_out, out_y, CV_32F, 1, 0 ,3);
        
        //sobel检测边缘
#if 0
        Mat out = abs(out_x) + abs(out_y);
        imshow("sobel", out);
        double sobmin,sobmax;
        minMaxLoc(out, &sobmin,&sobmax);
        Mat sobel;
        out.convertTo(sobel, CV_8U,-255./sobmax,255);
        imshow("edge", sobel);
        imwrite("/Users/brdev/Desktop/test.jpg", sobel);
        Mat thres;
        threshold(sobel, thres, 200, 255, CV_THRESH_BINARY);
        imshow("thresh", thres);
#endif
        //imshow("sobel_y", out_y);
        // dilate(out_y, out_y, Mat(5,5,CV_8U),Point(-1,-1),1);
        // morphologyEx(out_y, out_y, MORPH_CLOSE, Mat(5,5,CV_8U),Point(-1,-1),1);
        //imshow("erode", out_y);
        
        blur(out_x, out_x, Size(3,3));//盒型滤波，boxfilter
        //imshow("blur_x", out_x);
        // Canny(out_x, out_x, 50, 300);
        
        blur(out_y, out_y, Size(3,3));
        // Canny(out_y, out_y, 50, 300);
        //imshow("blur_y", out_y);
        //blur(out, out, Size(3,3));
        
        Mat magnitude,angle;//幅度和角度
        
        cartToPolar(out_y, out_x, magnitude, angle);//计算二维向量的长度、角度或者两者， magnitude：存储向量长度输出数组
        //cartToPolar(out, out,magnitude, angle);
        
        vector<Vec4i> lSegs,outlines;
        
        //线检测
        Ptr<LineSegmentDetector> ld = createLineSegmentDetector();
        ld->detect(blur_out, lSegs);
        
        
        Mat floodzy = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存竖直摆放的书籍边界点
        Mat floodzx = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存水平摆放的书籍边界点
        Mat floodzr = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存向右倾斜45度摆放的书籍边界点
        Mat floodzl = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存向左倾斜45度摆放的书籍边界点
        vector<LSEG> oolinesy,oolinesr,oolinesl,oolinesx;
        //float thresh = 0.1 * image.rows;
        for(int i=0;i<lSegs.size();i++)
        {
            cv::Point pt1 = cv::Point(lSegs[i][0],lSegs[i][1]);//将lSegs中的每个元素的前两个赋给pt1
            cv::Point pt2 = cv::Point(lSegs[i][2],lSegs[i][3]);////将lSegs中的每个元素的后两个赋给pt2
            //double length = line_size(pt1,pt2);
            //cout<<"length: "<<length[i]<<endl;
            
            if(line_size(pt1,pt2) > 86)//95compute the distance of two points 85
            {
                //            cv::Point2f v,fv;
                //            //两点组成直线的方向向量
                //            v.x = ((float)pt2.x - (float)pt1.x)/length;
                //            v.y = ((float)pt2.y - (float)pt1.y)/length;
                //            //反向量
                //            fv.x = -v.x;
                //            fv.y = -v.y;
                //            cv::Point p1,p2;
                //            //计算投影点
                //            p1 = prpoint(pt1, 0, fv);
                //            p2 = prpoint(pt2, 0, v);
                //            LSEG l_tmp;
                //            l_tmp.push_back(pt1);
                //            l_tmp.push_back(pt2);
                
                //计算夹角,将所有直线分类，分成四个方向
                float a = line_jiao(pt1,pt2);
                
                if (a < 22.5 || a > 157.5)//近似为水平方向，0
                {
                    line(floodzx, pt1, pt2, Scalar(255,255,255),2,3);
                    
                    // oolinesx.push_back(l_tmp);
                    // l_tmp.clear();
                }
                else if(a > 22.5  && a <= 67.5)//近似为45度方向
                    //else if (a >= 22.5 && a < 67.5)
                {
                    line(floodzr, pt1, pt2, Scalar(255,255,255),2,3);
                    //oolinesr.push_back(l_tmp);
                    //l_tmp.clear();
                    
                }
                else if(a >= 112.5 && a < 157.5)//近似为－45度方向
                    //else if (a >= 112.5 && a <= 157.5)
                {
                    line(floodzl, pt1, pt2, Scalar(255,255,255),2,3);
                    
                    //oolinesl.push_back(l_tmp);
                    //l_tmp.clear();
                }
                else /*if(a >= 80 && a <= 100)*///a>67.5 && a<112.5,近似呈竖直方向，既90度方向
                    //else
                {
                    line(floodzy, pt1, pt2, Scalar(255,255,255),2,3);
                    
                    
                    //oolinesy.push_back(l_tmp);
                    // l_tmp.clear();
                    //n ++;
                    
                }
                line(image, pt1, pt2, Scalar(0,255,0));
            }
            
        }
        
        imshow("y", floodzy);
        imshow("x", floodzx);
        imwrite("/Users/brdev/Desktop/test_out/dx.jpg", floodzx);
        imshow("r", floodzr);
        imshow("l", floodzl);
        //    imshow("r1", r225);
        //    imshow("r2", r675);
        //    imshow("l1", l1125);
        //    imshow("l2", l1575);
        imshow("dddddd", image);
        // imwrite("/Users/brdev/Desktop/test_out/ddd2.jpg", image);
        //    //cout<<"n:"<<n<<endl;
        
        Mat oo_out = image.clone();
        vector<LSEG> result,rest;
        
        //轮廓查找并拟合成一条直线,最接近的
        //90度 y,0;
        //0度 x,1;
        //45度 m,2;
        //135度 n,3;
        //22.5  4
        //67.5 5
        //112.5 6
        //157.5 7
        //////    切割，从四个方向
        
        
        
#if 0
        vector<LSEG> lines_y,lines_x,lines_r,lines_l;
        double t = (double)getTickCount();
        lines_y = line_preprocess(oolinesy, 0);
        //           for (int i = 0; i < lines_y.size(); i ++)
        //           {
        //                line(image, lines_y[i][0], lines_y[i][1], Scalar(128,128,255));
        //               circle(image, lines_y[i][0], 3, Scalar(255,0,0));
        //               circle(image, lines_y[i][1], 3, Scalar(0,255,0));
        //                imshow("findline", image);
        //                waitKey();
        //           }
        lines_x = line_preprocess(oolinesx, 1);
        //    for (int i = 0; i < lines_x.size(); i ++)
        //    {
        //        line(image, lines_x[i][0], lines_x[i][1], Scalar(128,128,255));
        //        circle(image, lines_x[i][0], 3, Scalar(255,0,0));
        //        circle(image, lines_x[i][1], 3, Scalar(0,255,0));
        //        imshow("findline", image);
        //        waitKey();
        //    }
        lines_r = line_preprocess(oolinesr, 2);
        lines_l = line_preprocess(oolinesl, 3);
        cout<<"time of line process:"<<((double)getTickCount()-t)/(double)getTickFrequency()<<endl;
        double t1 = (double)getTickCount();
        pr_detect(oo_out,src1,angle,lines_x,result,1,oolinesx[0]);
        pr_detect(oo_out,src1,angle,lines_y,result,0,oolinesx[0]);
        pr_detect(oo_out,src1,angle,lines_r,result,2,oolinesx[0]);
        pr_detect(oo_out,src1,angle,lines_l,result,3,oolinesx[0]);
        cout<<"time of pr_detect:"<<((double)getTickCount()-t1)/(double)getTickFrequency()<<endl;
#endif
        
        
        
#if TEST
        
        
        
        vector<cv::Mat> book_cordinates;
        double t1 = (double)getTickCount();
        findline(floodzy,oolinesy,0);
        //       for (int i = 0; i < oolinesy.size(); i ++)
        //       {
        //            line(oo_out, oolinesy[i][0], oolinesy[i][1], Scalar(128,128,255));
        //            imshow("findline", oo_out);
        //            waitKey();
        //       }
        findline(floodzx,oolinesx,1);
        //           for (int i = 0; i < oolinesx.size(); i ++)
        //           {
        //                line(image, oolinesx[i][0], oolinesx[i][1], Scalar(128,128,255));
        //                imshow("findline", image);
        //                waitKey();
        //           }
        findline(floodzr,oolinesr,2);
        findline(floodzl,oolinesl,3);
        cout<<"time of findline:"<<((double)getTickCount()-t1)/(double)getTickFrequency()<<endl;
        double t = (double)getTickCount();
        pr_detect(oo_out,src1,angle,oolinesx,result,rest,1,oolinesx[0],book_cordinates);
        if (0 == oolinesx.size())
        {
            LSEG bline;
            bline.push_back(cv::Point(-20,0));
            bline.push_back(cv::Point(500,0));
            oolinesx.push_back(bline);
        }
        pr_detect(oo_out,src1,angle,oolinesy,result,rest,0,oolinesy[0],book_cordinates);
        pr_detect(oo_out,src1,angle,oolinesr,result,rest,2,oolinesr[0],book_cordinates);
        pr_detect(oo_out,src1,angle,oolinesl,result,rest,3,oolinesl[0],book_cordinates);
        cout<<"time of pr_detect:"<<((double)getTickCount()-t)/(double)getTickFrequency()<<endl;
        //    black(image, book_cordinates);
        //    imshow("notbook", image);
        //    nonbook_extract(image);
        //    imshow("nonbook", image);
        
        
#endif
        
        
        
        
        
        
        vector<Mat> res_new;
        vector<vector<LSEG>> lls;
#if 1
        for (int i = 0; i < result.size(); i++)
        {
            //cv::Point center = get_gravity_center(result[i]);
            cv::Point2f ul = get_up_left(result[i], 3.0);
            cv::Point2f offset = Point(3,3);
            cv::Point2f pc = ul - offset;
            circle(image, ul, 7, Scalar(0,255,0),-1,5,0);
            char *text = new char[100];
            sprintf(text, "%d",i+1);
            putText(image, text, pc, FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(0,255,255),0.2);
            //        imshow("marker", image);
            // imwrite("/Users/brdev/Desktop/test_out/marker.jpg", image);
            //waitKey(100);
            Mat a(500,500,CV_8U);
            Mat coordinate(4,2,CV_32FC1);
            vector<Point2f> points;
            //矫正
            warp_change(result[i],src1,oo_out,a,points);
            coordinate.at<float>(0,0) = points[0].x;
            coordinate.at<float>(0,1) = points[0].y;
            coordinate.at<float>(1,0) = points[1].x;
            coordinate.at<float>(1,1) = points[1].y;
            coordinate.at<float>(2,0) = points[2].x;
            coordinate.at<float>(2,1) = points[2].y;
            coordinate.at<float>(3,0) = points[3].x;
            coordinate.at<float>(3,1) = points[3].y;
            
            out_result.push_back(a);
            corner.push_back(coordinate);
            imshow("a", out_result[i]);
            
            delete [] text;
        }
        imshow("dadian", image);
#endif
        
        
        
        image.release();
        blur_out.release();
        out_x.release();
        out_y.release();
        //out.release();
        magnitude.release();
        angle.release();
        floodzl.release();
        floodzr.release();
        floodzx.release();
        floodzy.release();
        oo_out.release();
        
#endif
        
        
    }
    
    
    void ls::black(Mat &src1,vector<Mat> &coordinates)
    {
        
        
        // Mat src = src1.clone();
        for(int i = 0;i<coordinates.size();i++)
        {
            Mat img1 = Mat::zeros(src1.rows,src1.cols,CV_8UC1);
            
            line(img1, Point(coordinates[i].at<float>(0,0),coordinates[i].at<float>(0,1)), Point(coordinates[i].at<float>(1,0),coordinates[i].at<float>(1,1)), Scalar(255,0,0),2,8);
            line(img1, Point(coordinates[i].at<float>(0,0),coordinates[i].at<float>(0,1)), Point(coordinates[i].at<float>(2,0),coordinates[i].at<float>(2,1)), Scalar(255,0,0),2,8);
            line(img1, Point(coordinates[i].at<float>(3,0),coordinates[i].at<float>(3,1)), Point(coordinates[i].at<float>(1,0),coordinates[i].at<float>(1,1)), Scalar(255,0,0),2,8);
            line(img1, Point(coordinates[i].at<float>(3,0),coordinates[i].at<float>(3,1)), Point(coordinates[i].at<float>(2,0),coordinates[i].at<float>(2,1)), Scalar(255,0,0),2,8);
            Point seed1;
            seed1 = Point((coordinates[i].at<float>(0,0)+coordinates[i].at<float>(1,0)+coordinates[i].at<float>(2,0)+coordinates[i].at<float>(3,0))*0.25,(coordinates[i].at<float>(0,1)+coordinates[i].at<float>(1,1)+coordinates[i].at<float>(2,1)+coordinates[i].at<float>(3,1))*0.25);
            floodFill(img1, seed1, Scalar(125));
            threshold(img1, img1, 1, 255, CV_THRESH_BINARY_INV);
            cvtColor(img1, img1, CV_GRAY2BGR);
            //Mat tmp;
            src1 = src1 & img1;
            //imshow("src1", src1);
            
            img1.release();
        }
        // src1 = src1 & src;
        
    }
    cv::Point ls::get_gravity_center(LSEG &pr)
    {
        cv::Point grav_center;
        LINE l1 = makeline(pr[0], pr[3]);
        LINE l2 = makeline(pr[1], pr[2]);
        //    double height = line_size(pr[0], pr[1]);
        lineintersect(l1, l2, grav_center);
        return grav_center;
        
        
    }
    cv::Point2f ls::get_up_left(LSEG &pr,float os)
    {
        Point2f middle,up_left;
        //float offset = 3.0;
        middle.x = (pr[0].x + pr[2].x)/2;
        middle.y = (pr[0].y + pr[2].y)/2;
        float jiao = line_jiao1(pr[0], pr[2]);
        if (jiao < 10 || jiao > 170) {//水平
            up_left.x = middle.x;
            up_left.y = middle.y - os;
        }
        else if (jiao > 80 && jiao < 100){//竖直
            up_left.x = middle.x - os;
            up_left.y = middle.y;
        }
        
        return up_left;
        
        
    }
    
    //void ls::markcounter()
    //{
    //    circle(<#InputOutputArray img#>, <#Point center#>, <#int radius#>, <#const Scalar &color#>)
    //
    //
    //
    //}
    void ls::line_conn(vector<LSEG> &oolines)
    {
        vector<LINE> l;
        for (int mm = 0; mm < oolines.size(); mm ++)
        {
            l.push_back(makeline(oolines[mm][0], oolines[mm][1]));
        }
        vector<LSEG> les;
        vector<int> index;
        getlines2(l, les,index);
        for (int nn = 0; nn < oolines.size() - 2; nn ++)
        {
            for (int ll = 0; ll < index.size(); ll ++)
            {
                if (nn == index[ll])
                {
                    oolines[nn] = les[nn];
                    oolines[nn + 1] = oolines[nn + 2];
                }
            }
        }
        
        
    }
    
    void ls::get_angle(Mat &blur_out,Mat &angle,Mat &magnitude)
    {
        Mat out_x,out_y,out;
        //边缘检测，sobel算子分别对水平方向和竖直方向进行检测
        Sobel(blur_out, out_x, CV_32F, 0, 1 ,3);
        //int low_threshold = 100;
        //Canny(blur_out, out_x, low_threshold, 3*low_threshold,3);
        //Canny(blur_out, out_y, low_threshold, 3*low_threshold,3);
        imshow("sobel_x", out_x);
        // dilate(out_x, out_x, Mat(5,5,CV_8U),Point(-1,-1),1);
        //morphologyEx(out_x, out_x, MORPH_CLOSE, Mat(5,5,CV_8U),Point(-1,-1),1);
        //src,dst,MORPH_OPEN,Mat(3,3,CV_8U),Point(-1,-1),1
        imshow("erode", out_x);
        //imshow("canny", out_y);
        
        Sobel(blur_out, out_y, CV_32F, 1, 0 ,3);
        imshow("sobel_y", out_y);
        // dilate(out_y, out_y, Mat(5,5,CV_8U),Point(-1,-1),1);
        // morphologyEx(out_y, out_y, MORPH_CLOSE, Mat(5,5,CV_8U),Point(-1,-1),1);
        imshow("erode", out_y);
        
        blur(out_x, out_x, Size(5,5));//盒型滤波，boxfilter
        imshow("blur_x", out_x);
        
        blur(out_x, out_x, Size(5,5));
        imshow("blur_y", out_y);
        //blur(out, out, Size(3,3));
        
        //    Mat magnitude;//幅度和角度
        
        cartToPolar(out_y, out_x, magnitude, angle);//计算二维向量的长度、角度或者两者， magnitude：存储向量长度输出数组
        
        
        
    }
    
    vector<vector<LSEG>> ls::lines_detection(Mat &a,double thresh)
    {
        Mat image;
        resize(a,image,Size(((float)a.cols/(float)a.rows)*krows,krows));
        //imwrite("/Users/brdev/Desktop/test.bmp", image);
        // imshow("resize", image);
        Mat blur_out;
        //convert rgb to gray image
        cvtColor(image, blur_out, CV_RGB2GRAY);
        imshow("gray", blur_out);
        //直方图归一化，增强图像
        equalizeHist(blur_out, blur_out);
        // imshow("equalizeHist", blur_out);
        //中值滤波，去噪
        // medianBlur(blur_out, blur_out, 3);
        // imshow("median", blur_out);
        //高斯模糊
        // GaussianBlur(blur_out, blur_out, Size(5,5),0,0);
        // imshow("gauss", blur_out);
        
        //    //锐化，突出边界轮廓
        //    // Laplacian(img, img, img.depth());
        Mat kernel(3,3,CV_32F,Scalar(-1));
        kernel.at<float>(1,1) = 8;
        filter2D(blur_out, blur_out, blur_out.depth(), kernel);
        imshow("sharp", blur_out);
        imwrite("/Users/brdev/Desktop/test_out/sharp.jpg", blur_out);
        //b =blur_out.clone();
        
        vector<Vec4i> lines;
        Ptr<LineSegmentDetector> ld = createLineSegmentDetector();
        ld->detect(blur_out, lines);
        Mat floodzy = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存竖直摆放的书籍边界点
        Mat floodzx = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存水平摆放的书籍边界点
        Mat floodzr = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存向右倾斜45度摆放的书籍边界点
        Mat floodzl = Mat::zeros(blur_out.rows, blur_out.cols, CV_8U);//保存向左倾斜45度摆放的书籍边界点
        //  Mat aa = image.clone();
        
        //vector<LSEG> zlinesx,zlinesy,zlinesm,zlinesn;//point
        // vector<double> length;
        
        for(int i=0;i<lines.size();i++)
        {
            cv::Point pt1 = cv::Point(lines[i][0],lines[i][1]);//将lSegs中的每个元素的前两个赋给pt1
            cv::Point pt2 = cv::Point(lines[i][2],lines[i][3]);////将lSegs中的每个元素的后两个赋给pt2
            //length.push_back(line_size(pt1,pt2));
            //cout<<"length: "<<length[i]<<endl;
            
            
            if(line_size(pt1,pt2) > thresh)//65compute the distance of two points
            {
                cv::Point2f v,fv;
                //两点组成直线的方向向量
                v.x = ((float)pt2.x - (float)pt1.x)/line_size(pt1,pt2);
                v.y = ((float)pt2.y - (float)pt1.y)/line_size(pt1,pt2);
                //反向量
                fv.x = -v.x;
                fv.y = -v.y;
                cv::Point p1,p2;
                //计算投影点
                p1 = prpoint(pt1, 0, fv);
                p2 = prpoint(pt2, 0, v);
                //计算夹角,将所有直线分类，分成四个方向
                float a = line_jiao(pt1,pt2);
                if (a < 22.5 || a > 157.5)//近似为水平方向，0
                {
                    line(floodzx, p1, p2, Scalar(255,255,255),3,8);
                }
                else if(a > 22.5  && a <= 67.5)//近似为45度方向
                {
                    line(floodzr, p1, p2, Scalar(255,255,255),3,8);
                }
                else if(a >= 112.5 && a < 157.5)//近似为－45度方向
                {
                    line(floodzl, p1, p2, Scalar(255,255,255),3,8);
                }
                else    //a>67.5 && a<112.5,近似呈竖直方向，既90度方向
                {
                    line(floodzy, p1, p2, Scalar(255,255,255),3,8);
                }
                
                
            }
        }
        
        vector<LSEG> oolinesy,oolinesr,oolinesl,oolinesx;
        vector<vector<LSEG>> lines_all;
        //轮廓查找并拟合成一条直线,最接近的
        findline(floodzy,oolinesy,0);
        lines_all.push_back(oolinesy);
        
        
        
        findline(floodzx,oolinesx,1);
        lines_all.push_back(oolinesx);
        
        
        findline(floodzr,oolinesr,2);
        lines_all.push_back(oolinesr);
        
        
        findline(floodzl,oolinesl,3);
        lines_all.push_back(oolinesl);
        
        
        
        return lines_all;
        
    }
    
    
    //生成变换矩阵
    PerspectiveTransform PerspectiveTransform::PerspectiveTransformM(float inA11, float inA21,float inA31, float inA12,
                                                                     float inA22, float inA32,float inA13, float inA23,float inA33)
    {
        PerspectiveTransform a;
        a.a11 = inA11;
        a.a12 = inA12;
        a.a13 = inA13;
        a.a21 = inA21;
        a.a22 = inA22;
        a.a23 = inA23;
        a.a31 = inA31;
        a.a32 = inA32;
        a.a33 = inA33;
        return a;
    }
    //四边形变换为四边形
    PerspectiveTransform PerspectiveTransform::quadrilateralToQuadrilateral(float x0, float y0, float x1, float y1,
                                                                            float x2, float y2, float x3, float y3,
                                                                            float x0p, float y0p, float x1p, float y1p,
                                                                            float x2p, float y2p,float x3p, float y3p)
    {
        //首先，将四边形变换到矩形
        PerspectiveTransform qToS = PerspectiveTransform::quadrilateralToSquare(x0, y0, x1, y1, x2, y2, x3, y3);
        //再由矩形变换到四边形
        PerspectiveTransform sToQ = PerspectiveTransform::squareToQuadrilateral(x0p, y0p, x1p, y1p, x2p, y2p, x3p, y3p);
        return sToQ.times(qToS);//qToS左乘以sToQ
    }
    //矩形变换到四边形
    PerspectiveTransform PerspectiveTransform::squareToQuadrilateral(float x0, float y0, float x1, float y1,
                                                                     float x2,float y2, float x3, float y3)
    {
        float dx3 = x0 - x1 + x2 - x3;
        float dy3 = y0 - y1 + y2 - y3;
        if (dx3 == 0.0f && dy3 == 0.0f)//变换平面与原来平行
        {
            PerspectiveTransform result(PerspectiveTransformM(x1 - x0, x2 - x1, x0, y1 - y0, y2 - y1, y0, 0.0f,
                                                              0.0f, 1.0f));
            return result;
        }
        else//不平行
        {
            float dx1 = x1 - x2;
            float dx2 = x3 - x2;
            float dy1 = y1 - y2;
            float dy2 = y3 - y2;
            float denominator = dx1 * dy2 - dx2 * dy1;
            float a13 = (dx3 * dy2 - dx2 * dy3) / denominator;
            float a23 = (dx1 * dy3 - dx3 * dy1) / denominator;
            PerspectiveTransform result(PerspectiveTransformM(x1 - x0 + a13 * x1, x3 - x0 + a23 * x3, x0, y1 - y0
                                                              + a13 * y1, y3 - y0 + a23 * y3, y0, a13, a23, 1.0f));
            return result;
        }
    }
    //四边形变换到矩形，需要借助矩形变换四边形
    PerspectiveTransform PerspectiveTransform::quadrilateralToSquare(float x0, float y0, float x1, float y1,
                                                                     float x2,float y2, float x3, float y3)
    {
        // Here, the adjoint serves as the inverse:
        return squareToQuadrilateral(x0, y0, x1, y1, x2, y2, x3, y3).buildAdjoint();
    }
    //求伴随矩阵（不能忘记转置）
    PerspectiveTransform PerspectiveTransform::buildAdjoint()
    {
        // Adjoint is the transpose of the cofactor matrix:
        PerspectiveTransform result(PerspectiveTransformM(a22 * a33 - a23 * a32, a23 * a31 - a21 * a33, a21 * a32
                                                          - a22 * a31, a13 * a32 - a12 * a33, a11 * a33 - a13 * a31,
                                                          a12 * a31 - a11 * a32, a12 * a23 - a13 * a22,
                                                          a13 * a21 - a11 * a23, a11 * a22 - a12 * a21));
        return result;
    }
    //两矩阵相乘
    PerspectiveTransform PerspectiveTransform::times(PerspectiveTransform other)
    {
        PerspectiveTransform result(PerspectiveTransformM(a11 * other.a11 + a21 * other.a12 + a31 * other.a13,
                                                          a11 * other.a21 + a21 * other.a22 + a31 * other.a23, a11 * other.a31 + a21 * other.a32 + a31
                                                          * other.a33, a12 * other.a11 + a22 * other.a12 + a32 * other.a13, a12 * other.a21 + a22
                                                          * other.a22 + a32 * other.a23, a12 * other.a31 + a22 * other.a32 + a32 * other.a33, a13
                                                          * other.a11 + a23 * other.a12 + a33 * other.a13, a13 * other.a21 + a23 * other.a22 + a33
                                                          * other.a23, a13 * other.a31 + a23 * other.a32 + a33 * other.a33));
        return result;
    }
    //求变换后点的坐标
    void PerspectiveTransform::transformPoints(vector<float> &points)
    {
        int max = (int)points.size();
        for (int i = 0; i < max; i += 2)
        {
            float x = points[i];
            float y = points[i + 1];  
            float denominator = a13 * x + a23 * y + a33; //w
            points[i] = (a11 * x + a21 * y + a31) / denominator;  //x/w
            points[i + 1] = (a12 * x + a22 * y + a32) / denominator; //y/w
        }  
    }
    
    
    
