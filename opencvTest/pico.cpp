#include "pico.h"
#include <xmmintrin.h>


void MyPico::help()
{
	//target
	printf("Detect faces in images or video.\r\n");
	printf("\r\n");

	// infos about the params
	printf(" --input=PATH--sets the path to the input image or video.\r\n");
	printf(" --minsize=SIZE--sets the minimum size (in pixels) of an object (default is 32).\r\n");
	printf(" --maxsize=SIZE--sets the maximum size (in pixels) of an object (default is 256).\r\n");
	printf(" --angle=ANGLE--cascade rotation angle: 0.0(default) is 0 radians and 1.0 is 2*pi radians.\r\n");
	printf(" --qthreshold=THRESH--detection quality threshold (>=0.0): all detections with estimated quality.\r\n");
	printf(" ------below this threshold will be discarded(default is 5.0).\r\n");
	printf(" --scalefactor=SCALE--how much to rescale the window during the multiscale detection process (default is 1.1).\r\n");
	printf(" --stridefactor=STRIDE--how much to move the window between neighboring detections (default is 0.1, i.e., 10%).\r\n");
	printf(" --usepyr--turns on the coarse image pyramid support.\r\n");
	printf(" --noclustering--turns off detection clustering.\r\n");
	printf(" --verbose--print details of the detection process.\r\n");

	//the output status
	printf("Exit status: \r\n");
	printf(" 0 if OK,\r\n");
	printf(" 1 if trouble (e.g., invalid path to input image or video)..\r\n");

}


void MyPico::processImage(cv::Mat &frame, ALLTracks track, int drawFlag, bool usepyr, bool NO_Clistering, bool verbose, void *cascade, float angle, float qthreshold, float scalefactor, float stridefactor, int minsize, int maxsize)
{
	//int i, j;
	float t;

	uint8_t* pixels;
	int nrows, ncols, ldim;

#define MAXNDETECTIONS 50
	int ndetections = 0;
	//float qs[MAXNDETECTIONS], rs[MAXNDETECTIONS], cs[MAXNDETECTIONS], ss[MAXNDETECTIONS];
	float *qs, *rs, *cs, *ss;
	qs = (float *)malloc(MAXNDETECTIONS * sizeof(float));
	rs = (float *)malloc(MAXNDETECTIONS * sizeof(float));
	cs = (float *)malloc(MAXNDETECTIONS * sizeof(float));
	ss = (float *)malloc(MAXNDETECTIONS * sizeof(float));
	memset(qs, 0, MAXNDETECTIONS * sizeof(float));
	memset(rs, 0, MAXNDETECTIONS * sizeof(float));
	memset(cs, 0, MAXNDETECTIONS * sizeof(float));
	memset(ss, 0, MAXNDETECTIONS * sizeof(float));

	static cv::Mat gray;
	static cv::Mat pyr[5];
	

	gray = cv::Mat(frame.size(), frame.depth(), 1);
	pyr[0] = gray;
	pyr[1] = cv::Mat(cv::Size(frame.cols / 2, frame.rows / 2), frame.depth(), 1);
	pyr[2] = cv::Mat(cv::Size(frame.cols / 4, frame.rows / 4), frame.depth(), 1);
	pyr[3] = cv::Mat(cv::Size(frame.cols / 8, frame.rows / 8), frame.depth(), 1);
	pyr[4] = cv::Mat(cv::Size(frame.cols / 16, frame.rows / 16), frame.depth(), 1);

	// get grayscale image
	if (frame.channels() == 3)
		cvtColor(frame, gray, CV_RGB2GRAY);
	else
		gray = frame;

	// perform detection with the pico library
	t = get_ticks();

	if (usepyr)
	{
		int nd;

		//
		pyr[0] = gray;

		pixels = (uint8_t*)pyr[0].data;
		nrows = pyr[0].rows;
		ncols = pyr[0].cols;
		ldim = pyr[0].step;

		ndetections = find_objects(rs, cs, ss, qs, MAXNDETECTIONS, cascade, angle, pixels, nrows, ncols, ldim, scalefactor, stridefactor, MAX(16, minsize), MIN(128, maxsize));//16, 128

		for (int i = 1; i<5; ++i)
		{
			resize(pyr[i - 1], pyr[i], cv::Size(pyr[i-1].cols / 2, pyr[i-1].rows / 2));

			pixels = (uint8_t*)pyr[i].data;
			nrows = pyr[i].rows;
			ncols = pyr[i].cols;
			ldim = pyr[i].step;

			nd = find_objects(&rs[ndetections], &cs[ndetections], &ss[ndetections], &qs[ndetections], MAXNDETECTIONS - ndetections, cascade, angle, pixels, nrows, ncols, ldim, scalefactor, stridefactor, MAX(64, minsize >> i), MIN(128, maxsize >> i));//64, 128

			for (int j = ndetections; j<ndetections + nd; ++j)
			{
				rs[j] = (1 << i)*rs[j];
				cs[j] = (1 << i)*cs[j];
				ss[j] = (1 << i)*ss[j];
			}

			ndetections = ndetections + nd;
		}
	}
	else
	{
		//
		pixels = (uint8_t*)gray.data;
		nrows = gray.rows;
		ncols = gray.cols;
		ldim = gray.step;

		//
		ndetections = find_objects(rs, cs, ss, qs, MAXNDETECTIONS, cascade, angle, pixels, nrows, ncols, ldim, scalefactor, stridefactor, minsize, MIN(nrows, ncols));
	}

	if (! NO_Clistering)
		ndetections = cluster_detections(rs, cs, ss, qs, ndetections);

	t = get_ticks() - t;

	// if the flag is set, draw each detection
	vector<cv::Mat> face_image;
	cv::Mat img;
	//vector<cv::Rect> rects;
	if (drawFlag)
	{
		//cv::Rect rect_curr;
		
		//bool track_res;
		track.trackInit();
		cout << ndetections << endl;
		for (int i = 0; i < ndetections; ++i)
		{
			if (qs[i] >= qthreshold) // check the confidence threshold
			{
#if 1
				cv::Rect rect_curr = cv::Rect(cs[i] - ss[i] / 2, rs[i] - ss[i] / 2, ss[i], ss[i]);
				//cout << "rect_curr: " << rect_curr.x << "," << rect_curr.y << "," << rect_curr.width << "," << rect_curr.height << endl;
				cv::rectangle(frame, rect_curr, Scalar(0, 0, 255), 3);
				//rects.push_back(rect_curr);
				//circle(frame, cv::Point(cs[i], rs[i]), ss[i] / 2, Scalar(255, 0, 0), 2, 3, 0); // we draw circles here since height-to-width ratio of the detected face regions is 1.0f
				//rect = cv::Rect(cs[i] - ss[i]/2, rs[i] - ss[i]/2, ss[i], ss[i]);
				//cv::Point center_curr = cv::Point(cs[i], rs[i]);
				//cv::Point center_pre = cv::Point(rect_pre.x + rect_pre.width / 2, rect_pre.y + rect_pre.height / 2);
				track.processImage(frame, rect_curr);
				//float area_overlap = get_overlap(cs[i], rs[i], ss[i], track.rect_new.x, track.rect_new.y, track.rect_new.width);
#if 1				
				bool overlap_flag = isOverlap(rect_curr, track.rect_new);
				float area_join, area_union;
				float ratio = computeRectJoinUnion(rect_curr, track.rect_new, area_join, area_union);
				//std::cout << "the ratio of overlap is: " << ratio << std::endl;
				if ( ratio > 0.8)
				{
					// do not take face
					
				}
					
				else
				{
					//take face
					img = cv::Mat(frame, rect_curr);
					imshow("take face", img);
					face_image.push_back(img);
					img.release();
					//imwrite("D:\\zhlWorkDocs\\testWorksDemo\\out_face_images\\", face_image);
				}
#endif				
				//next
				//rect_pre = rect_curr;
				//rect_res.push_back(rect);
#endif
		
			}
				
		}		
	}

	/*track.trackInit();
	for (int i = 0; i < rects.size(); i++){
		track.processImage(frame, rects[i]);
	}*/

	char fileName[500];
	for (int num = 0; num < face_image.size(); num++)
	{
		sprintf_s(fileName, "D:\\zhlWorkDocs\\testWorksDemo\\out_face_images\\face%04d.jpg", num);
		imwrite(fileName, face_image[num]);

	}
	
	face_image.clear();
	gray.release();
	free(qs);
	free(rs);
	free(cs);
	free(ss);
	//delete[] pyr;
	for (int i = 0; i < 5; i++)
		pyr[i].release();
}


void MyPico::process_webcam_frames(char *video_path, ALLTracks track, int drawFlag, bool usepyr, bool NO_Clistering, bool verbose, void *cascade, float angle, float qthreshold, float scalefactor, float stridefactor, int minsize, int maxsize)
{
	VideoCapture capture(video_path);
	cv::Mat frame;
	cv::Mat framecopy;

	int stop;
	if (!capture.isOpened())
	{
		printf("* cannot initialize video capture ...\n");
	}

	
	// the main loop
	framecopy = 0;
	stop = 0;
	while (!stop)
	{
		// wait 5 miliseconds
		int key = waitKey(1);

		// get the frame from webcam
		capture.read(frame);
		if (frame.empty())
		{
			stop = 1;
			frame = 0;
		}

		// we terminate the loop if the user has pressed 'q'
		if (frame.empty() || key == 'q')
			stop = 1;
		else
		{
			// we mustn't tamper with internal OpenCV buffers
			if (framecopy.empty())
				framecopy = cv::Mat(frame.size(), frame.depth(), frame.channels());
			framecopy = frame.clone();

			// face detect and track
			processImage(framecopy, track, drawFlag, usepyr, NO_Clistering, verbose, cascade, angle, qthreshold, scalefactor, stridefactor, minsize, maxsize);
			// ...
			imshow("drawing", framecopy);
		}
	}

	

	

	// cleanup
	framecopy.release();
	capture.release();
	rects_detect.clear();
}

int MyPico::run_cascade(void* cascade, float* o, int r, int c, int s, void* vppixels, int nrows, int ncols, int ldim)
{
	//
	int i, j, idx;

	uint8_t* pixels;

	int tdepth, ntrees, offset;

	int8_t* ptree;
	int8_t* tcodes;
	float* lut;
	float thr;

	//
	pixels = (uint8_t*)vppixels;

	//
	tdepth = ((int*)cascade)[2];
	ntrees = ((int*)cascade)[3];

	//
	r = r * 256;
	c = c * 256;

	if ((r + 128 * s) / 256 >= nrows || (r - 128 * s) / 256<0 || (c + 128 * s) / 256 >= ncols || (c - 128 * s) / 256<0)
		return -1;

	//
	offset = ((1 << tdepth) - 1)*sizeof(int32_t)+(1 << tdepth)*sizeof(float)+1 * sizeof(float);
	ptree = (int8_t*)cascade + 2 * sizeof(float)+2 * sizeof(int);

	*o = 0.0f;

	for (i = 0; i<ntrees; ++i)
	{
		//
		tcodes = ptree - 4;
		lut = (float*)(ptree + ((1 << tdepth) - 1)*sizeof(int32_t));
		thr = *(float*)(ptree + ((1 << tdepth) - 1)*sizeof(int32_t)+(1 << tdepth)*sizeof(float));

		//
		idx = 1;

		for (j = 0; j<tdepth; ++j)
			idx = 2 * idx + (pixels[(r + tcodes[4 * idx + 0] * s) / 256 * ldim + (c + tcodes[4 * idx + 1] * s) / 256] <= pixels[(r + tcodes[4 * idx + 2] * s) / 256 * ldim + (c + tcodes[4 * idx + 3] * s) / 256]);

		*o = *o + lut[idx - (1 << tdepth)];

		//
		if (*o <= thr)
			return -1;
		else
			ptree = ptree + offset;
	}

	//
	*o = *o - thr;

	return +1;
}



int MyPico::run_rotated_cascade(void* cascade, float* o, int r, int c, int s, float a, void* vppixels, int nrows, int ncols, int ldim)
{
	//
	int i, j, idx;

	uint8_t* pixels;

	int tdepth, ntrees, offset;

	int8_t* ptree;
	int8_t* tcodes;
	float* lut;
	float thr;

	static int qcostable[32 + 1] = { 256, 251, 236, 212, 181, 142, 97, 49, 0, -49, -97, -142, -181, -212, -236, -251, -256, -251, -236, -212, -181, -142, -97, -49, 0, 49, 97, 142, 181, 212, 236, 251, 256 };
	static int qsintable[32 + 1] = { 0, 49, 97, 142, 181, 212, 236, 251, 256, 251, 236, 212, 181, 142, 97, 49, 0, -49, -97, -142, -181, -212, -236, -251, -256, -251, -236, -212, -181, -142, -97, -49, 0 };

	//
	pixels = (uint8_t*)vppixels;

	//
	tdepth = ((int*)cascade)[2];
	ntrees = ((int*)cascade)[3];

	//
	r = r * 65536;
	c = c * 65536;

	if ((r + 46341 * s) / 65536 >= nrows || (r - 46341 * s) / 65536<0 || (c + 46341 * s) / 65536 >= ncols || (c - 46341 * s) / 65536<0)
		return -1;

	//
	offset = ((1 << tdepth) - 1)*sizeof(int32_t)+(1 << tdepth)*sizeof(float)+1 * sizeof(float);
	ptree = (int8_t*)cascade + 2 * sizeof(float)+2 * sizeof(int);

	*o = 0.0f;

	int qsin = s*qsintable[(int)(32 * a)]; //s*(int)(256.0f*sinf(2*M_PI*a));
	int qcos = s*qcostable[(int)(32 * a)]; //s*(int)(256.0f*cosf(2*M_PI*a));

	for (i = 0; i<ntrees; ++i)
	{
		//
		tcodes = ptree - 4;
		lut = (float*)(ptree + ((1 << tdepth) - 1)*sizeof(int32_t));
		thr = *(float*)(ptree + ((1 << tdepth) - 1)*sizeof(int32_t)+(1 << tdepth)*sizeof(float));

		//
		idx = 1;

		for (j = 0; j<tdepth; ++j)
		{
			int r1, c1, r2, c2;

			//
			r1 = (r + qcos*tcodes[4 * idx + 0] - qsin*tcodes[4 * idx + 1]) / 65536;
			c1 = (c + qsin*tcodes[4 * idx + 0] + qcos*tcodes[4 * idx + 1]) / 65536;

			r2 = (r + qcos*tcodes[4 * idx + 2] - qsin*tcodes[4 * idx + 3]) / 65536;
			c2 = (c + qsin*tcodes[4 * idx + 2] + qcos*tcodes[4 * idx + 3]) / 65536;

			//
			idx = 2 * idx + (pixels[r1*ldim + c1] <= pixels[r2*ldim + c2]);
		}

		*o = *o + lut[idx - (1 << tdepth)];

		//
		if (*o <= thr)
			return -1;
		else
			ptree = ptree + offset;
	}

	//
	*o = *o - thr;

	return +1;
}

int MyPico::find_objects
(
float rs[], float cs[], float ss[], float qs[], int maxndetections,
void* cascade, float angle, // * `angle` is a number between 0 and 1 that determines the counterclockwise in-plane rotation of the cascade: 0.0f corresponds to 0 radians and 1.0f corresponds to 2*pi radians
void* pixels, int nrows, int ncols, int ldim,
float scalefactor, float stridefactor, float minsize, float maxsize
)
{
	float s;
	int ndetections;

	//
	ndetections = 0;
	s = minsize;

	while (s <= maxsize)
	{
		float r, c, dr, dc;

		//
		dr = dc = MAX(stridefactor*s, 1.0f);

		//
		for (r = s / 2 + 1; r <= nrows - s / 2 - 1; r += dr)
		for (c = s / 2 + 1; c <= ncols - s / 2 - 1; c += dc)
		{
			float q;
			int t;

			if (0.0f == angle)
				t = run_cascade(cascade, &q, r, c, s, pixels, nrows, ncols, ldim);
			else
				t = run_rotated_cascade(cascade, &q, r, c, s, angle, pixels, nrows, ncols, ldim);

			if (1 == t)
			{
				if (ndetections < maxndetections)
				{
					qs[ndetections] = q;
					rs[ndetections] = r;
					cs[ndetections] = c;
					ss[ndetections] = s;

					//
					++ndetections;
				}
			}
		}

		//
		s = scalefactor*s;
	}

	//
	return ndetections;
}

/*

*/

float MyPico::get_overlap(float r1, float c1, float s1, float r2, float c2, float s2)
{
	float overr, overc;

	//
	overr = MAX(0, MIN(r1 + s1 / 2, r2 + s2 / 2) - MAX(r1 - s1 / 2, r2 - s2 / 2));
	overc = MAX(0, MIN(c1 + s1 / 2, c2 + s2 / 2) - MAX(c1 - s1 / 2, c2 - s2 / 2));

	//
	return overr*overc / (s1*s1 + s2*s2 - overr*overc);
}

void MyPico::ccdfs(int a[], int i, float rs[], float cs[], float ss[], int n)
{
	int j;

	//
	for (j = 0; j<n; ++j)
	if (a[j] == 0 && get_overlap(rs[i], cs[i], ss[i], rs[j], cs[j], ss[j])>0.3f)
	{
		//
		a[j] = a[i];

		//
		ccdfs(a, j, rs, cs, ss, n);
	}
}

int MyPico::find_connected_components(int a[], float rs[], float cs[], float ss[], int n)
{
	int i, ncc, cc;

	//
	if (!n)
		return 0;

	//
	for (i = 0; i<n; ++i)
		a[i] = 0;

	//
	ncc = 0;
	cc = 1;

	for (i = 0; i<n; ++i)
	if (a[i] == 0)
	{
		//
		a[i] = cc;

		//
		ccdfs(a, i, rs, cs, ss, n);

		//
		++ncc;
		++cc;
	}

	//
	return ncc;
}

int MyPico::cluster_detections(float rs[], float cs[], float ss[], float qs[], int n)
{
	int idx, ncc, cc;
	int a[4096];

	//
	ncc = find_connected_components(a, rs, cs, ss, n);

	if (!ncc)
		return 0;

	//
	idx = 0;

	for (cc = 1; cc <= ncc; ++cc)
	{
		int i, k;

		float sumqs = 0.0f, sumrs = 0.0f, sumcs = 0.0f, sumss = 0.0f;

		//
		k = 0;

		for (i = 0; i<n; ++i)
		if (a[i] == cc)
		{
			sumqs += qs[i];
			sumrs += rs[i];
			sumcs += cs[i];
			sumss += ss[i];

			++k;
		}

		//
		qs[idx] = sumqs; // accumulated confidence measure

		//
		rs[idx] = sumrs / k;
		cs[idx] = sumcs / k;
		ss[idx] = sumss / k;

		//
		++idx;
	}

	//
	return idx;
}

float MyPico::get_ticks()
{
	static double freq = -1.0;
	LARGE_INTEGER lint;

	if (freq < 0.0)
	{
		if (!QueryPerformanceFrequency(&lint))
			return -1.0f;

		freq = lint.QuadPart;
	}

	if (!QueryPerformanceCounter(&lint))
		return -1.0f;

	return (float)(lint.QuadPart / freq);
}



//compute the length of two points
double MyPico::line_size(cv::Point &p1, cv::Point &p2){
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

//compute the area of triangle maked up by 3 points
float MyPico::area(cv::Point &pt, cv::Point &p1, cv::Point &p2){
	Point2f v;
	double d = line_size(p1, p2);
	v.x = ((float)p1.x - (float)p2.x) / d;
	v.y = ((float)p1.y - (float)p2.y) / d;
	float dd = abs(v.y*pt.x - v.x*pt.y + v.x*p1.y - v.y*p1.x);
	return 0.5*dd*d;
}

//decide if a point is in the rectangle or not
bool MyPico::isInRectangle(cv::Point& pt, vector<cv::Point>& pr){
	float S01, S12, S23, S30;
	S01 = area(pt, pr[0], pr[1]);
	S12 = area(pt, pr[1], pr[2]);
	S23 = area(pt, pr[2], pr[3]);
	S30 = area(pt, pr[3], pr[0]);
	float ss = S01 + S12 + S23 + S30;
	float s012, s123;
	// float s102,s302;
	s012 = area(pr[0], pr[1], pr[2]);
	s123 = area(pr[3], pr[1], pr[2]);
	//s102 = area(pr[1], pr[0], pr[2]);
	// s302 = area(pr[3], pr[0], pr[2]);
	if (abs(ss - s012 - s123) < 100)
		//if ((s102 + s302) == ss)
		//float sr = line_size(pr[0], pr[1]) * line_size(pr[1], pr[2]);
		//if (ss == sr)
	{
		return true;
	}
	else
		return false;
}
//get the vector from two point
cv::Point MyPico::vector_g(cv::Point &a, cv::Point &b){
	cv::Point c;
	c.x = b.x - a.x;
	c.y = b.y - a.y;
	return c;
}

//compute the cross product
double MyPico::cross_product(cv::Point &a, cv::Point &b){
	double cp, d1, d2, m;
	double cosr;
	m = a.x*b.x + a.y*b.y;
	d1 = sqrt(a.x * a.x + a.y * a.y);
	d2 = sqrt(b.x * b.x + b.y * b.y);
	cosr = m / d1 / d2;
	cp = d1 * d2 * sin(acos(cosr));
	return cp;
}

//compute the interesct area maked up by vertices
double MyPico::area_vertice_intersection(vector<cv::Point> &points){
	vector<int> hull;
	if (points.size() < 3)
		return 0;
	convexHull(points, hull, true);
	cv::Point poi = points[hull[hull.size() - 1]];
	vector<cv::Point> vec;
	for (int i = 0; i < hull.size(); i++)
	{
		cv::Point p_new = points[hull[i]];
		cv::Point v;
		v = vector_g(poi, p_new);
		if (v.x != 0 || v.y != 0)
		{
			vec.push_back(v);
		}
		poi = p_new;
	}
	double sum = 0.0;
	int i = 0;
	while (i <= (int)vec.size() - 2)
	{
		double css;
		css = cross_product(vec[i], vec[i + 1]);
		sum = sum + css;
		i = i + 2;
	}
	return sum / 2;
}

void MyPico::DecideOberlap(vector<cv::Point> &pr1, vector<cv::Point> &pr2, Size imgsize, int& k1, int & k2)
{
	vector<cv::Point> p1, p2;
	p1.push_back(pr1[0]);
	p1.push_back(pr1[2]);
	p1.push_back(pr1[3]);
	p1.push_back(pr1[1]);
	p2.push_back(pr2[0]);
	p2.push_back(pr2[2]);
	p2.push_back(pr2[3]);
	p2.push_back(pr2[1]);
	vector<cv::Point> points;
	for (int m = 0; m < 4; m++)
	{
		if (isInRectangle(p1[m], p2))
		{
			points.push_back(p1[m]);
		}
		if (isInRectangle(p2[m], p1))
		{
			points.push_back(p2[m]);
		}
		for (int n = 0; n < 4; n++)
		{
			Point pt;
			LINE l1, l2;
			l1 = makeline(p1[m % 4], p1[(m + 1) % 4]);
			l2 = makeline(p2[n % 4], p2[(n + 1) % 4]);
			if (lineintersect(l1, l2, pt))//互相交叉
			{
				if (online(l1, pt) && online(l2, pt))
				{
					points.push_back(pt);
				}
			}
			else
			{//互相包含
				if (online(l1, l2.start))
					points.push_back(l2.start);
				else if (online(l1, l2.end))
					points.push_back(l2.end);
				else if (online(l2, l1.start))
					points.push_back(l1.start);
				else if (online(l2, l1.end))
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
		k1 = 0; k2 = 0;
	}
	else
	{
		double area1, area2;
		area1 = area_vertice_intersection(pr1);
		area2 = area_vertice_intersection(pr2);
		float rate = 0.4;
		double b1, b2;
		b1 = area / area1;
		b2 = area / area2;
		if ((b1 >= rate || b2 >= rate) && b1 > b2)
		{
			k1 = 1; k2 = 0;
		}
		else if ((b1 >= rate || b2 >= rate) && b2 > b1)
		{
			k1 = 0; k2 = 1;
		}
		else
		{
			k1 = 0; k2 = 0;
		}
	}
}

//get the linear equation
LINE MyPico::makeline(cv::Point &p1, cv::Point &p2) {
	LINE tl;
	tl.start = p1;
	tl.end = p2;
	int sign = 1;
	tl.a = p2.y - p1.y;
	if (tl.a<0) {
		sign = -1;
		tl.a = sign*tl.a;
	}
	tl.b = sign*(p1.x - p2.x);
	tl.c = sign*(p1.y*p2.x - p1.x*p2.y);
	return tl;
}

//get the intersect point of two lines
bool MyPico::lineintersect(LINE l1, LINE l2, cv::Point &p)
{
	double EP = 0.03;
	double d = l1.a*l2.b - l2.a*l1.b;
	if (abs(d)<EP) return false;// 不相交
	p.x = (l2.c*l1.b - l1.c*l2.b) / d;
	p.y = (l2.a*l1.c - l1.a*l2.c) / d;
	return true;
}

bool MyPico::online(LINE l, cv::Point p)
{
	return ((((p.x - l.start.x)*(p.x - l.end.x) <= 0) && ((p.y - l.start.y)*(p.y - l.end.y) <= 0)));
}


bool MyPico::isOverlap(cv::Rect &rc1, cv::Rect &rc2)
{
	if (rc1.x + rc1.width  > rc2.x &&
		rc2.x + rc2.width  > rc1.x &&
		rc1.y + rc1.height > rc2.y &&
		rc2.y + rc2.height > rc1.y
		)
		return true;
	else
		return false;
}

float MyPico::computeRectJoinUnion(cv::Rect &rc1, cv::Rect &rc2, float& AJoin, float& AUnion)
{
	cv::Point p1, p2;                 //p1为相交位置的左上角坐标，p2为相交位置的右下角坐标
	p1.x = std::max(rc1.x, rc2.x);
	p1.y = std::max(rc1.y, rc2.y);

	p2.x = std::min(rc1.x + rc1.width, rc2.x + rc2.width);
	p2.y = std::min(rc1.y + rc1.height, rc2.y + rc2.height);

	AJoin = 0;
	if (p2.x > p1.x && p2.y > p1.y)            //判断是否相交
	{
		AJoin = (p2.x - p1.x)*(p2.y - p1.y);    //如果先交，求出相交面积
	}
	float A1 = rc1.width * rc1.height;
	float A2 = rc2.width * rc2.height;
	AUnion = (A1 + A2 - AJoin);                 //两者组合的面积

	if (AUnion > 0)
		return (AJoin / AUnion);                  //相交面积与组合面积的比例
	else
		return 0;
}