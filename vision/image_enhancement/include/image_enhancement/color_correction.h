#ifndef COLOR_CORRECTION
#define COLOR_CORRECTION

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class white_balancing{
    // This class consists of functions which are used for white balancing.
    private:
        void clip_double(Mat& img,double min,double max);
        void clip_int(Mat& img,int min,int max);
        void percentile(Mat& img,double per,int* min,int* max);

    public:
        void algo1(Mat& image,double per,Mat& im);
        void gray_world_algo(Mat& img,Mat& out,double lambda);
        void algo_2(Mat& img,Mat& im);
};

Mat white_balance(Mat& img,double per=5,double lambda=0.2);
Mat biletral(Mat& img, int n=5,double sf=0.1,double sg=0.1);
Mat clahe(Mat& img, double clip=4.0, int n=8);

#endif