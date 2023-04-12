#ifndef RGHS_H
#define RGHS_H

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class rghs_params{
    // this class calculates parameters for histogram streching for a channel for image.
    // imin,imax are min,max intensities of input image, omin,omax are min,max intensities for output image.

    public:
    int hist[256] = {0};
    int mode = 0;
    int mode_index = 0;
    double imin,imax,omin,omax;
    void hist_mode_index(Mat c);
    void find_min_max(Mat c,double tl,double kappa,double per);
    void kappa_tl(int i,double d,double* kappa, double* tl);
};
Mat rghs(Mat img, double per=0.1, double d=3);

#endif