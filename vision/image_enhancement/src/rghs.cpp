#include<iostream>
#include"../include/image_enhancement/rghs.h"
#include<opencv2/opencv.hpp>
#include<cmath>

using namespace std;
using namespace cv;

void rghs_params::hist_mode_index(Mat c){
    // c is 8UC1 type
    // calculates histogram, mode of histogram and index of mode in sorted image, which are required in caculating imin,imax,omin,omax.

    int i=0;
    int m=0;
    for(i=0;i<c.rows;i++){
        for(int j=0;j<c.cols;j++){
            int intensity = c.at<uchar>(i,j);
            hist[intensity]++;
            if (hist[intensity]>m){
                mode = intensity;
                m = hist[intensity];
            }
        }
    }
    if(mode==0)
    mode = 1;
    for(i=0;i<mode;i++){
        mode_index += hist[i];
    }
    mode_index+=1;
    return;
}
void rghs_params::kappa_tl(int i,double d,double* kappa, double* tl){
    // gives kappa and tl parameters for the channel, these parameters depends on colour channel and are derived from model of light propagation.

    *kappa = 0.9;
    if (i==0){
        *kappa = 1.1;
        *tl = pow(0.83,d);
    }
    else if(i==1)
    *tl = pow(0.95,d);
    else
    *tl = pow(0.97,d);
    return;
}
void rghs_params::find_min_max(Mat c,double tl,double kappa,double per){
    // calculates imin,imax,omin,omax.

    int ind,n=256;
    ind = mode_index*per/100;
    int su=0,i;
    for(i=0;i<n;i++){
        su+=hist[i];
        if(su>=ind){
            imin = i;
            break;
        }
    }
    for(su=0,i=255;i>=0;i--){
        su+=hist[i];
        if(su>=ind){
            imax = i;
            break;
        }
    }
    double sigma;
    sigma = sqrt((4-M_PI)/2.0)*mode;
    omin = floor(mode - 1.5*sigma);
    int ll,ul;
    ll = ceil(kappa*tl*imax/sigma-1.526);
    ul = floor(kappa*tl*255/sigma-1.526);
    if(ll>ul){
        omax = 255;
    }
    else{
        double mu = (ll+ul)/2.0;
        omax = floor((mode+mu*sigma)/(kappa*tl));
    }
    // cout<<imin<<" "<<imax<<" "<<omin<<" "<<omax<<endl;
}

Mat rghs(Mat img, double per, double d){
    // img is 8uchar c3 BGR image
    // this is a wrapper function, only this is called for applying rghs.

    Mat im;
    cvtColor(img,im,COLOR_BGR2RGB);
    vector<Mat> channels;
    vector<Mat> out_channels;
    split(im,channels);
    for(int i=0;i<3;i++){
        rghs_params params;
        double kappa,tl;
        params.hist_mode_index(channels[i]);
        params.kappa_tl(i,d,&kappa,&tl);
        params.find_min_max(channels[i],tl,kappa,per);
        Mat tmp;
        Mat imin_ = Mat::ones(img.rows,img.cols,CV_8UC1)*(params.imin);
        Mat omin_ = Mat::ones(img.rows,img.cols,CV_8UC1)*(params.omin);
        tmp = (channels[i].clone()-imin_)*(params.omax-params.omin)/(params.imax-params.imin) + omin_;
        out_channels.push_back(tmp);
    }
    Mat tmp1,tmp2,out;
    merge(out_channels,tmp1);
    tmp1.convertTo(tmp2,CV_8UC3);
    cvtColor(tmp2,out,COLOR_RGB2BGR);
    return out;
}
