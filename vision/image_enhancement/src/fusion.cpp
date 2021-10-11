#include<iostream>
#include<opencv2/opencv.hpp>
#include"../include/image_enhancement/fusion.h"

using namespace std;
using namespace cv;

Mat binomial_kernel(){
    float data[] = {1.0,4.0,6.0,4.0,1.0};
    Mat mask = Mat(1,5,CV_32F,data);
    mask = mask/16;
    mask = (mask.t())*mask;
    return mask;
}

Mat weights::extract_lchannel(Mat img){
    // assuming img is 8uchar BGR image.
    Mat im;
    cvtColor(img,im,COLOR_BGR2Lab);
    vector<Mat> lab;
    split(im,lab);
    Mat l;
    (lab[0]).convertTo(l,CV_32FC1);
    return l;
}

void clip_double(Mat& img,double min,double max){
    // assuming img is 32float C1.
    for(int i=0;i<img.rows;i++){
        for(int j=0;j<img.cols;j++){
            if(img.at<float>(i,j)<min)
            img.at<float>(i,j) = min;
            else if(img.at<float>(i,j)>max)
            img.at<float>(i,j) = max;
        }
    }
}

vector<Mat> pyramids::gauss_pyramid(Mat img,int level=6,bool apply_mask=false){
    // assuiming img is single channel.
    Mat im;
    img.convertTo(im,CV_32FC1);
    Mat mask = binomial_kernel();
    if(apply_mask){
        Mat tmp;
        filter2D(im,tmp,-1,mask,Point(-1,-1),0.0,BORDER_DEFAULT);
        tmp.copyTo(im);
    }
    vector<Mat> pyr;
    pyr.push_back(im);
    for(int i=1;i<level;i++){
        Mat tmp,tmp1;
        pyrDown(pyr[i-1],tmp,Size(),BORDER_DEFAULT);
        tmp.convertTo(tmp1,CV_32FC1);
        pyr.push_back(tmp1);
    }
    return pyr;
}

vector<Mat> pyramids::laplace_pyramid_c1(Mat c,int level=6){
    // assuiming c is single channel.
    vector<Mat> gauss = gauss_pyramid(c,level);
    vector<Mat> pyr;
    int h,w;
    for(int i=0;i<level-1;i++){
        Mat tmp1,tmp2;
        pyrUp(gauss[i+1],tmp1,Size(),BORDER_DEFAULT);
        resize(tmp1,tmp2,(gauss[i]).size());
        tmp2 = gauss[i]-tmp2;
        pyr.push_back(tmp2);
    }
    pyr.push_back(gauss[level-1]);
    return pyr;
}

vector<vector<Mat>> pyramids::laplace_pyramid_c3(Mat img, int level=6){
    vector<Mat> channels;
    split(img,channels);
    vector<vector<Mat>> pyr3;
    vector<Mat> pyr1;
    for(int i=0;i<3;i++){
        pyr1 = laplace_pyramid_c1(channels[i],level);
        pyr3.push_back(pyr1);
    }
    return pyr3;
}

vector<Mat> pyramids::multiply(vector<Mat> pyr1, vector<Mat> pyr2){
    int level = pyr1.size();
    vector<Mat> pyr;
    Mat tmp,tmp1,tmp2;
    for(int i=0;i<level;i++){
        (pyr1[i]).convertTo(tmp1,CV_32FC1);
        (pyr2[i]).convertTo(tmp2,CV_32FC1);
        // cout<<i<<"/"<<level<<endl;
        tmp = tmp1.mul(tmp2);
        pyr.push_back(tmp);
    }
    return pyr;
}

vector<Mat> pyramids::add(vector<Mat> pyr1, vector<Mat> pyr2){
    int level = pyr1.size();
    vector<Mat> pyr;
    Mat tmp;
    for(int i=0;i<level;i++){
        tmp = pyr1[i] + pyr2[i];
        pyr.push_back(tmp);
    }
    return pyr;
}

Mat pyramids::reconstruct_image_c1(vector<Mat> pyr){
    int level = pyr.size();
    for(int i=level-1;i>0;i--){
        Mat tmp1,tmp2;
        pyrUp(pyr[i],tmp1,Size(),BORDER_DEFAULT);
        resize(tmp1,tmp2,(pyr[i-1]).size());
        pyr[i-1] = pyr[i-1] + tmp2;
    }
    return pyr[0];
}

Mat weights::local_contrast_weights(Mat img){
    // assuming img is 8uchar BGR image.
    Mat l = extract_lchannel(img);
    Mat mask = binomial_kernel();
    Mat tmp,iwhc;
    filter2D(l,tmp,-1,mask,Point(-1,-1),0.0,BORDER_DEFAULT);
    tmp.convertTo(iwhc,CV_32FC1);
    clip_double(iwhc,0,M_PI/2.75);
    Mat w = cv::abs(l-iwhc);
    return w;
}

Mat weights::exposedness_weights(Mat img,double sigma=0.25){
    // assuming img is 8uchar BGR image.
    Mat l = extract_lchannel(img);
    Mat w;
    Mat _half = 0.5*(Mat::ones(l.rows,l.cols,CV_32FC1));
    Mat tmp;
    pow(l-_half,2.0,tmp);
    cv::exp(-1*(tmp/(2*sigma*sigma)),w);
    return w;
}

Mat weights::saliency_weights(Mat img){
    // assuming img is 8uchar BGR image.
    Mat lab;
    cvtColor(img,lab,COLOR_BGR2Lab);
    Mat tmp1;
    lab.convertTo(tmp1,CV_32FC3);
    Scalar mean;
    mean = cv::mean(tmp1);
    vector<Mat> mean_im;
    for(int i=0;i<3;i++){
        mean_im.push_back((Mat::ones(tmp1.rows,tmp1.cols,CV_32FC1))*mean[i]);
    }
    Mat mean_img;
    merge(mean_im,mean_img);
    Mat tmp2;
    cv::pow(tmp1-mean_img,2,tmp2);
    Mat w;
    transform(tmp2,w,Matx13f(1,1,1));
    Mat w2;
    sqrt(w,w2);
    w2 = w2/3;
    return w2;
}

Mat weights::laplacian_contrast(Mat img){
    // assuming img is 8uchar BGR image.
    Mat l;
    l = extract_lchannel(img);
    Mat tmp;
    Laplacian(l,tmp,-1);
    Mat tmp2,w;
    tmp2 = cv::abs(tmp);
    cv::normalize(tmp2,w);
    return w;
}
Mat weights::weight(Mat inp,double sigma = 0.25){
    // assuming inp is 8uchar BGR image.
    Mat lc1 = local_contrast_weights(inp);
    Mat ew1 = exposedness_weights(inp,sigma);
    Mat sw = saliency_weights(inp);
    Mat lapc = laplacian_contrast(inp);
    Mat w;
    w = lc1+ew1+sw+lapc;
    return w;
}
vector<Mat> weights::normalized_weights(Mat inp1,Mat inp2,double sigma){
    // assuming inp1, inp2 are 8uchar BGR image.
    Mat w1 = weight(inp1,sigma);
    Mat w2 = weight(inp2,sigma);
    Mat sum = w1+w2;
    Mat w1n,w2n;
    divide(w1,sum,w1n,1,-1);
    divide(w2,sum,w2n,1,-1);
    vector<Mat> out;
    out.push_back(w1n);
    out.push_back(w2n);
    return out;
}

Mat laplace_blending(Mat inp1,Mat inp2,double sigma,int level){
    vector<Mat> wgh;
    weights w;
    pyramids p;
    wgh = w.normalized_weights(inp1,inp2,sigma);
    vector<Mat> gauss1 = p.gauss_pyramid(wgh[0],level,true);
    vector<Mat> gauss2 = p.gauss_pyramid(wgh[1],level,true);
    vector<vector<Mat>> lap1 = p.laplace_pyramid_c3(inp1,level);
    vector<vector<Mat>> lap2 = p.laplace_pyramid_c3(inp2,level);
    vector<vector<Mat>> tmp1,tmp2;
    vector<Mat> tmp;
    for(int i=0;i<3;i++){
        tmp = p.multiply(gauss1,lap1[i]);
        tmp1.push_back(tmp);
        tmp = p.multiply(gauss2,lap2[i]);
        tmp2.push_back(tmp);
    }
    vector<Mat> channels;
    Mat channel;
    for(int i=0;i<3;i++){
        tmp = p.add(tmp1[i],tmp2[i]);
        channel = p.reconstruct_image_c1(tmp);
        channels.push_back(channel);
    }
    Mat out;
    merge(channels,out);
    Mat result;
    out.convertTo(result,CV_8UC3);
    // white_balance(result,0,out);
    return result;
    // return out;
}
