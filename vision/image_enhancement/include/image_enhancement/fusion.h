#ifndef FUSION_H
#define FUSION_H

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class pyramids{
    public:
    vector<Mat> gauss_pyramid(Mat img,int level,bool apply_mask);
    vector<Mat> laplace_pyramid_c1(Mat c,int level);
    vector<vector<Mat>> laplace_pyramid_c3(Mat img, int level);
    vector<Mat> multiply(vector<Mat> pyr1, vector<Mat> pyr2);
    vector<Mat> add(vector<Mat> pyr1, vector<Mat> pyr2);
    Mat reconstruct_image_c1(vector<Mat> pyr);
};

class weights{
    private:
    Mat local_contrast_weights(Mat img);
    Mat exposedness_weights(Mat img,double sigma);
    Mat saliency_weights(Mat img);
    Mat laplacian_contrast(Mat img);
    Mat weight(Mat inp,double sigma);

    Mat extract_lchannel(Mat img);

    public:
    vector<Mat> normalized_weights(Mat inp1,Mat inp2,double sigma);

};
Mat laplace_blending(Mat inp1,Mat inp2,double sigma=0.25,int level=6);
Mat binomial_kernel();
#endif