#include<iostream>
#include<opencv2/opencv.hpp>
#include"../include/image_enhancement/color_correction.h"

using namespace std;
using namespace cv;

void white_balancing::clip_double(Mat& img,double min,double max){
    // assuming img is 32float C1.
    // This function clips the img between min and max values.
    
    for(int i=0;i<img.rows;i++){
        for(int j=0;j<img.cols;j++){
            if(img.at<float>(i,j)<min)
            img.at<float>(i,j) = min;
            else if(img.at<float>(i,j)>max)
            img.at<float>(i,j) = max;
        }
    }
}

void white_balancing::clip_int(Mat& img,int min,int max){
    // assuming img is 8Uchar C1 type.
    // clips the img between min and max
    
    for(int i=0;i<img.rows;i++){
        for(int j=0;j<img.cols;j++){
            if(img.at<uchar>(i,j)<min)
            img.at<uchar>(i,j) = min;
            else if(img.at<uchar>(i,j)>max)
            img.at<uchar>(i,j) = max;
        }
    }
}

void white_balancing::percentile(Mat& img,double per,int* min,int* max){
    // assuming img id 8uchar c1 type.
    // it find the pixel intensity at per and 100-per percentile in sorted image(flattening the image into 1-d array and sorting it).
    
    int hist[256]={0};
    int i;
    for(i=0;i<img.rows;i++){
        for(int j=0;j<img.cols;j++){
            hist[img.at<uchar>(i,j)]++;
        }
    }
    int sum=0;
    double index;
    index = per*(img.rows)*(img.cols)/100;
    for(i=0;i<256;i++){
        sum+=hist[i];
        if(sum>=index)
        break;
    }
    *min = i;
    for(sum=0,i=255;i>=0;i--){
        sum+=hist[i];
        if(sum>=index)
        break;
    }
    *max = i;
}

void white_balancing::algo1(Mat& image,double per,Mat& im){
    // assuming image is 8uc3 BGR image
    // Applies histogram streching to image for color correction.

    Mat ims = image.clone();
    Mat img;
    int chs = 3;
    Mat c = Mat::zeros(image.rows,image.cols,CV_8UC1);
    Mat tmp = Mat::zeros(image.rows,image.cols,CV_8UC1);
    // Mat channels[3] = {Mat::zeros(image.rows,image.cols,CV_8UC1),Mat::zeros(image.rows,image.cols,CV_8UC1),Mat::zeros(image.rows,image.cols,CV_8UC1)};
    vector<Mat> channels;
    vector<Mat> outputs;
    split(ims,channels);
    for(int i=0;i<chs;i++){
        int mi,ma;
        percentile(channels[i],per,&mi,&ma);
        clip_int(channels[i],(double)(mi),(double)(ma));
        channels[i].convertTo(c,CV_32FC1);
        c = (c-mi*(Mat::ones(c.rows,c.cols,CV_32FC1)))/(ma-mi);
        c = c*255.0;
        c.convertTo(tmp,CV_8UC1);
        outputs.push_back(tmp.clone());
    }
    merge(outputs,im);
}

void white_balancing::gray_world_algo(Mat& img,Mat& out,double lambda=0.2){
    // img is 8uc3 image
    // applies modified gray world algorithm for color correction, as described in research paper.
    
    Mat ims;
    img.convertTo(ims,CV_32FC3);
    ims = ims/255.0;
    Scalar mn;
    double ilum;
    Mat tmp,tmp2;
    vector<Mat> chs,out_chs;
    split(ims,chs);
    for(int i=0;i<3;i++){
        mn = mean(chs[i]);
        ilum = mn[0];
        ilum = 0.5+lambda/ilum;
        tmp2 = ilum*chs[i];
        tmp = tmp2.clone();
        clip_double(tmp,0.0,1.0);
        out_chs.push_back(tmp);
    }
    Mat tmp3;
    merge(out_chs,tmp3);
    tmp3 = tmp3*255;
    tmp3.convertTo(out,CV_8UC3);
}

void white_balancing::algo_2(Mat& img,Mat& out){
    // img is 8uc3 BGR image
    // applies illumination correction to red and blue channel. red channel is not overstreched using this algorithm. 
    
    vector<Mat> chs;
    Mat ims;
    img.convertTo(ims,CV_32FC3);
    ims = ims/255.0;
    split(ims,chs);
    Mat one;
    one = Mat::ones(chs[1].rows,chs[1].cols,CV_32FC1);
    Mat rrc,rbc;
    rrc = (chs[1]).mul(one-chs[2]);
    rbc = (chs[1]).mul(one-chs[0]);
    vector<Mat> tmp_chs;
    Mat tmp,tm;
    Scalar mn;
    double men;

    mn = mean(chs[1]-chs[0]);
    men = mn[0];
    tmp = chs[0] + rbc*(men);
    tmp_chs.push_back(tmp);

    tmp_chs.push_back(chs[1]);

    mn = mean(chs[1]-chs[2]);
    men = mn[0];
    tm = chs[2] + rrc*(men);
    tmp_chs.push_back(tm);

    Mat tmp2;
    merge(tmp_chs,tmp2);
    Mat tmp4;
    tmp4 = tmp2*255;
    tmp4.convertTo(out,CV_8UC3);
    // white_balance_algo1(tmp3,per,tmp4);
    // gray_world_algo(tmp4,im,lambda);
}

Mat white_balance(Mat& img,double per,double lambda){
    // img is 8UC3 BGR image.
    // a wrapper function for white balancing. Only this function is called directly for white balancing.
    // it returns color corrected, white balanced image.
    
    Mat tmp1,tmp2,out;
    white_balancing wb;
    wb.algo_2(img,tmp1);
    wb.algo1(tmp1,per,tmp2);
    wb.gray_world_algo(tmp2,out,lambda);
    return out;
}

Mat biletral(Mat& img, int n,double sf,double sg){
    // img is 8Uc3 BGR image.
    // biletral filter is applied on img, for noise reduction.

    Mat im = img.clone();
    Mat out;
    bilateralFilter(im,out,n,sf,sg);
    return out;
}
Mat clahe(Mat& img, double clip, int n){
    // img is 8UC3 BGR image.
    // returns image after applying clahe algorithm on img.

    Mat im,tmp,clahed;
    cvtColor(img,im,COLOR_BGR2HSV);
    vector<Mat> hsv;
    split(im,hsv);
    Ptr<CLAHE> cl = createCLAHE(clip,Size(n,n));
    cl->apply(hsv[2],tmp);
    tmp.copyTo(hsv[2]);
    merge(hsv,clahed);
    Mat out;
    cvtColor(clahed,out,COLOR_HSV2BGR);
    return out;
}