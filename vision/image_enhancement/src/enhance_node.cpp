#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_enhancement/color_correction.h>
#include<image_enhancement/fusion.h>
#include<image_enhancement/rghs.h>

using namespace std;
using namespace cv;

class enhance{
    private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    image_transport::Publisher pub_;
    image_transport::Subscriber sub_;
    double per_whitebalance,per_rghs,d,lamda,sf,sg,sigma;
    int level;

    public:
    
    enhance(): it(nh_){
        nh_.getParam("/color_correction/percentile",per_whitebalance);
        nh_.getParam("/color_correction/lamda",lamda);
        nh_.getParam("/biletral_filter/sigma_f",sf);
        nh_.getParam("/biletral_filter/sigma_g",sg);
        nh_.getParam("/fusion/pyr_level",level);
        nh_.getParam("/fusion/exposedness_sigma",sigma);
        nh_.getParam("/rghs/percentile",per_rghs);
        nh_.getParam("/rghs/distance",d);
        pub_ = it.advertise("enhanced",100);
        sub_ = it.subscribe("webcam",100,&enhance::enhance_rghs,this);
    }
    
    void enhance_fusion(const sensor_msgs::ImageConstPtr& img_msg){
    
        Mat org_img = cv_bridge::toCvShare(img_msg,"bgr8")->image;
        // enhancing the image....
        // Mat org_img;
        // resize(org_img1,org_img,Size(256,256));
        Mat inp1 = white_balance(org_img,per_whitebalance,lamda);
        Mat inp2_1 = biletral(inp1,5,sf,sg);
        Mat inp2 = clahe(inp2_1);
        Mat result = laplace_blending(inp1,inp2,sigma,level);
        
        // publishing the image to "enhanced" topic
        sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",result).toImageMsg();
        pub_.publish(result_msg);
    }

    void enhance_rghs(const sensor_msgs::ImageConstPtr& img_msg){
        Mat org_img1 = cv_bridge::toCvShare(img_msg,"bgr8")->image;
        // enhancing the image....
        Mat org_img;
        resize(org_img1,org_img,Size(256,256));
        Mat inp1 = white_balance(org_img,per_whitebalance,lamda);
        Mat tmp = biletral(inp1,5,sf,sg);
        Mat result = rghs(tmp,per_rghs,d);

        // publishing the image to "enhanced" topic...
        sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",result).toImageMsg();
        pub_.publish(result_msg);
    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"enhancer");
    enhance en;
    ros::spin();
}
