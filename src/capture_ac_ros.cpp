#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>


using namespace cv;
using namespace std;

void configCapture(VideoCapture &capture, int device){
    if(capture.isOpened())
    {
        cout << "Capture is opened" << endl;
        capture.set(CAP_PROP_CONVERT_RGB, 0); 
        capture.set(CAP_PROP_BUFFERSIZE, 4);  
        capture.set(CAP_PROP_FOURCC, V4L2_PIX_FMT_Y10);
        capture.grab();
        usleep(1000 * 1000); 
        if (device == 0)
        {
            system("v4l2-ctl -d 0 -c frame_timeout=10000");
            system("v4l2-ctl -d 0 -c trigger_mode=1");
        }
        else if(device == 1)
        {
            system("v4l2-ctl -d 1 -c frame_timeout=10000");
            system("v4l2-ctl -d 1 -c trigger_mode=1");
        } 
        usleep(1000 * 1000); 
        capture.grab();

    }
    else
    {
        cout << "No capture" << endl;
    }
}

class synchImages 
{
public:
    void validCam0() {ret0 = 1;}
    void validCam1() {ret1 = 1;}
    int getRet0() {return ret0;}
    int getRet1() {return ret1;}
    bool synchronized() {
        return (ret0==1&&ret1==1) ? true:false;
    }
    Mat image0, image1;
private:
    int ret0 = 0;
    int ret1 = 0;
};

class Arducam
{
public:
    Arducam(int dev, VideoCapture* capture, synchImages* sync):device_(dev),capture_(capture),sync_(sync){}
    void operator()()
    {
        readImages_once();
    }
    void nameWindows()
    {
        windows_="Arducam_Camera_"+to_string(device_);
    }
    void readImages_once() {
        nameWindows();
        int ret_=capture_->read(image_);   
        if (device_==0) 
        {
            if (ret_) {
            //std::cout<<"cam0 received:" << ret_ << std::endl;
            image_.convertTo(image_, CV_8UC1, 0.25);
            resize(image_,image_,Size(),0.5,0.5);
            sync_->image0=image_;
            sync_->validCam0();
            }
        }
        else if (device_==1) 
        {
            if (ret_) {
            //std::cout<<"cam1 received:" << ret_ << std::endl;
            image_.convertTo(image_, CV_8UC1, 0.25);
            resize(image_,image_,Size(),0.5,0.5);
            sync_->image1=image_;
            sync_->validCam1();
            }
        }  
    }
private:
    int device_;
    VideoCapture* capture_;
    Mat image_;
    string windows_;
    synchImages* sync_;
};


int main(int argc, char** argv)
{
    ros::init(argc,argv,"arducam_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(5);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub0 = it.advertise("arducam/image0",1);
    image_transport::Publisher pub1 = it.advertise("arducam/image1",1);

    sensor_msgs::ImagePtr msg0;
    sensor_msgs::ImagePtr msg1;
    VideoCapture capture0(0, CAP_V4L2); 
    VideoCapture capture1(1, CAP_V4L2); 
    configCapture(capture0,0);
    configCapture(capture1,1);

    if(capture0.isOpened())
    {
        synchImages sync;
        Arducam ac0(0,&capture0, &sync);
        Arducam ac1(1,&capture1, &sync);


        while (nh.ok())
        { 
            std::thread t0 = std::thread(ac0);
            std::thread t1 = std::thread(ac1);
            t0.join();
            t1.join();
            if (sync.synchronized()){
		auto timestamp = ros::Time::now();
                msg0 = cv_bridge::CvImage(std_msgs::Header(), "mono8",sync.image0).toImageMsg();
		msg0->header.stamp = timestamp;
                pub0.publish(msg0);
                msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8",sync.image1).toImageMsg();
		msg1->header.stamp = timestamp;
                pub1.publish(msg1);
            }
        }
    }
    else
    {
        cout << "No capture" << endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
}


