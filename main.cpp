#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <chrono>
#include <vector>

using namespace cv;
using namespace std;

/* Drone Publisher */
ros::Publisher rostopictakeoff;
ros::Publisher rostopiclanding;
ros::Publisher cmd_vel;
ros::ServiceClient flattrim;
// ros::Publisher topicinfo;
// float logitud, latitud, altitud;

geometry_msgs::Twist changeTwist(float x, float y, float z, float turn)
{
    geometry_msgs::Twist msg_vel;
    msg_vel.angular.x = 0;
    msg_vel.angular.y = 0;
    msg_vel.angular.z = turn; //turn right and left
    msg_vel.linear.x = x; //move horizontally --> forward and backward
    msg_vel.linear.y = y; //move horizontally --> left and right
    msg_vel.linear.z = z; //move vertically --> up and down 
    return (msg_vel);
}

// Stabilisation of the drone after take off or following the target
void ajuste(void)
{
    std_srvs::Empty srvflattrim;
    flattrim.call(srvflattrim);
}

// Take off function including stabilising
void takeoff(void)
{
    std_msgs::Empty empty;
    geometry_msgs::Twist msg_vel;
    rostopictakeoff.publish(empty);
    printf("Taking off\n");
    usleep(300000);
    printf("Stabilisation\n");
    msg_vel = changeTwist(0, 0, 0, 0);
    cmd_vel.publish(msg_vel);
}

// Land function
void land(void)
{
    std_msgs::Empty empty;
    rostopiclanding.publish(empty);
}

// All the drone functions used to move the drone according to the circle
void forward(float a)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(a, 0, 0, 0);
    cmd_vel.publish(msg_vel);
    printf("Going forward\n");
}

void turnLeft(float a)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, a);
    cmd_vel.publish(msg_vel);
    printf("Turning left\n");
}

void back(float a)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(a, 0, 0, 0);
    cmd_vel.publish(msg_vel);
    printf("Going backwards\n");
}

void turnRight(float a)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, a);
    cmd_vel.publish(msg_vel);
    printf("Turning Right\n");
}

void up(float a)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, a, 0);
    cmd_vel.publish(msg_vel);
    printf("Going up\n");
}

void down(float a)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, a, 0);
    cmd_vel.publish(msg_vel);
    printf("Going down\n");
}

void doEverything(string direction, float r)
{
 float acc;

    if (direction == "forward") {
    acc = 1-((r/50) * 1);
     if (acc > 1){acc = 1;}
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(acc, 0,0,0);
    cmd_vel.publish(msg_vel);
    printf("do everything forward %f \n", acc );
   }

    if (direction == "backward") {
    acc = (r/100) * (-1);
     if (acc < -1){acc = -1;}
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(acc, 0,0,0);
    cmd_vel.publish(msg_vel);
    printf("do everything backward %f \n", acc);
    }
   
    if (direction == "up") {
    acc = (r/100) * 1;
     if (acc > 1){acc = 1;}
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,acc,0);
    cmd_vel.publish(msg_vel);
    printf("do everything up %f \n", acc);
    }

    if (direction == "down") {
    acc = (r/100) * (-1);
     if (acc < -1){acc = -1;}
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,acc,0);
    cmd_vel.publish(msg_vel);
    printf("do everything down %f \n",acc);
    }

    if (direction == "turnLeft") {
	acc = (r/200) * 1;
     if(acc > 1){acc = 1;}
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,0,acc);
    cmd_vel.publish(msg_vel);
    printf("do everything left %f \n",acc);

    }

    if (direction == "turnRight") {
       acc = (r/200) * (-1);
     if (acc < -1) {acc = -1;}
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,0,acc);
    cmd_vel.publish(msg_vel);
    printf("do everything right %f \n", acc);
    }

}

// Stopping the drone --> it results in hoverring mode
void stop(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, 0);
    cmd_vel.publish(msg_vel);
    printf("Stopping the drone.\n");
}




static const std::string OPENCV_WINDOW = "Image window";

// Class used for converting the incomming drone camera image to the HSV recognized image
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub_depth;
    image_transport::Subscriber image_sub_threshold;
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub_depth;
    image_transport::Publisher image_pub_threshold;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        //printf("subscribed");
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        //printf("published");
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //-----------------------------------------------------------------------------------------

        // Convert input image to HSV
        cv::Mat HSVImage;
        cv::cvtColor(cv_ptr->image, HSVImage, COLOR_BGR2HSV);

        cv::Size size = HSVImage.size();
        cv::Mat mask = cvCreateMat(size.height, size.width, CV_8UC1); //CV_8UC1 = grayscle mask

        // Detecting colors --> starting with higher contrast to lower for the colors
        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(HSVImage, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(HSVImage, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

        // Combine the images from the range lower and the upper range
        cv::Mat red_hue_image;
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
        cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

        // Morphological closing (fill small holes in the foreground) --> removing the noise from the images and combining the elemenets
        dilate(mask, mask, getStructuringElement(0, Size(21, 21)));
        erode(mask, mask, getStructuringElement(0, Size(10, 10)));
        erode(mask, mask, getStructuringElement(0, Size(11, 11)));
        dilate(mask, mask, getStructuringElement(0, Size(5, 5)));

        // Use the Hough transform to detect circles in combined threshold images
        vector<Vec3f> circles;
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 100, 20, 0, 0); // hough transform

        // Processing the circle and recognising it's center
        float x, y, r;
        for (size_t i = 0; i < circles.size(); i++) //i++ of ++i?
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle(mask, center, 3, Scalar(0, 255, 8), -1, 8, 0);
            // circle outline
            circle(mask, center, radius, Scalar(0, 0, 255), 3, 8, 0);
            // circle center
            circle(cv_ptr->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(cv_ptr->image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
            x = circles[i][0];
            y = circles[i][1];
            r = circles[i][2];
        }
        if (circles.size() > 0)
        {
            printf("Coordinates x: %f \n Coordinates y: %f \n radial %f \n", x, y, r);
        }

        bool temp1, temp2;       // help or temporary variables
        if (circles.size() == 0) // no circle detected
        {
            stop();
            printf("no Circle\n");
        }

        if (r < 20 && circles.size() > 0) // circle detected, but it's too small --> flying forward
        {

            doEverything("forward", r);

            printf("forward \n");
            temp1 = true;
        }


        if (r > 50) // circle detected, but it's too big --> flying backward
        {
            doEverything("backward", r);
            printf("back\n");
            temp1 = true;
        }

        if (r < 40 && r > 20) // Normal circle detected
        {
            if (temp1 == true)
            {
                stop();
                temp1 = false;
            }

            if (y < 200 && circles.size() > 0) // Lower half of the circle detected --> going up
            {
                doEverything("up", y);
                printf("up \n");
                temp2 = true;
            }


            if (y > 250 && circles.size() > 0) // Upper half of the circle detected --> going down
            {
                doEverything("down", y);
                printf("down\n");
                temp2 = true;
            }


            if (y < 300 && y > 200) // Normal circle detected --> ajustment to left or right
            {
                if (temp2 == true)
                {
                    stop();
                    temp2 = false;
                }

                if (x > 450 && circles.size() > 0) // The left side of the circle detected --> going right
                {
                    doEverything("turnRight", x);
                    printf("turn right\n");
                }

                
                if (x < 250 && circles.size() > 0) // The right side of the circle detected --> going left
                {
                    doEverything("turnLeft", x);
                    printf("turn left\n");
                }

                
                if (x < 350 && x > 250)
                {
                    stop();
                }
            }
        }

        // Debugging the code + caliberating the HSV
        cv::imshow(OPENCV_WINDOW, cv_ptr->image); //show detected object
        cv::imshow("Threshold lower image", lower_red_hue_range);
        cv::namedWindow("Threshold upper image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Threshold upper image", upper_red_hue_range);
        cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined threshold images", red_hue_image);
    }
};

// The main function for executing all the functions above!
int main(int argc, char **argv)
{
    // -------- DRONE PART
    ros::init(argc, argv, "drone");
    ros::NodeHandle n;
    rostopictakeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);
    rostopiclanding = n.advertise<std_msgs::Empty>("/ardrone/land", 1, true);
    cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    flattrim = n.serviceClient<std_srvs::Empty>("/ardrone/flattrim");
    ajuste();
    printf("Calibration \n");
    takeoff();

    // ------ IMAGE PART
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    while (1)
    {
        ros::spinOnce();
        if (waitKey(30) == 27)
        {
            cout << "esc pressed" << endl;
            break;
        }
    }
    return 0;
}
