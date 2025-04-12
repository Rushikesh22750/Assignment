
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <swaayatt_rushikesh/DetectedObjects.h>
#include <swaayatt_rushikesh/TrackedObjects.h>
#include <map>
#include <vector>

/////// Used open cv for visualization purpose
#include <opencv2/opencv.hpp>

using namespace std;


class VisualizationNode {

public:

    VisualizationNode () {

    subImage = nh.subscribe("/processed_image", 1 , & VisualizationNode::imageCallback, this);
    subDetection = nh.subscribe("/object_detection", 1 , & VisualizationNode::detectionCallback, this);

    subTracking = nh.subscribe("/object_tracking", 1 , & VisualizationNode::trackingCallback, this);

    pubVis = nh.advertise<sensor_msgs::Image>("/visualization/image", 1);

    }


    ros::NodeHandle nh;
    ros::Subscriber subImage;
    ros::Subscriber subDetection;
    ros::Subscriber subTracking;
    ros::Publisher pubVis;

    cv::Mat currentFrame;

    // for storing the latest detection and tracking data

    swaayatt_rushikesh::DetectedObjects currentDetections;
    swaayatt_rushikesh::TrackedObjects currentTracks;


    /// Map for recording track id and trajectory history
    map<int, vector<cv::Point>>  trackHistories;


    ///// Callback for image

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg){

        cv::Mat temp(msg->height, msg->width, CV_8UC3, (void *)&msg->data[0], msg->step);
        temp.copyTo(currentFrame);
        annotateAndPublish();

    }



    ///// Callback for detections

    void detectionCallback(const swaayatt_rushikesh::DetectedObjects::ConstPtr &msg){

        currentDetections = * msg;

    }


    ///// Callback for tracking

    void trackingCallback(const swaayatt_rushikesh::TrackedObjects::ConstPtr &msg){

        currentTracks = * msg;

        
        // Update the track histories for each tracked object
        for (auto &tobj : currentTracks.objects){

            cv::Point pt((int)tobj.x, (int)tobj.y);

            // Add the new point to the object's history

            trackHistories[tobj.id].push_back(pt);

            if (trackHistories[tobj.id].size() > 100){

                trackHistories[tobj.id].erase(trackHistories[tobj.id].begin()); // Remove the old points
            }
        }
    }



    void annotateAndPublish(){

        // Make a copy of the image to annotate
        cv::Mat annotatedImage = currentFrame.clone();

    //// Draw bounding boxes, labels and confidence

        for (auto & obj : currentDetections.objects){

            int x = (int)obj.x;
            int y = (int)obj.y;
            int w = (int)obj.width;
            int h = (int)obj.height;

            cv::Rect box(x, y, w, h);
            cv::rectangle (annotatedImage, box ,  cv::Scalar(0, 255, 0), 2);
            string label = obj.class_name +  " "  + to_string(obj.confidence).substr(0, 4);

            cv::putText(annotatedImage, label, cv::Point(x, y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        }


    //// Draw ID and trajectory

        for (auto &tobj : currentTracks.objects){

            int trackId = tobj.id;

            // Current position
            cv::Point center((int)tobj.x, (int)tobj.y);
            cv::circle(annotatedImage, center, 4, cv::Scalar(0, 0, 255), -1);

            // Display the ID
            string idStr = "ID:" + to_string(trackId);
            cv::putText(annotatedImage, idStr, cv::Point(center.x + 5, center.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

            // Draw the trajectory
            auto &history = trackHistories[trackId];
            for (size_t i = 1; i < history.size(); i++){

                cv::line(annotatedImage, history[i - 1], history[i], cv::Scalar(255, 0, 0), 2);


            }
        }


        // Publish the annotated image

        sensor_msgs::Image visMsg;
        visMsg.header.stamp = ros::Time::now();
        visMsg.header.frame_id = "camera_frame";
        visMsg.height = annotatedImage.rows;
        visMsg.width = annotatedImage.cols;
        visMsg.encoding = "bgr8";
        visMsg.is_bigendian = false;
        visMsg.step = annotatedImage.cols * annotatedImage.elemSize();
        visMsg.data.resize(annotatedImage.total() * annotatedImage.elemSize());

        
        // Copy data from OpenCV Mat to ROS message
       

        for (size_t i = 0; i < annotatedImage.total(); i++){
          visMsg.data[i] = annotatedImage.data[i];
        }

        pubVis.publish(visMsg);

        
    ///// Visualization
        cv::imshow("Annotated Image", annotatedImage);
        cv::waitKey(1);

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualization_node");
    VisualizationNode node;
    ros::spin();

    return 0;
}
