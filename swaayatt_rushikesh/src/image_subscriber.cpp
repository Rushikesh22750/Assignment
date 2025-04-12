#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace std;

class ImageProcess {
public:

    ImageProcess() {

    pub = nh.advertise<sensor_msgs::Image>("/processed_image", 1);

 ////// I am using Intel realsense camera. This node is subscribing to that.

        sub = nh.subscribe("/camera/color/image_raw", 1, & ImageProcess::imageCallback, this);
    
    }

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;


///// Converting RBG to BGR for opencv which is used later in the visualization node.
//////// I also tried normalizing and resizing it but didnt worked (was showing many bounding boxes)so did the preprocessing with numpy in yolo node


    void convertandnormalize(sensor_msgs::Image * msg) {

    //vector<float> normalized_data(msg->data.size());

        for (int i = 0; i < msg->data.size(); i += 3) {
            
            // Swap R and B channels to convert to BGR
            int temp = msg->data[i];    
            msg->data[i] = msg->data[i + 2];  
            msg->data[i + 2] = temp;           

    

            // Normalizizing
            //normalized_data[i] = static_cast<float>(msg->data[i]) / 255.0f;         // Red
           //normalized_data[i + 1] = static_cast<float>(msg->data[i + 1]) / 255.0f; // Green
        //normalized_data[i + 2] = static_cast<float>(msg->data[i + 2]) / 255.0f; // Blue
        }

       // Copying back for publishing
        //for (size_t i = 0; i < msg->data.size(); i += 3) {
        //msg->data[i] = static_cast<uint8_t>(normalized_data[i] * 255.0f);         // Red
           //msg->data[i + 1] = static_cast<uint8_t>(normalized_data[i + 1] * 255.0f); // Green
           // msg->data[i + 2] = static_cast<uint8_t>(normalized_data[i + 2] * 255.0f); // Blue
        //}
    }

    
        /// Getting data from ROS images

        sensor_msgs::Image* preprocess(const sensor_msgs::ImageConstPtr & msg) {
        
        // Creating a new image message that we will publish

        sensor_msgs::Image* processed_msg = new sensor_msgs::Image();

        processed_msg->header = msg->header;
        processed_msg->height = msg->height;
        processed_msg->width = msg->width;

        processed_msg->encoding = "bgr8";  
        processed_msg->step = msg->step;

        
        for (int i = 0; i < msg->data.size(); i++) {

            processed_msg->data.push_back(msg->data[i]);

        }

        convertandnormalize(processed_msg);

        return processed_msg;
    }


    // This is called every time a new image is received

    void imageCallback(const sensor_msgs::ImageConstPtr & msg) {

        sensor_msgs::Image* processed_msg = preprocess(msg);
        
        // Publish
        pub.publish(*processed_msg);

        ROS_INFO("publishing processed image to /processed_image in bgr format");

        delete processed_msg;

    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "image_processor");

    ImageProcess processor;
    ros::spin();

    return 0;
}
