#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <swaayatt_rushikesh/DetectedObjects.h>
#include <swaayatt_rushikesh/DetectedObject.h>
#include <swaayatt_rushikesh/TrackedObjects.h>
#include <swaayatt_rushikesh/TrackedObject.h>

// OpenCV for Kalman filter
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;

class ObjectTracker {
  
public:

  ObjectTracker() {


    ros::NodeHandle nh;

    /// subscribing to /object_detection

    subDetection = nh.subscribe("/object_detection", 1,  & ObjectTracker :: detectionCallback, this);

    /// publishing to /object_tracking

    pubTracking = nh.advertise <swaayatt_rushikesh :: TrackedObjects> ("/object_tracking", 1);

    // ID counter for new tracks
    nextId = 0;

   
    maxMissed = 10;        /// remove a track if it is not matched for 10 consecutive frames
    distThreshold = 50.0f;   /// maximum allowed matching distance


  }


  ros::Subscriber subDetection;
  ros::Publisher pubTracking;

  
  vector<int> trackIds;         // Unique ID for each track
  vector<cv::KalmanFilter> kfs;   // Kalman filter for each track
  vector<int> missedFrames;      // Missed detection count for each track
  vector<float> widths;          // Latest bounding box width for each track
  vector<float> heights;         // Latest bounding box height for each track

  int nextId;
  int maxMissed;
  float distThreshold;


  
  //call back receives detections and updates the tracks

  void detectionCallback(const swaayatt_rushikesh::DetectedObjects::ConstPtr &msg){


    double dt = 0.1; ////time step

    ///Predict the state for each active track

    for (size_t i = 0; i < kfs.size(); i++){

      predictTrack(i, dt);

    }

    ///  for each detection find the best matching track by distance
    //// matchedtrackindex[d] stores the track index that best matches the detection

    vector<int> matchedtrackindex(msg->objects.size(), -1);

    vector<bool> trackalreadymatched(trackIds.size(), false);


    for (int d = 0; d < msg->objects.size(); d++){

      const auto &det = msg->objects[d];

      // calculating the center

      float cx_det = det.x + det.width * 0.5f;
      float cy_det = det.y + det.height * 0.5f;

      int best_track = -1;
      float best_dist = distThreshold; // only accept if distance is less than this



      for (int t = 0; t < kfs.size(); t++){

        if (trackalreadymatched[t]) {

          continue; // Skip tracks already matched for this cycle

        }

        //// getting predicted center from the Kalman filter

        float tx = kfs[t].statePost.at<float>(0);
        float ty = kfs[t].statePost.at<float>(1);

        float dist = std::sqrt((cx_det - tx) * (cx_det - tx) + (cy_det - ty) * (cy_det - ty));

        if (dist < best_dist){

          best_dist = dist;
          best_track = t ;

        }

      }


      if (best_track >= 0){

        // Record that detection d matches track best_track
        matchedtrackindex[d] = best_track;
        trackalreadymatched[best_track] = true;

      }

    }


    ////Update matched tracks with the new detection and create new tracks for unmatched detections

    for (int d = 0; d < msg->objects.size(); d++){

      int t_index = matchedtrackindex[d];
      const auto &det = msg->objects[d];
      
      if (t_index >= 0){

        updateTrack(t_index, det);

        // Reset missedFrames count for the matched track
        missedFrames[t_index] = 0;

      }

      else{

        // No track matched this detection we create a new track.
        createTrack(det);

      }
    }


    ///// for each track that was not matched we increment missedFrames

    for (size_t i = 0; i < trackIds.size(); i++){

      if (i >= trackalreadymatched.size() || !trackalreadymatched[i]){

        missedFrames[i]++;

      }

    }


    /// Remove tracks that have missed too many frames.
  
    for (int i = trackIds.size() - 1; i >= 0; i--){

      if (missedFrames[i] > maxMissed) {

        trackIds.erase(trackIds.begin() + i);
        kfs.erase(kfs.begin() + i);
        missedFrames.erase(missedFrames.begin() + i);
        widths.erase(widths.begin() + i);
        heights.erase(heights.begin() + i);

      }

    }

    /// publish the updated track states

    swaayatt_rushikesh::TrackedObjects track_msg;
    track_msg.header = msg->header; // using the same header as the detections

    // For each active track we build a TrackedObject message

    for (size_t i = 0; i < trackIds.size(); i++) {

      swaayatt_rushikesh::TrackedObject tobj;
      tobj.id = trackIds[i];

      float x  = kfs[i].statePost.at<float>(0);
      float y  = kfs[i].statePost.at<float>(1);
      float vx = kfs[i].statePost.at<float>(2);
      float vy = kfs[i].statePost.at<float>(3);
      
      tobj.x = x;
      tobj.y = y;
      tobj.vx = vx;
      tobj.vy = vy;
      tobj.width  = widths[i];
      tobj.height = heights[i];
      
      track_msg.objects.push_back(tobj);

    }

    pubTracking.publish(track_msg);

  }



  void predictTrack(int idx, double dt){

/// Update the transition matrix with the current time step

    kfs[idx].transitionMatrix.at<float>(0,2) = dt;
    kfs[idx].transitionMatrix.at<float>(1,3) = dt;

/// Predict the next state using the Kalman filter
    kfs[idx].predict();

  }



  void createTrack(const swaayatt_rushikesh::DetectedObject &det){

//////// Initializing the KALMAN FILTER

    cv::KalmanFilter kf;
    kf.init(4, 2, 0);
    float dt = 0.1f;

    cv::setIdentity(kf.transitionMatrix);
    kf.transitionMatrix.at<float>(0,2) = dt;
    kf.transitionMatrix.at<float>(1,3) = dt;
    
    cv :: setIdentity(kf.measurementMatrix);
    cv :: setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
    cv :: setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv :: setIdentity(kf.errorCovPost, cv::Scalar::all(1));


/// calculating the center of the bounding box

    float cx = det.x + det.width * 0.5f;
    float cy = det.y + det.height * 0.5f;


    // setting the initial state as center position and zero velocity

    kf.statePost.at<float>(0) = cx;
    kf.statePost.at<float>(1) = cy;

    kf.statePost.at<float>(2) = 0; // initial velocity in x
    kf.statePost.at<float>(3) = 0; // initial velocity in y

 
    trackIds.push_back(nextId++);
    kfs.push_back(kf);
    missedFrames.push_back(0);
    widths.push_back(det.width);
    heights.push_back(det.height);

    ROS_INFO("Created new track ID=%d at (%.1f, %.1f)", trackIds.back(), cx, cy);

  }

 

  void updateTrack(int idx, const swaayatt_rushikesh::DetectedObject &det){

  
    float cx = det.x + det.width * 0.5f;

    float cy = det.y + det.height * 0.5f;

    cv::Mat meas(2, 1, CV_32F);
    meas.at<float>(0) = cx;
    meas.at<float>(1) = cy;

    // Correct the Kalman filter state with the new measurement
    kfs[idx].correct(meas);

    // Update the stored width and height
    widths[idx] = det.width;
    heights[idx] = det.height;
  }

};

int main(int argc, char** argv){

  ros::init(argc, argv, "object_tracker");

  ObjectTracker tracker;

  ros::spin();
  return 0;

}
