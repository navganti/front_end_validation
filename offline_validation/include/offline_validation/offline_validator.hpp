#ifndef OFFLINE_VALIDATION_NODE_HPP
#define OFFLINE_VALIDATION_NODE_HPP

// Wave
#include <wave/vision/detector/orb_detector.hpp>
#include <wave/vision/detector/fast_detector.hpp>
#include <wave/vision/descriptor/orb_descriptor.hpp>
#include <wave/vision/matcher/brute_force_matcher.hpp>
#include <wave/vision/matcher/flann_matcher.hpp>
#include <wave/vision/tracker/tracker.hpp>
#include <wave/vision/utils.hpp>
#include <wave/utils/utils.hpp>

// C++
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "offline_validation/FrontEndDiagnostics.h"

// OpenCV
#include <cv_bridge/cv_bridge.h>

class OfflineValidator {
 public:
    /** Constructor.
     *
     * @param tracker the image tracker object
     * @param nh the node handle
     * @param camera_topic the topic to subscribe to
     */
    OfflineValidator(ros::NodeHandle nh,
                     wave::Tracker<wave::ORBDetector,
                                   wave::ORBDescriptor,
                                   wave::BruteForceMatcher> tracker,
                     std::string camera_topic);

    ~OfflineValidator() = default;

    /** Callback for every new image published on camera_topic.
     *
     * @param msg the data subscribed to.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    /** Calculates the average track length for a vector of tracks.
     *
     * @param tracks the feature tracks extracted from an image.
     * @return the average value of the track lengths.
     */
    double calcAvgTrackLength(const std::vector<wave::FeatureTrack> &tracks);

    /** Convert an image into the required format for publishing.
     *
     * @param image the image information.
     * @return the message to publish.
     */
    cv_bridge::CvImage getImageMsg(const cv::Mat &image);

    offline_validation::FrontEndDiagnostics getDiagnostics(
      const std::vector<wave::FeatureTrack> &curr_tracks);

    void publishDiagnostics(const cv::Mat &image,
                            const std::vector<wave::FeatureTrack> &curr_tracks);

    void validateImage();

 private:
    // ROS
    ros::NodeHandle nh;

    // ROS variables
    std_msgs::Header curr_header;
    std::string curr_encoding;

    // Publisher and subscriber
    ros::Subscriber image_sub;
    ros::Publisher image_pub;
    ros::Publisher diagnostics_pub;

    // Tracker Object
    wave::Tracker<wave::ORBDetector,
                  wave::ORBDescriptor,
                  wave::BruteForceMatcher>
      feature_tracker;

    // Timekeeping
    std::chrono::steady_clock clock;
    size_t img_count = 0;

    // Diagnostic info
    size_t cum_track_length = 0;
    size_t cum_num_tracks = 0;

    // Current image for drawing tracks
    cv::Mat curr_image;
};

#endif  // OFFLINE_VALIDATION_NODE_HPP
