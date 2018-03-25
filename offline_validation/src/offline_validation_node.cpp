#include "offline_validation/offline_validator.hpp"

int main(int argc, char **argv) {
    // Set up ROS node
    ros::init(argc, argv, "offline_validation_node");
    ros::NodeHandle nh;

    // Declare variables to be modified by launch file or command line
    std::string camera_topic = "/image";

    // Define parameters for detector, descriptor, and matcher
    wave::ORBDetectorParams detector_params;
    detector_params.num_features = 1500;

    // wave::FASTDetectorParams detector_params;
    // detector_params.num_features = 350;

    wave::ORBDescriptorParams descriptor_params;
    // wave::FLANNMatcherParams matcher_params;
    // matcher_params.flann_method = wave::FLANN::KDTree;
    wave::BFMatcherParams matcher_params;
    matcher_params.fm_method = cv::FM_RANSAC;
    matcher_params.ratio_threshold = 0.9;

    // Define detector, descriptor, and matcher
    wave::ORBDetector detector(detector_params);
    wave::ORBDescriptor descriptor(descriptor_params);
    // wave::FLANNMatcher matcher(matcher_params);
    wave::BruteForceMatcher matcher(matcher_params);

    int window_size = 5;

    // Create tracker
    wave::Tracker<wave::ORBDetector,
                  wave::ORBDescriptor,
                  wave::BruteForceMatcher>
      tracker(detector, descriptor, matcher, window_size);

    // Create validator
    OfflineValidator validator(nh, tracker, camera_topic);

    ros::spin();

    return 0;
}
