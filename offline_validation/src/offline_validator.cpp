#include "offline_validation/offline_validator.hpp"

OfflineValidator::OfflineValidator(
  ros::NodeHandle nh,
  wave::Tracker<wave::ORBDetector, wave::ORBDescriptor, wave::BruteForceMatcher>
    tracker,
  std::string camera_topic)
    : nh(nh), feature_tracker(tracker) {
    // Initialize image subscriber
    this->image_sub =
      nh.subscribe(camera_topic, 1, &OfflineValidator::imageCallback, this);

    // Advertise publishers
    this->image_pub =
      nh.advertise<sensor_msgs::Image>("/offline_validation/image_tracks", 10);
    this->diagnostics_pub =
      nh.advertise<offline_validation::FrontEndDiagnostics>(
        "/offline_validation/diagnostics", 10);
}

// Define image callback
void OfflineValidator::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        // Extract image from cv_bridge
        this->curr_image = cv_bridge::toCvShare(msg)->image;
        this->curr_header = msg->header;
        this->curr_encoding = msg->encoding;
        this->img_count++;
        auto time = this->clock.now();

        // Add image to feature_tracker.
        this->feature_tracker.addImage(this->curr_image, time);
        this->validateImage();

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from %s to BGR8", msg->encoding.c_str());
    }
}

double OfflineValidator::calcAvgTrackLength(
  const std::vector<wave::FeatureTrack> &tracks) {
    double avg;

    // Get number of tracks
    size_t num_tracks = tracks.size();
    this->cum_num_tracks += num_tracks;

    size_t length_sum = 0;
    for (const auto &t : tracks) {
        length_sum += t.size();
    }
    this->cum_track_length += length_sum;

    avg = (double) length_sum / num_tracks;

    return avg;
}

cv_bridge::CvImage OfflineValidator::getImageMsg(const cv::Mat &image) {
    cv_bridge::CvImage image_msg;

    image_msg.header = this->curr_header;
    image_msg.encoding = this->curr_encoding;
    image_msg.image = image;

    return image_msg;
}

offline_validation::FrontEndDiagnostics OfflineValidator::getDiagnostics(
  const std::vector<wave::FeatureTrack> &curr_tracks) {
    offline_validation::FrontEndDiagnostics diagnostics;

    // Enter the track information
    diagnostics.avg_track_length = this->calcAvgTrackLength(curr_tracks);
    diagnostics.cumulative_avg_track_length =
      (double) this->cum_track_length / this->cum_num_tracks;
    diagnostics.num_tracks = curr_tracks.size();

    // Enter the detector information
    diagnostics.num_keypoints =
      this->feature_tracker.detector.num_keypoints_detected;

    // Enter the matcher information
    diagnostics.num_raw_matches = this->feature_tracker.matcher.num_raw_matches;
    diagnostics.num_filtered_matches =
      this->feature_tracker.matcher.num_filtered_matches;
    diagnostics.num_good_matches =
      this->feature_tracker.matcher.num_good_matches;

    // Enter the tracker information
    diagnostics.lmc_size = this->feature_tracker.lmc_size;

    return diagnostics;
}

void OfflineValidator::publishDiagnostics(
  const cv::Mat &image, const std::vector<wave::FeatureTrack> &curr_tracks) {
    // Create msg objects
    cv_bridge::CvImage img_msg = this->getImageMsg(image);
    offline_validation::FrontEndDiagnostics diagnostics_msg =
      this->getDiagnostics(curr_tracks);

    // Publish info
    this->image_pub.publish(img_msg);
    this->diagnostics_pub.publish(diagnostics_msg);
}

void OfflineValidator::validateImage() {
    // Only can get tracks if more than one image has been received
    if (this->img_count > 1) {
        size_t img = this->img_count - 1;

        std::vector<wave::FeatureTrack> curr_tracks =
          this->feature_tracker.getTracks(img);

        cv::Mat image_ft =
          this->feature_tracker.drawTracks(curr_tracks, this->curr_image);

        // Publish info
        this->publishDiagnostics(image_ft, curr_tracks);
    }
}
