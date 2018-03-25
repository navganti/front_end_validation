# Custom message containing diagnostics of the front end.

# Averages for track length, in both the current photo and cumulative over all
# images
float64 avg_track_length
float64 cumulative_avg_track_length

# The number of tracks in the current image
uint64 num_tracks

# The number of keypoints detected in the current image
uint64 num_keypoints

# Information regarding matches, including the initial number of matches, the
# number of matches after filtering (ratio test/ distance test), and the number
# of good matches remaining after full outlier removal through RANSAC (or
# similar)
uint64 num_raw_matches
uint64 num_filtered_matches
uint64 num_good_matches

# The size of the LandmarkMeasurementContainer within the Tracker. If in online
# mode, this should not grow too large as old features are thrown out. Else,
# will steadily grow.
uint64 lmc_size
