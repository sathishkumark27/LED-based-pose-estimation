#ifndef BLOBDETECTOR
#define BLOBDETECTOR

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <algorithm>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace LED_pose_estimator
{

#define NUM_FRAMES_HISTORY  2
#define MAX_BLOBS_PER_FRAME 20
#define BLINK_FREQ_FRAMES 4 /* each bit can be decoded using 4 successive frames */
#define NUM_OF_LEDS 10

typedef Eigen::Matrix< Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints;

     struct blob {
             /* center of bounding box */
             cv::Point2f  Center;
             cv::Point2f  DiffCenter;
             /* bounding box */
             cv::Rect rect;
             double area;
             double last_area;            
             uint32_t age;
             int16_t track_index;
             uint16_t pattern;
             uint16_t patternCrrctd;
             int16_t led_id;
             //Vector3d worldpoint;
             cv::Point3d worldpoint;
             char color;
             cv::Point2f  ExpCenterInNxtFrm;
     };

     /*
      * Stores all blobs observed in a single frame.
      */
     struct blobservation {
             int num_blobs;
             //struct blob blobs[MAX_BLOBS_PER_FRAME];
             std::vector<blob> blobs;
             int tracked_blobs;
             uint8_t tracked[MAX_BLOBS_PER_FRAME];
             cv::Mat gray;
             std::vector<cv::Point2f> distorted_points;
             std::vector<cv::Point2f> expected_locs;
     };

 /*    struct extent {
             uint16_t start;
             uint16_t end;
             / * inherited parameters * /
             uint16_t top;
             uint16_t left;
             uint16_t right;
             uint8_t index;
             uint32_t area;
     };

     struct extent_line {
             struct extent extents[MAX_EXTENTS_PER_LINE];
             uint16_t num;
             uint16_t padding[3];
     };
*/

     struct LEDInfo
     {
         uint8_t LEDId;
         uint32_t pattern;
         cv::Point3d worldpoint;
     };

     extern LEDInfo gLEDsInfo[NUM_OF_LEDS];

     struct blobtrack {
         int width;
         int height;
         int last_observation;
         struct blobservation history[NUM_FRAMES_HISTORY];
         struct extent_line *el;
         bool debug;
         struct flicker *fl;
         bool isSyncPtrnFnd;
         unsigned DataFrmCnt;
         bool isWldPntInitDn;         
        };
     void FindLEDs(const cv::Mat &image, cv::Rect ROI, blobtrack **BlobTrack, const int &threshold_value, const double &gaussian_sigma,
                          const double &min_blob_area, const double &max_blob_area,
                          const double &max_width_height_distortion, const double &max_circular_distortion,
                          List2DPoints &pixel_positions, std::vector<cv::Point2f> &distorted_detection_centers,
                          const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                           blobservation **BlobOb, int width, int height);

     void TrackBlobs(const cv::Mat &image, cv::Rect ROI, int width, int height, blobtrack **BlobTrack, blobservation **BlobOb);
     bool  isAllLEDsFound(blob *blobs, int num_blobs);
     void MotionEstimation(const cv::Mat &image, blobservation *CurrOb, blobservation *LastOb);


} // namespace



#endif // BLOBDETECTOR

