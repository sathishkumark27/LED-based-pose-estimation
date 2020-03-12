#ifndef CAMERA_BLOB_CFG
#define CAMERA_BLOB_CFG
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <algorithm>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace LED_pose_estimator
{

    struct BlobCfg
    {
        int threshold_value = 159;//220;
        double gaussian_sigma = 0.6;
        double min_blob_area = 10;//20;//10;//100;//50//10;
        double max_blob_area = 400;//120;//800;//200;
        double max_width_height_distortion = 0.5;
        double max_circular_distortion = 0.5;
        double back_projection_pixel_tolerance = 43;//3;
        double nearest_neighbour_pixel_tolerance = 50;//5;
        double certainty_threshold = 0.75;
        double valid_correspondence_threshold = 0.7;
        double roi_border_thickness = 10;
    };

    struct LEDPatternsTable
    {
        uint16_t patterns[10] = {0X7000, 0X0E00, 0X7E00, 0X01C0, 0X71C0, 0X0FC0, 0X7FC0, 0X0038, 0X7038, 0X0E38};
        unsigned num_patterns = 10;
    };

}


#endif // CAMERA_BLOB_CFG

