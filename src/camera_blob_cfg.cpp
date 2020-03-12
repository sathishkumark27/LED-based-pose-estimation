#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "camera_blob_cfg.h"

#include <algorithm>

using namespace std;
using namespace cv;

namespace LED_pose_estimator
{
/*
    void GetCameraMatrix (cv::Mat& Camera_Matrix_K)
    {
        double focal_length_x = 670.5071954647542;//640; // Approximate focal length.
        double focal_length_y = 672.7481061588313;//670.5071954647542;
        Point2d center = cv::Point2d(349.6604015585297,262.7841100242821); //cv::Point2d(320,240);
        Camera_Matrix_K = (cv::Mat_<double>(3,3) << focal_length_x, 0, center.x, 0 , focal_length_y, center.y, 0, 0, 1);
        return;
    }

    void GetDistortionCoeff(vector<double>  &Distortion_Matrix)
    {
        //distCoeffs : [0.05743682699263847, -0.5564809343353944, 0.008220886702309891, 0.003388386142362759, 0.6581419535087263]
        Distortion_Matrix.push_back(0.05743682699263847);
        Distortion_Matrix.push_back(-0.5564809343353944);
        Distortion_Matrix.push_back(0.008220886702309891);
        Distortion_Matrix.push_back(0.003388386142362759);
        Distortion_Matrix.push_back(0.6581419535087263);

        cout<<"Distrotion coeffs initilized to camera model"<<endl;
        return;
    }
*/

/*

void FillLedPatternsTable(LEDPatternsTable *GlbPatternTable)
{
    GlbPatternTable->num_patterns = 2;
    GlbPatternTable->patterns[0] = 0X13;
    GlbPatternTable->patterns[1] = 0X15;
}

*/

}
