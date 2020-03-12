#ifndef BLINKPATTERN
#define BLINKPATTERN

#include "blobdetector.h"
#include "camera_blob_cfg.h"
#include <iostream>
#include <cstddef>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace LED_pose_estimator
{ 
  void FindBlinkPattern(blobservation *CurrOb);
  //bool IsSyncPatternFound(blobservation *CurrOb);
  bool IsSyncPatternFound(blob *blobs, int num_blobs);
  void DataErrCorrectionFor20BitPattern(blob *blobs, int num_blobs);
  void DataErrCorrection(blob *blobs, int num_blobs);
  void ErrCorrection(blob *blobs);
  void CorrectPattern(blobservation *CurrOb);
  bool GetLEDInfoFromPattern(blobservation *CurrOb);
  void FindColorPattern(blobservation *CurrOb);
} //namespace end

#endif // BLINKPATTERN

