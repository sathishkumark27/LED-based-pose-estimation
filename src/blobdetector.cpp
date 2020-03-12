#include "blobdetector.h"
#include <iostream>
#include "camera_blob_cfg.h"
#include "camera_blob_cfg.cpp"
#include "blinkpattern.h"
#include <opencv2/core/eigen.hpp>

using namespace std;

using namespace cv;
using namespace Eigen;

namespace LED_pose_estimator
{

    int globcount = 0;
    int gframe_width = 0;
    int gframe_height = 0;
    cv::Mat iphone_camera_matrix;
    cv::Mat iphone_dist_coeffs;

    cv::Mat rotation_vector(3, 1, CV_64F); // Rotation in axis-angle form
    cv::Mat translation_vector(3, 1, CV_64F);

    //LEDInfo gLEDsInfo[NUM_OF_LEDS];

    /*    image.rows()848
    image.cols()480
    */
    //double focal_length = 480; // Approximate focal length.
    //Point2d center = cv::Point2d(480/2,848/2);

    VideoWriter video1;

    VideoWriter video2;





    static void StoreBlobs(unsigned NumOfBlobs, std::vector<cv::Point2f> centers, vector<double> areas, vector<cv::Rect> blobrect,
                           std::vector<blob> &blobs, vector<char> blobcolor)
    {
        int i;
        //blob *b = NULL;
        blob tempblob;
        //cout<<"StoreBlobs called for frame :"<<globcount<<endl;

       /* if (NULL == blobs)
        {
            cout<<"StoreBlob input blob is NULL"<<endl;
            return;
        }*/

        //b = blobs;

        for (i=0; i < NumOfBlobs; i++)
        {

            tempblob.Center = centers[i];
            tempblob.DiffCenter.x = 0;
            tempblob.DiffCenter.y = 0;
            tempblob.rect = blobrect[i];
            tempblob.area = areas[i];
            tempblob.age = 0;
            tempblob.track_index = -1;
            tempblob.pattern = 0;
            tempblob.led_id = -1;
            tempblob.worldpoint = cv::Point3d(-1,-1,-1);
            tempblob.color = blobcolor[i];
            blobs.push_back(tempblob);

        }
        return;
    }

    static int find_free_track(uint8_t *tracked)
    {
            int i;

             /*cout<<"find_free_track called for frame :"<<globcount<<endl;
             cout<<"MAX_BLOBS_PER_FRAME :"<<MAX_BLOBS_PER_FRAME<<endl;*/

            for (i = 0; i < MAX_BLOBS_PER_FRAME; i++) {
                    if (tracked[i] == 0)
                    {
                        cout<<"findframe freet track : "<<i<<endl;
                            return i;
                    }
            }

            return -1;
    }

    void InitBlobTrack ( blobtrack *BlobTrack, int width, int height)
    {
        int i, j;
        BlobTrack->width = width;
        BlobTrack->height = height;
        BlobTrack->last_observation = -1;
        BlobTrack->debug = true;
        BlobTrack->isSyncPtrnFnd = false;
        BlobTrack->DataFrmCnt = 0;
        BlobTrack->isWldPntInitDn = false;
        /* chaged blobs array of struct to vector
        blobservation *ob;
        for (j=0;j<NUM_FRAMES_HISTORY;j++)
        {
            ob = &(BlobTrack->history[j]);
            for (i=0;i<MAX_BLOBS_PER_FRAME;i++)
            {
                    ob->blobs[i].Center.x = 0;
                    ob->blobs[i].Center.y = 0;
                    ob->blobs[i].DiffCenter.x = 0;
                    ob->blobs[i].DiffCenter.y = 0;
                    ob->blobs[i].rect = Rect(0, 0, 0, 0);
                    ob->blobs[i].area = 0;
                    ob->blobs[i].last_area = 0;
                    ob->blobs[i].age = 0;
                    ob->blobs[i].track_index = 0;
                    ob->blobs[i].pattern = 0;
                    ob->blobs[i].led_id = 0;
                    ob->blobs[i].worldpoint = cv::Point3d(-1,-1,-1);
                    ob->tracked[i] = 0;

            }
            ob->num_blobs = 0;
            ob->tracked_blobs = 0;
        }
        */
    }

/*
void PrintBlObservation(blobservation *BlobOb)
{
    if (NULL == BlobOb)
    {
        cout<<"PrintBlObservation invalid input"<<endl;
        return;
    }
    blobservation *ob = BlobOb;
    for (j=0;j<NUM_FRAMES_HISTORY;j++)
    {
        ob = &(BlobTrack->history[j]);
        for (i=0;i<MAX_BLOBS_PER_FRAME;i++)
        {
                ob->blobs[i].Center.x = 0;
                ob->blobs[i].Center.y = 0;
                ob->blobs[i].DiffCenter.x = 0;
                ob->blobs[i].DiffCenter.y = 0;
                ob->blobs[i].rect = Rect(0, 0, 0, 0);
                ob->blobs[i].area = 0;
                ob->blobs[i].last_area = 0;
                ob->blobs[i].age = 0;
                ob->blobs[i].track_index = 0;
                ob->blobs[i].pattern = 0;
                ob->blobs[i].led_id = 0;
                ob->tracked[i] = 0;
        }
        ob->num_blobs = 0;
        ob->tracked_blobs = 0;
    }
}

*/

static void AssociateBlobs(blobservation *CurrOb, blobservation *LastOb)
{
    if ((NULL == CurrOb) || (NULL == LastOb))
    {
        cout<<"AssociateBlobs invalid input blob observation is NULL"<<endl;
        return;
    }


    int i, j;    
    int count =0;
    cout<<"total blobs :"<<CurrOb->num_blobs<<endl;

    for (i = 0; i < CurrOb->num_blobs; i++)
    {
        blob *b2 = &CurrOb->blobs[i];
        cout<<"Curr Ob Center : "<<b2->Center;
        for (j = 0; j < LastOb->num_blobs; j++)
        {
            //cout<<"i : "<<i<< " j : "<<j<<endl;
            blob *b1 = &LastOb->blobs[j];
            float x, y, dx, dy;

            /*cout<<"Last Ob Center : "<<b1->Center<<endl;
            cout<<"Curr Ob Center : "<<b2->Center<<endl;
            cout<<"Last Ob Expected next frame center : "<<b1->ExpCenterInNxtFrm<<endl;*/
            

             /* Estimate b1's next position */
             //x = b1->ExpCenterInNxtFrm.x;
             //y = b1->ExpCenterInNxtFrm.y;

             x = b1->Center.x + b1->DiffCenter.x;
             y = b1->Center.y + b1->DiffCenter.y;

             /* Absolute distance */
             dx = abs(x - b2->Center.x);
             dy = abs(y - b2->Center.y);

             /*cout<<"x : "<<x<<endl;
             cout<<"y : "<<y<<endl;
             cout<<"dx : "<<dx<<endl;
             cout<<"dy : "<<dy<<endl;
             cout<<"b2->rect.width : "<<b2->rect.width<<endl;
             cout<<"b2->rect.height : "<<b2->rect.height<<endl;*/

             /*
              * Check if b1's estimated next position falls
              * into b2's bounding box.
              */

             //if ((2 * dx > 3 * b2->rect.width) || (2 * dy > 3 * b2->rect.height))
             if ((2 * dx > b2->rect.width) || (2 * dy > b2->rect.height))
             {
                 //cout<<"bounding box continuing "<<endl;
                continue;
             }

             /*if ((dx > 5) || (dy > 5))
             {
                //cout<<"     not associated "<<endl;
                continue;
             }*/

             b2->age = b1->age + 1;
             cout<<"    associated to Last Ob Center : "<<b1->Center<<endl;
             cout<<"b2->age :  "<<b2->age<<endl;
                          
             //cout<<"b1->age :  "<<b1->age<<endl;
             count++;
             if ((b1->track_index >= 0) && (CurrOb->tracked[b1->track_index] == 0))
             {
                 /* Only overwrite tracks that are not already set */
                 b2->track_index = b1->track_index;
                 CurrOb->tracked[b2->track_index] = i + 1;
                 /*b2->pattern = b1->pattern;  need to carefully check why need to comment here and add below
                 b2->led_id = b1->led_id;
                 printf("assos b2->pattern: %x\n", b2->pattern);
                 printf("assos b2->led_id: %x\n", b2->led_id);*/
              }

              b2->DiffCenter.x = b2->Center.x - b1->Center.x;
              b2->DiffCenter.y = b2->Center.y - b1->Center.y;
              b2->last_area = b1->area;
              b2->worldpoint = b1->worldpoint;
              b2->pattern = b1->pattern;
              b2->led_id = b1->led_id;
               //cout<<" b2->worldpoint : "<< b2->worldpoint<<endl;
              break;
        }

        /* As the exposure time decreased the blob rectagle size decreses and the displacement/movement of the blob between frames
            increase so the b1's estimated poseition falls outside of rectagle this happens for the first time only
            once we capture the movement we can use tha siffcentre to calculate the estimated position this time
            it falls inside the rectangle
        */

/*
        if ((0 == b2->DiffCenter.x) && (0 == b2->DiffCenter.y))
        {
            / * cal distance between rect centre and estimated position which is near to the center assign that value as initial
            diff center* /
            float dist = INT_MAX, temp;
            float rectX = b2->rect.width/2;
            float rectY = b2->rect.height/2;
            for (int k = 0; k < LastOb->num_blobs; k++)
            {
                 blob *b1 = &LastOb->blobs[k];
                 temp = sqrt(pow(rectX - b1->Center.x, 2) + pow(rectY - b1->Center.y, 2));
                 if (temp < dist)
                 {
                     b2->DiffCenter.x = b2->Center.x - b1->Center.x;
                     b2->DiffCenter.y = b2->Center.y - b1->Center.y;
                 }

            }

        }
*/
    }
    
    cout<<"associated blobs :"<<count<<endl;

  /*
   * Associate newly tracked blobs with a free space in the
   * tracking array.
   */
    for (i = 0; i < CurrOb->num_blobs; i++)
    {
       blob *b2 = &CurrOb->blobs[i];

        if ((b2->age > 0) && (b2->track_index < 0))
        {
            b2->track_index = find_free_track(CurrOb->tracked);

            //cout<<"b2->track_index :  "<<b2->track_index<<endl;
        }
        if (b2->track_index >= 0)
        {
            CurrOb->tracked[b2->track_index] = i + 1;
        }
    }

    /* Check blob <-> tracked array links for consistency */
    for (i = 0; i < CurrOb->num_blobs; i++)
    {
          blob *b = &CurrOb->blobs[i];

          if ((b->track_index >= 0) && (CurrOb->tracked[b->track_index] != i + 1))
          {
                  printf("Inconsistency! %d != %d\n", CurrOb->tracked[b->track_index], i + 1);
          }
    }

    return;
}

/***********************************************************************************************************
 * validate the corrected LED(blob) blinking pattern with ground truth LED pattern values in the table
 * if all the LED correspondences found then it will return true
 * *********************************************************************************************************/
bool  isAllLEDsFound(blob *blobs, int num_blobs)
{
    LEDPatternsTable gPatternTbl;
    blob *b;
    uint32_t CrctdPattern = 0;
    unsigned LEDCount = 0;
    unsigned i;

    if (NULL == blobs)
    {
        cout<<"isAllLEDsFound invalid input"<<endl;
        return false;
    }
    b = blobs;
    /* future work : need to handle if 2 blobs matched with same LED pattern   */
    for (b = blobs; b < blobs + num_blobs; b++)
    {
        CrctdPattern = b->patternCrrctd;
        for (i = 0; i < gPatternTbl.num_patterns; i++)
        {
            if (CrctdPattern == gPatternTbl.patterns[i])
            {
                LEDCount++;
            }
        }
    }
cout<<"LEDCount : "<<LEDCount<<endl;
    if (4 == LEDCount)
    {
        return true;
    }

    return false;
}

void InitCurrOb(blobservation *CurrOb)
{
    if (NULL == CurrOb)
    {
        cout<<"InitCurrOb Invalid input"<<endl;
        return;
    }
    int k;

    CurrOb->num_blobs = 0;
    CurrOb->tracked_blobs = 0;
    //Rect ROI (0, 0, 250, 250);
 /*   cv::Rect initrect (0,0,0,0);

    for (k=0; k<MAX_BLOBS_PER_FRAME;k++)
    {
        CurrOb->tracked[k] = 0;
        CurrOb->blobs[k].Center = cv::Point2f(0, 0);
        CurrOb->blobs[k].DiffCenter = cv::Point2f(0, 0);
        CurrOb->blobs[k].rect = initrect;
        CurrOb->blobs[k].area = 0;
        CurrOb->blobs[k].last_area = 0;
        CurrOb->blobs[k].age = 0;
        CurrOb->blobs[k].track_index = 0;
        CurrOb->blobs[k].pattern = 0;
        CurrOb->blobs[k].patternCrrctd = 0;
        CurrOb->blobs[k].led_id = -1;
        CurrOb->blobs[k].worldpoint = cv::Point3d(-1,-1,-1);

    }

 */
    if (!(CurrOb->blobs.empty()))
    {
        CurrOb->blobs.clear();
    }

    return;
}




/*********************************************************************************************************************
 * checks whether the worl co-ordinates assigned from previous fram to current frame for the corresponding LEDs
 * *******************************************************************************************************************/

bool isWldPntsAssociated(blob *blobs, int num_blobs)
{

    blob *b;
    cv::Point3d v(-1,-1,-1);
    if (NULL == blobs)
    {
        cout<<"isWldPntsAssociated invalid input"<<endl;
        return false;
    }
    b = blobs;
    for (b = blobs; b < blobs + num_blobs; b++)
    {

        cout<<"isWldPntsAssociated b->worldpoint : "<<b->worldpoint<<endl;
        if (b->worldpoint == v)
        {
            cout<<"b->worldpoint"<<b->worldpoint<<endl;
            return false;
        }
    }

    return true;
}

void projectOrientationVectorsOnImage(cv::Mat &image, const std::vector<cv::Point3f> points_to_project,
                                      cv::Mat rotation_vector,
                                      cv::Mat translation_vector)
{

  std::vector<cv::Point2f> projected_points;

  projectPoints(points_to_project, rotation_vector, translation_vector, iphone_camera_matrix, iphone_dist_coeffs, projected_points);

  cout<<"image count :"<<globcount<<endl;
  cout<<"points_to_project :"<<endl<< points_to_project<<endl;
  cout<<"projected_points :"<<endl<< projected_points<<endl;
  cout<<"rotation_vector :"<<endl<< rotation_vector<<endl;
  cout<<"translation_vector :"<<endl<< translation_vector<<endl;

  cv::line(image, projected_points[0], projected_points[1], CV_RGB(255, 0, 0), 2);
  cv::line(image, projected_points[0], projected_points[2], CV_RGB(0, 255, 0), 2);
  cv::line(image, projected_points[0], projected_points[3], CV_RGB(0, 0, 255), 2);
  return;

}

void createVisualizationImage(cv::Mat &image, Eigen::Matrix4d transform,cv::Rect region_of_interest,
                                             std::vector<cv::Point2d> distorted_detection_centers,
                              cv::Mat rotation_matrix,
                              cv::Mat translation_vector, std::vector<uint16_t> ledIDs)
{
  const double orientation_vector_length = 20; //!< Length of the orientation trivectors that will be projected onto the output image

  /*
  Eigen::Matrix4d orientation_vector_points; // Matrix holding the points for the orientation trivector that will be projected onto the output image in the object body frame of reference
  orientation_vector_points.col(0) << 0, 0, 0, 1; //58.0,38.0,25.0
  orientation_vector_points.col(1) << orientation_vector_length, 0, 0, 1;
  orientation_vector_points.col(2) << 0, orientation_vector_length, 0, 1;
  orientation_vector_points.col(3) << 0, 0, orientation_vector_length, 1;
*/
  Eigen::MatrixXd orientation_vector_points(3,4);
  orientation_vector_points <<  0, 20, 0, 0,
                                0, 0, 20, 0,
                                0, 0, 0, 20;
  cout<<"orientation_vector_points : "<<endl<<orientation_vector_points<<endl;
   Eigen::Matrix3d rot_matrix_eigen;
   Eigen::MatrixXd orientation_vector_points_wld(3,4);

  cv2eigen(rotation_matrix, rot_matrix_eigen);

  cout<<"rotation_matrix : "<<endl<<rotation_matrix<<endl;

    cout<<"rot_matrix_eigen : "<<endl<<rot_matrix_eigen<<endl;


  orientation_vector_points_wld = -rot_matrix_eigen * orientation_vector_points;

   cout<<"orientation_vector_points_wld : "<<endl<<orientation_vector_points_wld<<endl;




  std::vector<cv::Point3f> points_to_project;
  points_to_project.resize(4);
  int i;

    cout<<"vis translation_vector "<<endl<<translation_vector<<endl;

/*
  points_to_project[0].x = orientation_vector_points(0, 0);
  points_to_project[0].y = orientation_vector_points(1, 0);
  points_to_project[0].z = orientation_vector_points(2, 0);
  points_to_project[1].x = orientation_vector_points(0, 1);
  points_to_project[1].y = orientation_vector_points(1, 1);
  points_to_project[1].z = orientation_vector_points(2, 1);
  points_to_project[2].x = orientation_vector_points(0, 2);
  points_to_project[2].y = orientation_vector_points(1, 2);
  points_to_project[2].z = orientation_vector_points(2, 2);
  points_to_project[3].x = orientation_vector_points(0, 3);
  points_to_project[3].y = orientation_vector_points(1, 3);
  points_to_project[3].z = orientation_vector_points(2, 3);
*/

    points_to_project[0].x = orientation_vector_points_wld(0, 0);
    points_to_project[0].y = orientation_vector_points_wld(1, 0);
    points_to_project[0].z = orientation_vector_points_wld(2, 0);
    points_to_project[1].x = orientation_vector_points_wld(0, 1);
    points_to_project[1].y = orientation_vector_points_wld(1, 1);
    points_to_project[1].z = orientation_vector_points_wld(2, 1);
    points_to_project[2].x = orientation_vector_points_wld(0, 2);
    points_to_project[2].y = orientation_vector_points_wld(1, 2);
    points_to_project[2].z = orientation_vector_points_wld(2, 2);
    points_to_project[3].x = orientation_vector_points_wld(0, 3);
    points_to_project[3].y = orientation_vector_points_wld(1, 3);
    points_to_project[3].z = orientation_vector_points_wld(2, 3);


  projectOrientationVectorsOnImage(image, points_to_project, rotation_matrix, translation_vector);

  // Draw a circle around the detected LED
  char ledId[20] = {0};
  for (i = 0; i < distorted_detection_centers.size(); i++)
  {
    cout<<"vis ledIDs[" <<i<<"]: "<<ledIDs[i]<<endl;
    cout<<"vis distorted_detection_centers[" <<i<<"]: "<<distorted_detection_centers[i]<<endl;
    cv::circle(image, distorted_detection_centers[i], 10, CV_RGB(255, 0, 0), 2);
    //snprintf (ledId, 20, "L%d", ledIDs[i]);
    //cv::putText(image, ledId, distorted_detection_centers[i], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
      //          cvScalar(200,200,250), 1, CV_AA);

  }

  // Draw region of interest
  //cv::rectangle(image, region_of_interest, CV_RGB(0, 0, 255), 2);

  cv::imshow("Output2", image);
  if (!video2.isOpened())
  {
      video2.open("/home/sathish/sourcecode/LEDPattern_1/output_videos/Output30fps.avi",
                    CV_FOURCC('M','J','P','G'), 30, Size(gframe_width,gframe_height), true);
  }

  if (video2.isOpened())
  {
      video2.write(image);
  }
  if (!video1.isOpened())
  {
      video1.open("/home/sathish/sourcecode/LEDPattern_1/output_videos/Output120fps.avi",
                    CV_FOURCC('M','J','P','G'), 120, Size(gframe_width,gframe_height), true);
  }

  if (video1.isOpened())
  {
      video1.write(image);
  }
  char imname[100] = {0};

  snprintf (imname, 100, "/home/sathish/sourcecode/LEDPattern_1/pose/poseimage%d.jpg",globcount);
  imwrite(imname, image);
  return;
}


void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
  KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
                 /* DYNAMIC MODEL */
  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
       /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
  KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}


// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else
  {
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }

  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;

  return euler;
}

// Converts a given Euler angles to Rotation Matrix
cv::Mat euler2rot(const cv::Mat & euler)
{
  cv::Mat rotationMatrix(3,3,CV_64F);

  double x = euler.at<double>(0);
  double y = euler.at<double>(1);
  double z = euler.at<double>(2);

  // Assuming the angles are in radians.
  double ch = cos(z);
  double sh = sin(z);
  double ca = cos(y);
  double sa = sin(y);
  double cb = cos(x);
  double sb = sin(x);

  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh*sb - ch*sa*cb;
  m02 = ch*sa*sb + sh*cb;
  m10 = sa;
  m11 = ca*cb;
  m12 = -ca*sb;
  m20 = -sh*ca;
  m21 = sh*sa*cb + ch*sb;
  m22 = -sh*sa*sb + ch*cb;

  rotationMatrix.at<double>(0,0) = m00;
  rotationMatrix.at<double>(0,1) = m01;
  rotationMatrix.at<double>(0,2) = m02;
  rotationMatrix.at<double>(1,0) = m10;
  rotationMatrix.at<double>(1,1) = m11;
  rotationMatrix.at<double>(1,2) = m12;
  rotationMatrix.at<double>(2,0) = m20;
  rotationMatrix.at<double>(2,1) = m21;
  rotationMatrix.at<double>(2,2) = m22;

  return rotationMatrix;
}

void fillMeasurements( cv::Mat &measurements,
                   const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
    // Convert rotation matrix to euler angles
    cv::Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);

     //cout<<"rot2euler OK"<<endl;
    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}


void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
                     cv::Mat &translation_estimated, cv::Mat &rotation_estimated)
{
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();

    //cout<<"KF.predict() OK"<<endl;

    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);

    //cout<<"KF.correct OK"<<endl;


    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);
    // Estimated euler angles
    cv::Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);
    // Convert estimated quaternion to rotation matrix
    rotation_estimated = euler2rot(eulers_estimated);

    //cout<<"euler2rot OK"<<endl;
}

void OptimizePose(cv::Mat rotation_matrix, cv::Mat translation_vector, cv::Mat &rotation_estimated, cv::Mat &translation_estimated)
{

    cv::KalmanFilter KF;         // instantiate Kalman Filter

    int nStates = 18;            // the number of states
    int nMeasurements = 6;       // the number of measured states
    int nInputs = 0;             // the number of action control
    double dt = 0.00833333;//0.125;           // time between measurements (1/FPS)

    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function

    //cout<<"initKalmanFilter OK"<<endl;

    cv::Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(Scalar(0));
    // fill the measurements vector
    fillMeasurements(measurements, translation_vector, rotation_matrix);

    //cout<<"fillMeasurements OK"<<endl;


    // update the Kalman filter with good measurements
    updateKalmanFilter( KF, measurements, translation_estimated, rotation_estimated);
     cout<<"updateKalmanFilter OK"<<endl;
}

void EstimatePose(const cv::Mat &image,  blobservation *CurrOb, cv::Rect ROI)
{
    blob *b = NULL;
    int i;

    if (NULL == CurrOb)
    {
        cout<<"EstimatePose invalid input blob observation is NULL"<<endl;
        return;
    }

    cv::Mat inliers;

    cv::Mat rotation_matrix; //translate rotation vector(1X3) returned by solvepnp to rotation matrix(3X3)


    // 2D image points. If you change the image, you need to change vector
    std::vector<cv::Point2d> image_points;
    std::vector<cv::Point3d> world_points;
    std::vector<uint16_t> ledIDs;
    std::vector<cv::Point3d>::iterator it;
    std::vector<cv::Point2d>::iterator itimg;
    std::vector<uint16_t>::iterator itled;
    for (i = 0; i < CurrOb->num_blobs; i++)
    //for (i = 0; i < 4; i++)
    {
        b = &CurrOb->blobs[i];
        if (-1 == b->led_id)
        {
            continue;
        }
        image_points.push_back(b->Center);
        world_points.push_back(b->worldpoint);
        ledIDs.push_back(b->led_id);
    }

/*
    cout<<"CurrOb->num_blobs : "<<CurrOb->num_blobs<<endl;
    for(it=world_points.begin();it!=world_points.end();++it)
        cout<<"world_points"<<*it<<endl;
    for(itimg=image_points.begin();itimg!=image_points.end();++itimg)
        cout<<"image_points"<<*itimg<<endl;
    for(itled=ledIDs.begin();itled!=ledIDs.end();++itled)
        cout<<"ledIDs"<<*itled<<endl;

*/
    //cv::solvePnP(world_points, image_points, iphone_camera_matrix, iphone_dist_coeffs, rotation_vector, translation_vector);


    /*CV_EXPORTS_W bool solvePnP( InputArray objectPoints, InputArray imagePoints,
                                InputArray cameraMatrix, InputArray distCoeffs,
                                OutputArray rvec, OutputArray tvec,
                                bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE );*/

    cv::solvePnP(world_points, image_points, iphone_camera_matrix, iphone_dist_coeffs, rotation_vector, translation_vector, true, SOLVEPNP_ITERATIVE);


    /*cv::solvePnPRansac(list_points3d, list_points2d, A, distCoeffs, rvec, tvec,
                               use_extrinsic_guess, iterationsCount, reprojectionError,
                               confidence, inliers, CV_ITERATIVE);*/

    /*cv::solvePnPRansac(world_points, image_points, iphone_camera_matrix, iphone_dist_coeffs, rotation_vector, translation_vector,
                       false, 500, 2.0, 0.95, inliers, SOLVEPNP_P3P);*/

    cout<<"rotation_vector_measured"<<endl<<rotation_vector<<endl;
    cout<<"translation_vector_measured"<<endl<<translation_vector<<endl;


    cv::Rodrigues(rotation_vector, rotation_matrix);
    cout<<"rotation_matrix_measured"<<endl<<rotation_matrix<<endl;
    // Instantiate estimated translation and rotation
    cv::Mat translation_estimated(3, 1, CV_64F);
    cv::Mat rotation_estimated(3, 3, CV_64F);

    OptimizePose(rotation_matrix, translation_vector, rotation_estimated, translation_estimated);


    vector<Point2d> proj_point2D_reproj;
    projectPoints(world_points, rotation_estimated, translation_estimated, iphone_camera_matrix, iphone_dist_coeffs, proj_point2D_reproj);

    cout<<"image count :"<<globcount<<endl;
    cout<<"reproj world_points :"<<endl<< world_points<<endl;
    cout<<"reproj proj_point2D_reproj :"<<endl<< proj_point2D_reproj<<endl;
    int rerr;
    bool isReProjErr = false;
    unsigned errcnt = 0;
    for(rerr = 0;rerr<image_points.size();rerr++)
    {
        Point2d reperr = image_points[rerr] - proj_point2D_reproj[rerr];
        cout<<"reproj err ["<<rerr<<"] : "<<reperr<<endl;
        if ((abs(reperr.x) > 50) || (abs(reperr.y) > 50))
        {
            errcnt++; 
            
        }

    }

    cout<<"errcnt : "<<errcnt<<endl;

    if (errcnt > 9)
    {
        for (i = 0; i < CurrOb->num_blobs; i++)
        {
            b = &CurrOb->blobs[i];
            b->led_id = -1;
            b->age = 0;
            b->pattern = 0;
            b->color = -1;
        }

        cout<<"return due to rep err"<<endl;

        return;
    }


    //cout<<"OptimizePose OK"<<endl;


    Eigen::Matrix4d transform;  //dummy remove in future
    Mat rgb_image = image.clone();
    //cvtColor(image, rgb_image, CV_GRAY2BGR);

    cv::Mat rot_vec_estimated;

    cv::Mat rot_estmated_wld;
    cv::Mat rot_vec_estmated_wld;
    cv::Mat translation_estmated_wld;
    Eigen::Matrix3d rot_matrix_eigen;
    Eigen::Vector3d trans_eigen;  

    rot_estmated_wld = rotation_matrix.t();


    translation_estmated_wld = -rot_estmated_wld * translation_vector;


    createVisualizationImage(rgb_image, transform, ROI, image_points, rotation_estimated, translation_estimated, ledIDs);
    //createVisualizationImage(rgb_image, transform, ROI, image_points, rot_estmated_wld, translation_estmated_wld.t(), ledIDs);


    return;
}




char FindBlobColor(const cv::Mat &blobim, unsigned count)
{

    cv::Mat blobimHSV;
    cv::Mat RedMask, RedMask1, RedMask2;
    cv::Mat BlueMask;
    unsigned hueRed =0, hueBlue=0;
    cv::imshow("blob_color", blobim);
    cv::cvtColor(blobim, blobimHSV, COLOR_BGR2HSV);
    std::vector<cv::Mat> hsvChannels(3);
    cv::split(blobimHSV, hsvChannels);

    //cout<<"blobim rows : "<<blobim.rows<<endl;
    //cout<<"blobim cols : "<<blobim.cols<<endl;
    //char imname[100] = {0};

    //snprintf (imname, 100, "/home/sathish/sourcecode/LEDPattern_1/blobs/blob%d_%d.jpg",globcount, count);
    //imwrite(imname, blobim);


    cv::inRange(blobimHSV, Scalar(0, 100, 100), Scalar(10, 255, 255), RedMask1);
    cv::inRange(blobimHSV, Scalar(170, 100, 100), Scalar(180, 255, 255), RedMask2);
    RedMask = RedMask1 | RedMask2;
    //cout<<"RedMask : "<<RedMask<<endl;
    hueRed = cv::sum(RedMask)[0];
    //cout<<"hueRed : "<<hueRed<<endl;

/*
    cv::inRange(blobimHSV, Scalar(50, 100, 100), Scalar(70, 255, 255), GreenMask);
    //cout<<"GreenMask : "<<GreenMask<<endl;
    hueGreen = cv::sum(GreenMask)[0];
    //cout<<"hueGreen : "<<hueGreen<<endl;
*/

    cv::inRange(blobimHSV, Scalar(110, 100, 100), Scalar(130, 255, 255), BlueMask);
    //cout<<"BlueMask : "<<BlueMask<<endl;
    hueBlue = cv::sum(BlueMask)[0];
    //cout<<"hueBlue : "<<hueBlue<<endl;

    /*
     * R = 0;
     * B = 1;
     */

    //if ((hueRed > 0) && (hueBlue == 0)) // RED
    if (hueRed >= 2*hueBlue) // RED
    {
        cout<<"RED"<<endl;
        return 0;
    }
    else if (hueBlue >= 2*hueRed) // GREEN
    {
        cout<<"BLUE"<<endl;
        return 1;
    }

    cout<<"INVALID"<<endl;
    return -1;
}

void MotionEstimation(const cv::Mat &image, blobservation *CurrOb, blobservation *LastOb)
{

    std::vector<uchar> features_found;
    std::vector<float> feature_errors;
    std::vector<cv::Point2f> expected_locs;
    char flowim[100] = {0};
    snprintf (flowim, 100, "/home/sathish/sourcecode/LEDPattern_1/flow/flow%d.jpg",(globcount-1));

    if (!LastOb->gray.empty())
    {
        cornerSubPix(LastOb->gray, LastOb->distorted_points, Size( 20, 20 ) ,Size( -1, -1 ), 
                    cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ));

        calcOpticalFlowPyrLK(LastOb->gray, CurrOb->gray, LastOb->distorted_points, expected_locs,
                            features_found, feature_errors, cvSize( 10, 10 ), 5, 
                            cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0);

        cout<<"last ob dis points"<<LastOb->distorted_points.size()<<endl;
        for (int j = 0; j < LastOb->num_blobs; j++)
        {
            blob *b = &LastOb->blobs[j];
            cout<<"last distorted_points : "<<LastOb->distorted_points[j]<<endl;
            Point2f p0 =  b->Center;
            Point2f p1 = Point2f(expected_locs[j].x, expected_locs[j].y);
            cout<<"p0 : "<<p0<<endl;
            cout<<"p1 : "<<p1<<endl;
            b->ExpCenterInNxtFrm = expected_locs[j];
            //line(image, p0, p1,CV_RGB(255, 0, 0));
            //circle(image, Point(distorted_points[yj].x, distorted_points[yj].y), 2, CV_RGB(255, 0, 0), -1);
        } 

        for (int l=0; l<<LastOb->distorted_points.size();l++)
        {
            cout <<"LastOb->distorted_points["<<l<<"] :"<<LastOb->distorted_points[l]<<endl;
        }

        for (int k=0;k< expected_locs.size(); k++)
        {
                cout <<"expectedlocs["<<k<<"] :"<<expected_locs[k]<<endl;
        }

        /*for (int d=0; d < CurrOb->distorted_points.size(); d++)
        {
            char ledx[10] = {0};
            snprintf (ledx, 10, "%u", (unsigned)CurrOb->distorted_points[d].x);
            cv::putText(image, ledx, CurrOb->distorted_points[d], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
                        cvScalar(200,200,250), 1, CV_AA);        
        }*/

    }
    //imwrite(flowim, image);
    //imshow("flow", image);
    return;
}



void FindLEDs(const cv::Mat &image, cv::Rect ROI, blobtrack **BlobTrack, const int &threshold_value, const double &gaussian_sigma,
                       const double &min_blob_area, const double &max_blob_area,
                       const double &max_width_height_distortion, const double &max_circular_distortion,
                       List2DPoints &pixel_positions, std::vector<cv::Point2f> &distorted_detection_centers,
                       const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                        blobservation **BlobOb, int width, int height)
{
    if (NULL == *BlobTrack)
    {
        cout<<"FindLEDs Invalid input"<<endl;
        return;
    }


   // std::cout<<"image channels : "<<image.channels()<<endl;

    int LastObIndx = (*BlobTrack)->last_observation;
    int CurrObIndx = (LastObIndx + 1) % NUM_FRAMES_HISTORY;
    blobservation *CurrOb = &((*BlobTrack)->history[CurrObIndx]);

    /* init current current observer. curr observer is frame level */
    /*chaged blobs arry of struct to vector we will fill the bokbs info once we know the no.of blobs of the curr frame*/
    InitCurrOb(CurrOb);

    // Threshold the image
    cv::Mat bw_image;
    cv::Mat grayimage;
    cv::Mat incolbk = image.clone();
    cv::Mat incolbk1 = image.clone();
    //cv::Mat imc1;

    //image.convertTo(imc1, CV_8UC1);
    //std::cout<<"imc1 channels : "<<imc1.channels()<<endl;
    cvtColor(image, grayimage, CV_BGR2GRAY);

    CurrOb->gray = grayimage.clone();




    //cv::threshold(image, bwImage, threshold_value, 255, cv::THRESH_BINARY);
    cv::threshold(grayimage(ROI), bw_image, threshold_value, 255, cv::THRESH_TOZERO);

    // Gaussian blur the image
    cv::Mat gaussian_image;
    cv::Size ksize; //Gaussian kernel size. If equal to zero, then the kerenl size is computed from the sigma
    ksize.width = 0;
    ksize.height = 0;
    GaussianBlur(bw_image.clone(), gaussian_image, ksize, gaussian_sigma, gaussian_sigma, cv::BORDER_DEFAULT);


    //char imname[100] = {0};
    cout<<"globcount :"<<globcount<<endl;
    //snprintf (imname, 100, "/home/sathish/sourcecode/LEDPattern_1/gaussian_nopts/ingimage%d.jpg",globcount);
    //imwrite(imname, gaussian_image);

    //cv::imshow( "Gaussian", gaussian_image );

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gaussian_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    int numPoints = 0; // Counter for the number of detected LEDs

    // Vector for containing the detected points that will be undistorted later
    std::vector<cv::Point2f> distorted_points;
    vector<double> areas;
    vector<cv::Rect> blobrect;
    vector<char> blobcolor; // R=0, G=0, everything else -1

    // Identify the blobs in the image
    for (unsigned i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]); // Blob area
        cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
        double radius = (rect.width + rect.height) / 4; // Average radius

        cv::Moments mu;
        mu = cv::moments(contours[i], false); // gives 3 degress of moments m00 to m22
        cv::Point2f mc;
        mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);  //center of then blob w.r.t the ROI

        cout<<"FindLEDs contour : "<<i<<"  blob area :"<<area<< "   centre : "<<mc<<endl;


        char ledx[10] = {0};
        //char ledx1[10] = {0};

        snprintf (ledx, 10, "%u", (unsigned)mc.x);
        cv::putText(incolbk1, ledx, mc, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);


        // Look for round shaped blobs of the correct size
        if ((area >= min_blob_area) && (area <= max_blob_area)
           && (std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width))
               <= max_width_height_distortion)
           && (std::abs(1 - (area / (CV_PI * std::pow(rect.width / 2, 2)))) <= max_circular_distortion)
           && (std::abs(1 - (area / (CV_PI * std::pow(rect.height / 2, 2)))) <= max_circular_distortion))
        {
             distorted_points.push_back(mc);
             areas.push_back(area);
             blobrect.push_back(rect);
             //cout<<"blob :"<<i<<endl;
             blobcolor.push_back(FindBlobColor(image(rect), i));
             //snprintf (ledx, 10, "%u", (unsigned)mc.x);
             //cv::putText(incolbk, ledx, mc, FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

             numPoints++;
        }
    }
//globcount++;
//return;

    CurrOb->distorted_points = distorted_points;

    //char imname2[100] = {0};
    char imname3[100] = {0};

    //snprintf (imname2, 100, "/home/sathish/sourcecode/LEDPattern_1/bloblocs/ingimage%d.jpg",globcount);
    snprintf (imname3, 100, "/home/sathish/sourcecode/LEDPattern_1/contourlocs/ingimage%d.jpg",globcount);
    //imwrite(imname2, incolbk);
    imwrite(imname3, incolbk1);
    globcount++;
    //cv::imshow( "Gaussian", gaussian_image );
    if (globcount < 10)
    {
        cout<<"frame skipped"<<endl;
        return;        
    }

    cout<<"FindLEDs num of points : "<<numPoints<<endl;


    CurrOb->num_blobs = numPoints;

   //minimum 4 points should be there for pose estimation, if not there skip this frame
    if (CurrOb->num_blobs < 4)
    {
        return;        
    }

    

    StoreBlobs(CurrOb->num_blobs, CurrOb->distorted_points, areas, blobrect, CurrOb->blobs, blobcolor);

    /* copy current obsever to last observer for the next frame curr ob will become last ob*/
    (*BlobTrack)->last_observation = CurrObIndx;

    /* for first frame LastObIndx = -1 then return from here nd next frame onwards start assosiating the blobs*/
    if (-1 == LastObIndx)
    {
        *BlobOb = NULL;
        return;
    }


    blobservation *LastOb = &((*BlobTrack)->history[LastObIndx]);

    //MotionEstimation(image, CurrOb, LastOb );

#if 0
    Mat flow, cflow;
    if (!LastOb->gray.empty())
    {
        calcOpticalFlowFarneback(LastOb->gray, grayimage, flow, 0.5, 5, 16, 3, 5, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN);
	    for (int y = 0; y < image.rows; y += 16)
		for (int x = 0; x < image.cols; x += 16)
		{
			const Point2f& fxy = flow.at<Point2f>(y, x) * 1.0;

cout<<"x :"<<x<<endl;
cout<<"y :"<<y<<endl;
            cout<<"fxy :"<<fxy<<endl;
            cout<<"fxy.x :"<<fxy.x<<endl;
            cout<<"fxy.y :"<<fxy.y<<endl;
            cout<<"Point(x, y) :"<<Point(x, y)<<endl;
            cout<<"Point(cvRound(x + fxy.x), cvRound(y + fxy.y)) :"<<Point(cvRound(x + fxy.x), cvRound(y + fxy.y))<<endl;
            

			line(image, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
				CV_RGB(0, 255, 0));
			//circle(image, Point(x, y), 2, CV_RGB(255, 0, 0), -1);
		}
        
        //char ledx1[10] = {0};

        
		for (int yj = 0; yj < distorted_points.size(); yj++)
		{
			const Point2f& fxy = flow.at<Point2f>(distorted_points[yj].x, distorted_points[yj].y) * 1.0;
            cout<<"distorted_points : "<<distorted_points[yj]<<endl;
            cout<<"fxy :"<<fxy<<endl;

			line(image, Point(distorted_points[yj].x, distorted_points[yj].y), 
                Point(cvRound(distorted_points[yj].x + fxy.x), cvRound(distorted_points[yj].y + fxy.y)),
				CV_RGB(255, 0, 0));
			//circle(image, Point(distorted_points[yj].x, distorted_points[yj].y), 2, CV_RGB(255, 0, 0), -1);
        }

        
       
        for (int d=0; d < distorted_points.size(); d++)
        {
            char ledx[10] = {0};
            snprintf (ledx, 10, "%u", (unsigned)distorted_points[d].x);
            cv::putText(image, ledx, distorted_points[d], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
                        cvScalar(200,200,250), 1, CV_AA);
            const Point2f& fxy_1 = flow.at<Point2f>(distorted_points[d].x, distorted_points[d].y) * 1.0;
            cout<<"fxy_1 : "<<fxy_1<<endl;
            //line(image, Point(distorted_points[d].x, distorted_points[d].y), 
                    //Point(cvRound(distorted_points[d].x + fxy_1.x), cvRound(distorted_points[d].y + fxy_1.y)),
				//CV_RGB(0, 255, 0));
			//circle(image, Point(distorted_points[d].x, distorted_points[d].y), 2, CV_RGB(255, 0, 0), -1);
        }
    }
    

if (!LastOb->gray.empty())
{
    cornerSubPix(LastOb->gray, LastOb->distorted_points,Size( 20, 20 ) ,Size( -1, -1 ), 
                cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );
    std::vector<uchar> features_found;
    std::vector<float> feature_errors;
    std::vector<cv::Point2f> expected_locs;
    calcOpticalFlowPyrLK(LastOb->gray, grayimage, LastOb->distorted_points, expected_locs,
                         features_found, feature_errors, cvSize( 10, 10 ), 5, 
                         cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0);

    for (int yj = 0; yj < LastOb->distorted_points.size(); yj++)
    {
        cout<<"last distorted_points : "<<LastOb->distorted_points[yj]<<endl;
        Point p0 =  Point(LastOb->distorted_points[yj].x, LastOb->distorted_points[yj].y);
        Point p1 = Point(expected_locs[yj].x, expected_locs[yj].y);
        cout<<"p0 : "<<p0<<endl;
        cout<<"p1 : "<<p1<<endl;

        line(image, p0, p1,CV_RGB(255, 0, 0));
        //circle(image, Point(distorted_points[yj].x, distorted_points[yj].y), 2, CV_RGB(255, 0, 0), -1);
    } 

    for (int d=0; d < distorted_points.size(); d++)
    {
        char ledx[10] = {0};
        snprintf (ledx, 10, "%u", (unsigned)distorted_points[d].x);
        cv::putText(image, ledx, distorted_points[d], FONT_HERSHEY_COMPLEX_SMALL, 0.8,
                    cvScalar(200,200,250), 1, CV_AA);        
    }                                    

}
    imwrite(flowim, image);
    imshow("flow", image);
return;
#endif
    AssociateBlobs(CurrOb, LastOb);

    FindColorPattern(CurrOb);

    //FindBlinkPattern(CurrOb);



    //CorrectPattern(CurrOb);

    if (false == GetLEDInfoFromPattern(CurrOb))
    {
        cout<<"not enough LED info found"<<endl;
        return;  //not enough led info found for pose estimation
    }

    // go for pose estimation

    EstimatePose(image, CurrOb, ROI);
    return;
}




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

    //cout<<"Distrotion coeffs initilized to camera model"<<endl;
    return;
}




    void TrackBlobs(const cv::Mat &image, cv::Rect ROI, int width, int height, blobtrack **BlobTrack, blobservation **BlobOb)
    {
        /*if (NULL == *BlobOb)
        {
            cout<<"TrackBlobs Invalid Input"<<endl;
            return;
        }*/

        BlobCfg GlbBlobCfg;
        Mat Camera_Matrix_K;
        vector<double>  Distortion_Matrix;
        //GetCameraMatrix(Camera_Matrix_K);
        //GetDistortionCoeff(Distortion_Matrix);

        //cout<<"image.rows()"<<image.rows<<endl;
        //cout<<"image.cols()"<<image.cols<<endl;

        List2DPoints pixel_positions;
        std::vector<cv::Point2f> distorted_detection_centers;

        if (NULL == *BlobTrack)
        {
            *BlobTrack = new blobtrack;
            if (NULL == *BlobTrack)
            {
                cout<<"TrackBlobs mem allocation error"<<endl;
                return;
            }
            InitBlobTrack (*BlobTrack, width, height);
            //cout<<"TrackBlobs BlobTrack initialized"<<endl;
        }

        gframe_width = width;
        gframe_height = height;
        double focal_length = image.cols;
        Point2d center = cv::Point2d(image.cols/2,image.rows/2);

        iphone_camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
        iphone_dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion


        //iphone_camera_matrix = (cv::Mat_<double>(3,3) <<839.43920487140315, 0, 240, 0, 839.43920487140315, 424, 0, 0, 1);
        //iphone_dist_coeffs = (cv::Mat_<double>(5,1) <<4.6476561543838640e-02, -2.0580084834071521, 0, 0 ,2.0182662261396342e+01);

         //cout<<"TrackBlobs calling findLEDs"<<endl;
        FindLEDs(image, ROI, BlobTrack, GlbBlobCfg.threshold_value, GlbBlobCfg.gaussian_sigma, GlbBlobCfg.min_blob_area,
                               GlbBlobCfg.max_blob_area, GlbBlobCfg.max_width_height_distortion, GlbBlobCfg.max_circular_distortion,
                               pixel_positions, distorted_detection_centers, Camera_Matrix_K, Distortion_Matrix,
                               BlobOb, width, height);

        //cout<<"return from  findLEDs"<<endl;

        return;



    }

} // namespace end
