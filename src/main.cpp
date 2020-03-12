#include "ledpattern.h"
#include "camera_blob_cfg.cpp"
#include "camera_blob_cfg.h"
#include "blobdetector.h"
#include "opencv2/tracking.hpp"
#include <time.h>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
using namespace std;
using namespace cv;
using namespace LED_pose_estimator;
using namespace Eigen;


LED_pose_estimator::LEDInfo LED_pose_estimator::gLEDsInfo[NUM_OF_LEDS];

/*void initLEDsInfo()
{
    / * R = 0
     * B = 1
     * /

    gLEDsInfo[0].LEDId = 1;
    gLEDsInfo[0].pattern = 0X01E2; //RRRR RRRR RGGR RRRG
    gLEDsInfo[0].worldpoint = cv::Point3d(10.0, 223.0, 65.0);//cv::Point3d(0.0,-111.0, 2.0);//cv::Point3d(13, 5, 4);

    gLEDsInfo[1].LEDId = 2;
    gLEDsInfo[1].pattern = 0X02A3; // RRRR RRGR RRRR RRRG
    gLEDsInfo[1].worldpoint = cv::Point3d(35.0, 200.0, 65.0);//cv::Point3d(34.0,-88.8, 2.0);//cv::Point3d(47, 5, 4);

    gLEDsInfo[2].LEDId = 3;
    gLEDsInfo[2].pattern = 0X0090; // RRRR RRRG GRRR RRRG
    gLEDsInfo[2].worldpoint =  cv::Point3d(50.0, 176.0, 65.0);//cv::Point3d(68.0,-66.6, 2.0);//cv::Point3d(81, 5, 4);

    gLEDsInfo[3].LEDId = 4;
    gLEDsInfo[3].pattern = 0X0159;
    gLEDsInfo[3].worldpoint = cv::Point3d(72.0, 151.0, 65.0);//cv::Point3d(102.0,-44.4, 2.0);//cv::Point3d(115, 5, 4);

    gLEDsInfo[4].LEDId = 5;
    gLEDsInfo[4].pattern = 0X033B;
    gLEDsInfo[4].worldpoint = cv::Point3d(95.0, 121.0, 65.0);//cv::Point3d(115.0,-22.2, 2.0);//cv::Point3d(149, 5, 4);

    gLEDsInfo[5].LEDId = 6;
    gLEDsInfo[5].pattern = 0X0216;
    gLEDsInfo[5].worldpoint = cv::Point3d(123.0, 95.0, 65.0);//cv::Point3d(115.0,22.2, 2.0);//cv::Point3d(183, 5, 4);

    gLEDsInfo[6].LEDId = 7;
    gLEDsInfo[6].pattern = 0X0355;
    gLEDsInfo[6].worldpoint = cv::Point3d(142.0, 75.0, 65.0);//cv::Point3d(102.0,44.4, 2.0);//cv::Point3d(217, 5, 4);

    gLEDsInfo[7].LEDId =8;
    gLEDsInfo[7].pattern = 0X017A;
    gLEDsInfo[7].worldpoint = cv::Point3d(160.0,53.0, 65.0);//cv::Point3d(68.0,66.6, 2.0);//cv::Point3d(251, 5, 4);

    gLEDsInfo[8].LEDId = 9;
    gLEDsInfo[8].pattern = 0X019A;
    gLEDsInfo[8].worldpoint = cv::Point3d(185.0,26.0, 65.0);//cv::Point3d(34.0,88.8, 2.0);//cv::Point3d(285, 5, 4);

    gLEDsInfo[9].LEDId = 10;
    gLEDsInfo[9].pattern = 0X0077;
    gLEDsInfo[9].worldpoint = cv::Point3d(205.0, 5.0, 65.0);//cv::Point3d(0.0,111.0, 2.0);//cv::Point3d(319, 5, 4);

}*/

/*void initLEDsInfo()
{
    / * R = 0
     * B = 1
     * /

    gLEDsInfo[0].LEDId = 1;
    gLEDsInfo[0].pattern = 0X01E2; //RRRR RRRR RGGR RRRG
    gLEDsInfo[0].worldpoint = cv::Point3d(65.0,-92.0, 18.0);//cv::Point3d(0.0,-111.0, 2.0);//cv::Point3d(13, 5, 4);

    gLEDsInfo[1].LEDId = 2;
    gLEDsInfo[1].pattern = 0X02A3; // RRRR RRGR RRRR RRRG
    gLEDsInfo[1].worldpoint = cv::Point3d(100.0,-85.0, 18.0);//cv::Point3d(34.0,-88.8, 2.0);//cv::Point3d(47, 5, 4);

    gLEDsInfo[2].LEDId = 3;
    gLEDsInfo[2].pattern = 0X0090; // RRRR RRRG GRRR RRRG
    gLEDsInfo[2].worldpoint =  cv::Point3d(130.0,-70.0, 18.0);//cv::Point3d(68.0,-66.6, 2.0);//cv::Point3d(81, 5, 4);

    gLEDsInfo[3].LEDId = 4;
    gLEDsInfo[3].pattern = 0X0159;
    gLEDsInfo[3].worldpoint = cv::Point3d(154.0,-45.0, 18.0);//cv::Point3d(102.0,-44.4, 2.0);//cv::Point3d(115, 5, 4);

    gLEDsInfo[4].LEDId = 5;
    gLEDsInfo[4].pattern = 0X033B;
    gLEDsInfo[4].worldpoint = cv::Point3d(163.0,-15.0, 18.0);//cv::Point3d(115.0,-22.2, 2.0);//cv::Point3d(149, 5, 4);

    gLEDsInfo[5].LEDId = 6;
    gLEDsInfo[5].pattern = 0X0216;
    gLEDsInfo[5].worldpoint = cv::Point3d(163.0, 20.0, 18.0);//cv::Point3d(115.0,22.2, 2.0);//cv::Point3d(183, 5, 4);

    gLEDsInfo[6].LEDId = 7;
    gLEDsInfo[6].pattern = 0X0355;
    gLEDsInfo[6].worldpoint = cv::Point3d(154.0, 50.0, 18.0);//cv::Point3d(102.0,44.4, 2.0);//cv::Point3d(217, 5, 4);

    gLEDsInfo[7].LEDId =8;
    gLEDsInfo[7].pattern = 0X017A;
    gLEDsInfo[7].worldpoint = cv::Point3d(130.0,70.0, 18.0);//cv::Point3d(68.0,66.6, 2.0);//cv::Point3d(251, 5, 4);

    gLEDsInfo[8].LEDId = 9;
    gLEDsInfo[8].pattern = 0X019A;
    gLEDsInfo[8].worldpoint = cv::Point3d(95.0,88.0, 18.0);//cv::Point3d(34.0,88.8, 2.0);//cv::Point3d(285, 5, 4);

    gLEDsInfo[9].LEDId = 10;
    gLEDsInfo[9].pattern = 0X0077;
    gLEDsInfo[9].worldpoint = cv::Point3d(61.0, 94.0, 18.0);//cv::Point3d(0.0,111.0, 2.0);//cv::Point3d(319, 5, 4);

}*/

void initLEDsInfo()
{
    /* R = 0
     * B = 1
     */

    gLEDsInfo[0].LEDId = 1;
    gLEDsInfo[0].pattern = 0X01E2; //RRRR RRRR RGGR RRRG
    gLEDsInfo[0].worldpoint = cv::Point3d(132.0, 24.0, 25.0);//cv::Point3d(0.0,-111.0, 2.0);//cv::Point3d(13, 5, 4);

    gLEDsInfo[1].LEDId = 2;
    gLEDsInfo[1].pattern = 0X02A3; // RRRR RRGR RRRR RRRG
    gLEDsInfo[1].worldpoint = cv::Point3d(196.0, 40.0, 25.0);//cv::Point3d(34.0,-88.8, 2.0);//cv::Point3d(47, 5, 4);

    gLEDsInfo[2].LEDId = 3;
    gLEDsInfo[2].pattern = 0X0090; // RRRR RRRG GRRR RRRG
    gLEDsInfo[2].worldpoint =  cv::Point3d(225.0, 76.0, 25.0);//cv::Point3d(68.0,-66.6, 2.0);//cv::Point3d(81, 5, 4);

    gLEDsInfo[3].LEDId = 4;
    gLEDsInfo[3].pattern = 0X0159;
    gLEDsInfo[3].worldpoint = cv::Point3d(162.0, 88.0, 25.0);//cv::Point3d(102.0,-44.4, 2.0);//cv::Point3d(115, 5, 4);

    gLEDsInfo[4].LEDId = 5;
    gLEDsInfo[4].pattern = 0X033B;
    gLEDsInfo[4].worldpoint = cv::Point3d(95.0, 100.0, 25.0);//cv::Point3d(115.0,-22.2, 2.0);//cv::Point3d(149, 5, 4);

    gLEDsInfo[5].LEDId = 6;
    gLEDsInfo[5].pattern = 0X0216;
    gLEDsInfo[5].worldpoint = cv::Point3d(140.0, 143.0, 25.0);//cv::Point3d(115.0,22.2, 2.0);//cv::Point3d(183, 5, 4);

    gLEDsInfo[6].LEDId = 7;
    gLEDsInfo[6].pattern = 0X0355;
    gLEDsInfo[6].worldpoint = cv::Point3d(205.0, 170.0, 25.0);//cv::Point3d(102.0,44.4, 2.0);//cv::Point3d(217, 5, 4);

    gLEDsInfo[7].LEDId =8;
    gLEDsInfo[7].pattern = 0X017A;
    gLEDsInfo[7].worldpoint = cv::Point3d(243.0, 210.0, 25.0);//cv::Point3d(68.0,66.6, 2.0);//cv::Point3d(251, 5, 4);

    gLEDsInfo[8].LEDId = 9;
    gLEDsInfo[8].pattern = 0X019A;
    gLEDsInfo[8].worldpoint = cv::Point3d(182.0, 227.0, 25.0);//cv::Point3d(34.0,88.8, 2.0);//cv::Point3d(285, 5, 4);

    gLEDsInfo[9].LEDId = 10;
    gLEDsInfo[9].pattern = 0X0077;
    gLEDsInfo[9].worldpoint = cv::Point3d(116.0, 248.0, 25.0);//cv::Point3d(0.0,111.0, 2.0);//cv::Point3d(319, 5, 4);

    gLEDsInfo[9].LEDId = 11;
    gLEDsInfo[9].pattern = 0X0197;
    gLEDsInfo[9].worldpoint = cv::Point3d(23.0, 196.0, 25.0);//cv::Point3d(0.0,111.0, 2.0);//cv::Point3d(319, 5, 4);

    gLEDsInfo[9].LEDId = 12;
    gLEDsInfo[9].pattern = 0X00FB;
    gLEDsInfo[9].worldpoint = cv::Point3d(35.0, 132.0, 25.0);//cv::Point3d(0.0,111.0, 2.0);//cv::Point3d(319, 5, 4);

    gLEDsInfo[9].LEDId = 13;
    gLEDsInfo[9].pattern = 0X00A5;
    gLEDsInfo[9].worldpoint = cv::Point3d(52.0, 67.0, 25.0);//cv::Point3d(0.0,111.0, 2.0);//cv::Point3d(319, 5, 4);

}


std::queue<Mat> buffer;
std::mutex mtxCam;
std::atomic<bool> grabOn; //this is lock free


void GrabThread(VideoCapture *cap)
{
    Mat tmp;
    int count = 0;
    cout<<"thread started"<<endl;

    //To know how many memory blocks will be allocated to store frames in the queue.
    //Even if you grab N frames and create N x Mat in the queue
    //only few real memory blocks will be allocated
    //thanks to std::queue and cv::Mat memory recycling
    std::map<unsigned char*, int> matMemoryCounter;
    uchar *frameMemoryAddr;

    (void)system("v4l2-ctl -d /dev/video1 -c exposure_auto=1"); //logitech
    //(void)system("v4l2-ctl -d /dev/video1 -c auto_exposure=1"); // PS3
    //(void)system("v4l2-ctl -d /dev/video1 -c gain_automatic=0"); //PS3
    while (grabOn.load() == true) //this is lock free
    {
        //grab will wait for cam FPS
        //keep grab out of lock so that 
        //idle time can be used by other threads
        //(void)system("v4l2-ctl -d /dev/video1 -c exposure_absolute=20"); //logitech
        //(void)system("v4l2-ctl -d /dev/video1 -c exposure=20");  // PS3
        //(void)system("v4l2-ctl -d /dev/video1 -c gain=2");  // PS3
        *cap >> tmp; //this will wait for cam FPS

        if (tmp.empty()) 
        {
            //cout<<"tnp empty"<<endl;
            continue;
        }
        //cout<<"tmp is not empty"<<endl;
        char imname[100] = {0};
        snprintf (imname, 100, "/home/sathish/sourcecode/LEDPattern_1/input_thread/inimage%d.jpg",
                    count);
        count++;
        imwrite(imname, tmp);

        //get lock only when we have a frame
        mtxCam.lock();
        //buffer.push(tmp) stores item by reference than avoid
        //this will create a new cv::Mat for each grab
        buffer.push(Mat(tmp.size(), tmp.type()));
        tmp.copyTo(buffer.back());
        frameMemoryAddr = buffer.front().data;
        mtxCam.unlock();
        //count how many times this memory block has been used
        matMemoryCounter[frameMemoryAddr]++; 

        /*bool show = true;
        if (show)
        {
            int font = CV_FONT_HERSHEY_PLAIN;
            putText(tmp, "THREAD FRAME", Point(10, 10), font, 1, Scalar(0, 255, 0));
            imshow("Image thread", tmp);
            waitKey(1);    //just for imshow
        }*/
    }
    std::cout << std::endl << "Number of Mat in memory: " << matMemoryCounter.size();
}



int main()
{

        int frame_width, frame_height;
        double time_stamp = 0;

        initLEDsInfo();

        //cout<<"gLEDsInfo[3].LEDId : "<<gLEDsInfo[3].LEDId<<endl;
        //cout<<"gLEDsInfo[3].pattern : "<<gLEDsInfo[3].pattern<<endl;
        //cout<<"gLEDsInfo[3].worldpoint : "<<gLEDsInfo[3].worldpoint<<endl;


        VideoCapture cap(0);
        //VideoCapture cap("./input5bit4LEDs.mp4");
        if (!cap.isOpened())  // check if we succeeded
        {
            cout<<"Could not open the input video."<<endl;
            return -1;
        }

        cap.set(CV_CAP_PROP_FPS, 60);
        int fps = cap.get(CV_CAP_PROP_FPS);
        cout<<"fps :"<<fps<<endl;

        grabOn.store(true); 
        thread t(GrabThread, &cap);
        //t.join();
       

        //cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
        //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);


        blobtrack *BlobTrack = NULL;

        frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
        frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        //double exp_time = cap.get(CV_CAP_PROP_EXPOSURE);
        //cout<<"exp_time :"<<exp_time<<endl;

        cv::Mat image, out_image;

        int no_pose_frames = 0;
        int no_of_frames = 0;


        VideoWriter video;
        video.open("/home/sathish/sourcecode/LEDPattern_1/input_video/input.avi",
                    CV_FOURCC('M','J','P','G'), (cap.get(CV_CAP_PROP_FPS) - 15), 
                    Size(frame_width,frame_height), true);
        int i;
        //(void)system("v4l2-ctl -d /dev/video1 -c auto_exposure=1"); // PS3
        //(void)system("v4l2-ctl -d /dev/video1 -c gain_automatic=0"); //PS3
        //(void)system("v4l2-ctl -d /dev/video1 -c exposure_auto=1"); //logitech
        //(void)system("v4l2-ctl -d /dev/video1 -c exposure=120");  // PS3
        //(void)system("v4l2-ctl -d /dev/video1 -c gain=2");  // PS3
        //(void)system("v4l2-ctl -d /dev/video1 -c exposure_absolute=20"); // logitech
        cv::namedWindow( "input", cv::WINDOW_AUTOSIZE );
        int bufSize;
        int count = 0;
        for (i = 0;;i++)
        {
            //(void)system("v4l2-ctl -d /dev/video1 -c exposure_absolute=20"); // logitech
            //(void)system("v4l2-ctl -d /dev/video1 -c exposure=20");  // PS3
            //(void)system("v4l2-ctl -d /dev/video1 -c gain=2");  // PS3
            mtxCam.lock();                //lock memory for exclusive access
            bufSize = buffer.size();      //check how many frames are waiting         
            if (bufSize > 0)              //if some 
            {
                buffer.front().copyTo(image);   //get the oldest grabbed frame (queue=FIFO)
                buffer.pop();
            }
            mtxCam.unlock();            //unlock the memory

            if (bufSize <= 0)
            {
                continue;
            } 

            video.write(image);

            char imname[100] = {0};

            double tme = cap.get(CV_CAP_PROP_POS_MSEC);
            cout<<"tme :"<<tme<<endl;

            snprintf (imname, 100, "/home/sathish/sourcecode/LEDPattern_1/input/inimage%d.jpg",count);
            imwrite(imname, image);   //process it
               

            cv::Rect ROI = Rect(0, 0, frame_width, frame_height);

            cout<<"Frame :  "<<count<<endl;
            
            cv::imshow( "input", image );

            /* write function to send the blobwatch strct and frame inside the function call FindLEDs to detect the LEDs and areas of blobs
allocacte blob watch for first frame and pass to the blobwatch(BW) function

*/
            blobservation *BlobOb = NULL;
            TrackBlobs(image, ROI, frame_width, frame_height, &BlobTrack, &BlobOb);
            bufSize--;
            
            cout << "frame to process:" << bufSize<<endl; 
            cout<<"returned for frame : "<<count<<endl;
            count++;
            cout<<endl;
            cout<<endl;

            // causes to wait for 33 ms which my be the reason for frame drop
            char key = waitKey(1);  
            if (27 == key)
            {
                grabOn.store(false);    //stop the grab loop 
                t.join();               //wait for the grab loop
                cout << endl << "Flushing buffer of:" << bufSize << " frames...";
                while (!buffer.empty())    //flushing the buffer
                {
                    image = buffer.front();
                    video.write(image);
                    char imname[100] = {0};
                    double tme = cap.get(CV_CAP_PROP_POS_MSEC);
                    cout<<"tme :"<<tme<<endl;
                    snprintf (imname, 100, "/home/sathish/sourcecode/LEDPattern_1/input/inimage%d.jpg",count);
                    imwrite(imname, image);   //process it
                    cv::Rect ROI = Rect(0, 0, frame_width, frame_height);
                    cout<<"Frame :  "<<count<<endl;
                    cv::imshow( "input", image );
                    blobservation *BlobOb = NULL;
                    TrackBlobs(image, ROI, frame_width, frame_height, &BlobTrack, &BlobOb);
                    buffer.pop();
                }
                cout << "done"<<endl;
                break;
            }
            cvDestroyWindow("input");

        }
        /*int zz;
        for (zz=0; zz< BlobOb->num_blobs; zz++)
        {
            cout<<"blob "<<zz<<"  id :"<<BlobOb->blobs[zz].led_id<<endl;
        }*/
        cap.release();
        video.release();
        //destroyAllWindows();
        return 0;
}
