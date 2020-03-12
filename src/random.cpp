static void AssociateBlobs(blobservation *CurrOb, blobservation *LastOb)
{
    if ((NULL == CurrOb) || (NULL == LastOb))
    {
        cout<<"AssociateBlobs invalid input blob observation is NULL"<<endl;
        return;
    }

    //cout<<"AssociateBlobs called for frame :"<<globcount<<endl;

    int i, j;

    for (i = 0; i < CurrOb->num_blobs; i++)
    {
        blob *b2 = &CurrOb->blobs[i];

        for (j = 0; j < LastOb->num_blobs; j++)
        {
            //cout<<"i : "<<i<< " j : "<<j<<endl;
            blob *b1 = &LastOb->blobs[j];
            float x, y, dx, dy;

            cout<<"Last Ob Center : "<<b1->Center<<endl;
            cout<<"Curr Ob Center : "<<b2->Center<<endl;
            cout<<"Last Ob diff Center : "<<b1->DiffCenter<<endl;
            cout<<"Curr Ob diff Center : "<<b2->DiffCenter<<endl;

             /* Estimate b1's next position */
             x = b1->Center.x + b1->DiffCenter.x;
             y = b1->Center.y + b1->DiffCenter.y;

             /* Absolute distance */
             dx = abs(x - b2->Center.x);
             dy = abs(y - b2->Center.y);

             cout<<"x : "<<x<<endl;
             cout<<"y : "<<y<<endl;
             cout<<"dx : "<<dx<<endl;
             cout<<"dy : "<<dy<<endl;
             cout<<"b2->rect.width : "<<b2->rect.width<<endl;
             cout<<"b2->rect.height : "<<b2->rect.height<<endl;

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

             b2->age = b1->age + 1;
             cout<<"b2->age :  "<<b2->age<<endl;
             cout<<"b1->age :  "<<b1->age<<endl;
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



int main()
{

        int frame_width, frame_height;
        double time_stamp = 0;

        initLEDsInfo();

        //cout<<"gLEDsInfo[3].LEDId : "<<gLEDsInfo[3].LEDId<<endl;
        //cout<<"gLEDsInfo[3].pattern : "<<gLEDsInfo[3].pattern<<endl;
        //cout<<"gLEDsInfo[3].worldpoint : "<<gLEDsInfo[3].worldpoint<<endl;


        VideoCapture cap(1);
        //VideoCapture cap("./input5bit4LEDs.mp4");
        if (!cap.isOpened())  // check if we succeeded
        {
            cout<<"Could not open the input video."<<endl;
            return -1;
        }

        cap.set(CV_CAP_PROP_FPS, 30);
        int fps = cap.get(CV_CAP_PROP_FPS);
        cout<<"fps :"<<fps<<endl;

        grabOn.store(true); 
        thread t(GrabThread, &cap);
       

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
        for (i = 0;;i++)
        {
            //(void)system("v4l2-ctl -d /dev/video1 -c exposure_absolute=20"); // logitech
            //(void)system("v4l2-ctl -d /dev/video1 -c exposure=20");  // PS3
            //(void)system("v4l2-ctl -d /dev/video1 -c gain=2");  // PS3
            cap >> image;
            double tme = cap.get(CV_CAP_PROP_POS_MSEC);
            cout<<"tme :"<<tme<<endl;
            char imname[100] = {0};
            if (image.empty())
            {
                cout<<"input image from camera is empty"<<endl;
                return -1;
            }
            

             video.write(image);

            cv::Rect ROI = Rect(0, 0, frame_width, frame_height);

            cout<<"Frame :  "<<i<<endl;
            //cout<<"frame_width : "<<frame_width<<endl;
            //cout<<"frame_height : "<<frame_height<<endl;
            snprintf (imname, 100, "/home/sathish/sourcecode/LEDPattern_1/input/inimage%d.jpg",i);
            imwrite(imname, image);

            
            cv::imshow( "input", image );

            /* write function to send the blobwatch strct and frame inside the function call FindLEDs to detect the LEDs and areas of blobs
allocacte blob watch for first frame and pass to the blobwatch(BW) function

*/
            blobservation *BlobOb = NULL;
            TrackBlobs(image, ROI, frame_width, frame_height, &BlobTrack, &BlobOb);
            cout<<"returned for frame : "<<i<<endl;
            cout<<endl;
            cout<<endl;

            // causes to wait for 33 ms which my be the reason for frame drop
            /*char key = waitKey(33);  
            if (27 == key)
            {
                break;
            }*/
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
