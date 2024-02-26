#include<iostream>
#include<chrono>

#include <librealsense2/rs.hpp>
#include<opencv2/core/core.hpp>
#include<System.h>

using namespace std;
using namespace cv;

#define width 640
#define height 480
#define fps 30

bool SLAM_state  = false;

void enable_stream_init(rs2::config cfg)
{
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
}

//按下s结束
void Stop_thread()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            SLAM_state = false;
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


//字典 内参
int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tran path_to_vocabulary path_to_settings " << endl;
        return 1;
    }
    //vector<double> vTimestamps;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    SLAM_state = true;

    //配置realsense
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();

    rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    enable_stream_init(cfg);
    // start stream
    pipe.start(cfg);//指示管道使用所请求的配置启动流
    for( int i = 0; i < 30 ; i ++)
    {
        frames = pipe.wait_for_frames();
        //cv::Mat image( cv::Size(640, 480), CV_8UC3, (void*)frames.get_data(), cv::Mat::AUTO_STEP );
        //cv::imshow("imshow", image );
        //cv::waitKey( 20 );
    }
    //Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    thread stop_thread(Stop_thread);

    while(SLAM_state)
    {
        //cout << "frame" << endl;
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架
        //   Align to color
        rs2::align align_to_color(RS2_STREAM_COLOR);
        frames = align_to_color.process(frames);
        // Get imu data
        //Get_imu_data(frames);

        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame(); //change. hekai 20210916
        //rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        // rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

        // Creating OpenCV Matrix from a color image
        cv::Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cv::Mat depth( Size(width, height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP); // add hekai. 20210916
        double tframe = color_frame.get_timestamp();
        //Mat pic_right(Size(width,height), CV_8UC1, (void*)ir_frame_right.get_data());
        //Mat pic_left(Size(width,height), CV_8UC1, (void*)ir_frame_left.get_data());
        //Mat pic_depth(Size(width,height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        //namedWindow("Display Image", WINDOW_AUTOSIZE );
        //imshow("Display Image", color);


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        if(color.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  tframe << endl;
            return 1;
        }

        //SLAM.TrackMonocular(color,tframe);
        SLAM.TrackRGBD( color, depth, tframe );


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        //TODO:检测跟踪时间
        vTimesTrack.push_back( ttrack );
    }
    //while(SLAM.mpPointCloudMapping->loopbusy || SLAM.mpPointCloudMapping->cloudbusy)
    //{
    //    cout<<"";
    //}

    // Stop all threads
    SLAM.Shutdown();
    cout << "-------" << endl << endl;
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}



























//orb-slam2 with 435,save image not in flow.slow

/*
#include<iostream>
#include<algorithm>
#include<fstream>
#include<sstream>
#include<chrono>
#include<librealsense2/rs.hpp>
#include<opencv2/core/core.hpp>
#include <System.h>
#include <boost/format.hpp>
using namespace std;
#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"
#include <stb_image_write.h>

//void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

double LoadImages( cv::Mat &imgRGB, cv::Mat &imD );

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tran path_to_vocabulary path_to_settings " << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat imRGB, imD;
    const int Max_size=INT_MAX;
    for ( int ni = 0; ni < Max_size; ni++ )
    {
        // load image, depthmap,timestamp
        double tframe = LoadImages( imRGB, imD );
        if( imRGB.empty() )
        {
            cerr << endl << "Failed to load color image" << endl;
            return 1;
        }
        if( imD.empty() )
        {
            cerr << endl <<"Failed to load depth image" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back( ttrack );
    }
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort( vTimesTrack.begin(), vTimesTrack.end() );
    float totaltime = 0;
    for( int ni = 0; ni < Max_size; ni++ )
    {
        totaltime += vTimesTrack[ni];
    }

    cout << "--------" << endl << endl;
    cout << "median tracking time： " << vTimesTrack[Max_size / 2] << endl;
    cout << "mean tracking time: " << totaltime / Max_size << endl;

    // Save camera trajectory



    return 0;
}

double LoadImages( cv::Mat &imRGB, cv::Mat &imD )
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the acutal device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    pipe.start();

    int flag = 0;
    double timestamp = 0.0;
    int ping = 1000;
    for( auto&& frame : pipe.wait_for_frames(ping) )
    {    // We can only save video frames as pngs, so we skip the rest
        if( auto vf = frame.as<rs2::video_frame>() )
        {
            // Use the colorizer to get an rgb image for the depth stream
            //if( vf.is<rs2::depth_frame>()) vf = color_map(frame);
            if( vf.is<rs2::depth_frame>() ) vf = frame.apply_filter(color_map );

            // Write images to disk
            timestamp = static_cast<double>(vf.get_timestamp());
            std::stringstream png_file;
            //png_file << n<<"_"<<vf.get_profile().stream_name() << ".png";
            png_file <<timestamp<<vf.get_profile().stream_name() << ".png";
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                           vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());

            if(flag==0)
            {
                imD = cv::imread(png_file.str().c_str(),CV_LOAD_IMAGE_UNCHANGED);
                flag=1;
            }
            else
            {
                imRGB = cv::imread(png_file.str().c_str(),CV_LOAD_IMAGE_UNCHANGED);
                flag=0;
            }
        }
    }

    return timestamp;
}
*/















