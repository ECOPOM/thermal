#include <signal.h>
#include <string>
#include <iostream>
#include <fmt/format.h>
#include <opencv2/opencv.hpp> 

#include <librealsense2/rs.hpp>
#include "seek.h"
#include "SeekCam.h"
// #include <cplate/classic.h>

const std::string WINDOW_THERMAL = "SeekThermal";
const std::string WINDOW_SENSE = "RealSense";

// Setup sig handling
static volatile sig_atomic_t sigflag = 0;
bool auto_exposure_lock = false;

// Normalize the image so that it uses the full color space available for display. 
void normalize(cv::Mat &inframe) {
    static double min =-1, max = -1;
    static float multiplier = -1;

    if (auto_exposure_lock) {
        if (min == -1) {
            cv::minMaxLoc(inframe, &min, &max);
            multiplier = 65535 / (max - min);
        }
        for (int y = 0; y < inframe.rows; y++) {
            for (int x = 0; x < inframe.cols; x++) {
                uint16_t val = inframe.at<uint16_t>(y, x);
                if (val > max) {
                    val = 65535;
                } else if (val < min) {
                    val = 0;
                } else {
                    val = (val - min) * multiplier;
                }
                inframe.at<uint16_t>(y, x) = val;
            }
        }
    } else {
        normalize(inframe, inframe, 0, 65535, cv::NORM_MINMAX);
        min = -1;
    }
}

// Function to process a raw (corrected) seek frame
void process_frame(cv::Mat &inframe, cv::Mat &outframe) {
    cv::Mat frame_g8; // Transient Mat containers for processing
    normalize(inframe);

    // Convert seek CV_16UC1 to CV_8UC1
    inframe.convertTo(frame_g8, CV_8UC1, 1.0/256.0 );
    applyColorMap(frame_g8, outframe, cv::COLORMAP_JET);
}

void key_handler(char scancode) {
    switch (scancode) {
        case 'f': {
            int windowFlag = getWindowProperty(WINDOW_THERMAL, cv::WindowPropertyFlags::WND_PROP_FULLSCREEN) == cv::WINDOW_FULLSCREEN ? cv::WINDOW_NORMAL : cv::WINDOW_FULLSCREEN;
            setWindowProperty(WINDOW_THERMAL, cv::WindowPropertyFlags::WND_PROP_FULLSCREEN, windowFlag);
            break;
        }
        case 's': {
            cv::waitKey(0);
            break;
        }
        case 'a': {
            auto_exposure_lock = !auto_exposure_lock;
            fmt::print("AEL: {}", auto_exposure_lock);
            break;
        }
        case 'q': {
            sigflag = SIGINT;
        }
    }
}



int main(int argc, char * argv[]) try {
    // Setup seek camera
    LibSeek::SeekThermalPro seekpro;
    LibSeek::SeekCam* seek = &seekpro;
    if (!seek->open()) { fmt::print("Error accessing camera"); return 1; }

    // Retrieve a single frame, resize to requested scaling value and then determine size of matrix so we can size the VideoWriter stream correctly
    cv::Mat seekframe, outframe;
    if (!seek->read(seekframe)) { fmt::print( "Failed to read initial frame from camera, exiting" ); return 1; }
    process_frame(seekframe, outframe);





    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    pipe.start();
    namedWindow(WINDOW_SENSE, cv::WINDOW_NORMAL);
    setWindowProperty(WINDOW_SENSE, cv::WindowPropertyFlags::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(WINDOW_SENSE, outframe.cols, outframe.rows);

    // Create an output object, if mode specified then setup the pipeline unless mode is set to 'window'
    cv::VideoWriter writer;
    namedWindow(WINDOW_THERMAL, cv::WINDOW_NORMAL);
    setWindowProperty(WINDOW_THERMAL, cv::WindowPropertyFlags::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(WINDOW_THERMAL, outframe.cols, outframe.rows);


    cv::Rect roi;
    roi.x = 200;
    roi.y = 100;
    roi.width = outframe.cols - 50;
    roi.height = outframe.rows - 30;

    while (!sigflag) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat croped = image(roi);

        // Update the window with new data
        imshow(WINDOW_SENSE, croped);

        // If signal for interrupt/termination was received, break out of main loop and exit
        if (!seek->read(seekframe)) { fmt::print("Failed to read frame from camera, exiting"); return 1; }
        // Retrieve frame from seek and process
        process_frame(seekframe, outframe);

        imshow(WINDOW_THERMAL, outframe);
        key_handler(cv::waitKey(10));
    }

    return EXIT_SUCCESS;
} catch (const rs2::error & e) {
    fmt::print("RealSense error calling {} ( {} ): {}", e.get_failed_function(), e.get_failed_args(), e.what());
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    fmt::print("{}", e.what());
    return EXIT_FAILURE;
}
