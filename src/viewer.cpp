#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "seek.h"
#include "SeekCam.h"
#include <iostream>
#include <string>
#include <signal.h>

using namespace cv;
using namespace LibSeek;

const std::string WINDOW_THERMAL = "SeekThermal";

// Setup sig handling
static volatile sig_atomic_t sigflag = 0;


bool auto_exposure_lock = false;

void handle_sig(int sig) {
    (void)sig;
    sigflag = 1;
}


// Normalize the image so that it uses the full color space available for display. 
void normalize(Mat &inframe) {
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
        normalize(inframe, inframe, 0, 65535, NORM_MINMAX);
        min = -1;
    }
}

// Function to process a raw (corrected) seek frame
void process_frame(Mat &inframe, Mat &outframe, float scale, int colormap, int rotate) {
    Mat frame_g8; // Transient Mat containers for processing

    normalize(inframe);

    // Convert seek CV_16UC1 to CV_8UC1
    inframe.convertTo(frame_g8, CV_8UC1, 1.0/256.0 );

    // Rotate image
    if (rotate == 90) {
        transpose(frame_g8, frame_g8);
        flip(frame_g8, frame_g8, 1);
    } else if (rotate == 180) {
        flip(frame_g8, frame_g8, -1);
    } else if (rotate == 270) {
        transpose(frame_g8, frame_g8);
        flip(frame_g8, frame_g8, 0);
    }

    // Resize image: http://docs.opencv.org/3.2.0/da/d54/group__imgproc__transform.html#ga5bb5a1fea74ea38e1a5445ca803ff121
    // Note this is expensive computationally, only do if option set != 1
    if (scale != 1.0) {
        resize(frame_g8, frame_g8, Size(), scale, scale, INTER_LINEAR);
    }

    // Apply colormap: http://docs.opencv.org/3.2.0/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
    if (colormap != -1) {
        applyColorMap(frame_g8, outframe, colormap);
    } else {
        cv::cvtColor(frame_g8, outframe, cv::COLOR_GRAY2BGR);
    }
}

void key_handler(char scancode) {
    switch (scancode) {
        case 'f': {
            int windowFlag = getWindowProperty(WINDOW_THERMAL, WindowPropertyFlags::WND_PROP_FULLSCREEN) == cv::WINDOW_FULLSCREEN ? cv::WINDOW_NORMAL : cv::WINDOW_FULLSCREEN;
            setWindowProperty(WINDOW_THERMAL, WindowPropertyFlags::WND_PROP_FULLSCREEN, windowFlag);
            break;
        }
        case 's': {
            waitKey(0);
            break;
        }
        case 'a': {
            auto_exposure_lock = !auto_exposure_lock;
            std::cout << "AEL:" << auto_exposure_lock << std::endl;
            break;
        }
        case 'q': {
            sigflag = SIGINT;
        }
    }
}


int main() {

    float scale = 1.0;
    std::string mode = "window";
    std::string camtype = "seekpro";
    int colormap = COLORMAP_JET;
    int rotate = 0;
    std::string output = "";
    // Register signals
    // signal(SIGINT, handle_sig);
    // signal(SIGTERM, handle_sig);

    // Setup seek camera
    LibSeek::SeekThermalPro seekpro;
    LibSeek::SeekCam* seek = &seekpro;

    if (!seek->open()) { std::cout << "Error accessing camera" << std::endl; return 1; }

    // Mat containers for seek frames
    Mat seekframe, outframe;

    // Retrieve a single frame, resize to requested scaling value and then determine size of matrix so we can size the VideoWriter stream correctly
    if (!seek->read(seekframe)) { std::cout << "Failed to read initial frame from camera, exiting" << std::endl; return 1; }

    process_frame(seekframe, outframe, scale, colormap, rotate);

    // Create an output object, if mode specified then setup the pipeline unless mode is set to 'window'
    VideoWriter writer;
    namedWindow(WINDOW_THERMAL, cv::WINDOW_NORMAL);
    setWindowProperty(WINDOW_THERMAL, WindowPropertyFlags::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
    resizeWindow(WINDOW_THERMAL, outframe.cols, outframe.rows);


    // Main loop to retrieve frames from camera and write them out
    while (!sigflag) {

        // If signal for interrupt/termination was received, break out of main loop and exit
        if (!seek->read(seekframe)) { std::cout << "Failed to read frame from camera, exiting" << std::endl; return 1; }

        // Retrieve frame from seek and process
        process_frame(seekframe, outframe, scale, colormap, rotate);

        imshow(WINDOW_THERMAL, outframe);
        char c = waitKey(10);
        key_handler(c);

        // If the window is closed by the user all window properties will return -1 and we should terminate
        if (getWindowProperty(WINDOW_THERMAL, WindowPropertyFlags::WND_PROP_FULLSCREEN) == -1) {
            std::cout << "Window closed, exiting" << std::endl;
            return 0;
        }

    }

    std::cout << "Break signal detected, exiting" << std::endl;
    return 0;
}
