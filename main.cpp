#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "libs/httplib.h"
#include "libs/json.hpp"
#include <termios.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

// Use the cv and httplib namespaces to simplify code writing
using namespace cv;
using namespace httplib;

// Global variable to store the latest frame
Mat latest_frame;
// Open the default camera (index 0)
VideoCapture cap(0);

// Timer handler function
void timer_handler(const boost::system::error_code & /*e*/,
                   boost::asio::steady_timer *timer) {
    std::cout << "Timer triggered, current time: " << std::chrono::steady_clock::now().time_since_epoch().count() << std::endl;

    Mat frame;
    // Read a frame from the camera
    cap.read(frame);

    if (!frame.empty()) {
        latest_frame = frame.clone();
    }

    // Reset the timer to trigger again after 30 milliseconds
    timer->expires_at(timer->expiry() + boost::asio::chrono::milliseconds(30));
    // Asynchronously wait for the timer to expire and call the timer_handler function
    timer->async_wait(boost::bind(timer_handler,
                                  boost::asio::placeholders::error,
                                  timer));
}

// Thread function to capture camera frames
void capture_frames() {

    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return;
    }

    try {
        // Create an io_context object to handle asynchronous operations
        boost::asio::io_context io_context;
        // Create a steady_timer object, initially set to trigger after 30 milliseconds
        boost::asio::steady_timer timer(io_context, boost::asio::chrono::milliseconds(30));

        // Asynchronously wait for the timer to expire and call the timer_handler function
        timer.async_wait(boost::bind(timer_handler,
                                     boost::asio::placeholders::error,
                                     &timer));
        // Start the event loop to handle asynchronous operations
        io_context.run();
    } catch (std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

// Function to generate video streams
std::string generate_frames() {
    if (latest_frame.empty()) {
        return "";
    }
    std::vector<uchar> buffer;
    // Encode the frame as a JPEG image
    imencode(".jpg", latest_frame, buffer);

    std::string frame_str(buffer.begin(), buffer.end());
    std::string boundary = "frame";
    std::stringstream ss;
    // Format the frame data as a multipart response
    ss << "--" << boundary << "\r\n";
    ss << "Content-Type: image/jpeg\r\n\r\n";
    ss << frame_str << "\r\n";
    return ss.str();
}

int main(void) {

    Server svr;

    // Define a route to handle video stream requests
    svr.Get("/video_feed", [](const Request &req, Response &res) {
        res.set_content(generate_frames(), "multipart/x-mixed-replace; boundary=frame");
    });

    // Start the server and listen on all network interfaces at port 5000
    svr.listen("0.0.0.0", 5000);

    return 0;
}