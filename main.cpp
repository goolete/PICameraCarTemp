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

// Find the centerline contour points of the trajectory line
std::vector<cv::Point> findCenterlinePoints(const cv::Mat &binary, const std::vector<cv::Point> &contour) {
    std::vector<cv::Point> centerlinePoints;
    // Calculate the bounding rectangle of the contour
    cv::Rect boundingRect = cv::boundingRect(contour);
    for (int y = boundingRect.y; y < boundingRect.y + boundingRect.height; ++y) {
        // Get a single row of the binary image
        cv::Mat row = binary.row(y);
        // Calculate the moments of the row
        cv::Moments m = cv::moments(row, true);
        if (m.m00 != 0) {
            // If the zero - order moment is not zero, calculate the center x - coordinate of the row
            int centerX = static_cast<int>(m.m10 / m.m00);
            centerlinePoints.emplace_back(centerX, y);
        }
    }
    return centerlinePoints;
}

// Draw the centerline contour on the color image
void drawCenterlineOnColorImage(cv::Mat &colorImage, const std::vector<cv::Point> &centerlinePoints) {
    for (size_t i = 0; i < centerlinePoints.size() - 1; ++i) {
        // Draw a line between adjacent centerline points
        cv::line(colorImage, centerlinePoints[i], centerlinePoints[i + 1], cv::Scalar(0, 255, 0), 2);
    }
}

// Timer handler function
void timer_handler(const boost::system::error_code & /*e*/,
                   boost::asio::steady_timer *timer) {
    std::cout << "Timer triggered, current time: " << std::chrono::steady_clock::now().time_since_epoch().count() << std::endl;

    Mat frame;
    // Read a frame from the camera
    cap.read(frame);

    if (!frame.empty()) {
        cv::Mat gray;
        // Convert the frame to grayscale
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Color filtering to identify black lines
        cv::Mat binary;
        // Threshold the grayscale image to get a binary image
        threshold(gray, binary, 50, 255, THRESH_BINARY_INV);

        // Morphological operations
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // Perform an opening operation to remove small noise
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        // Perform a closing operation to connect broken trajectories
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

        // Contour detection
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        // Find contours in the binary image
        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Select the trajectory
        std::vector<cv::Point> selectedContour;
        for (const auto &contour: contours) {
            double area = cv::contourArea(contour);
            if (area > 100) {
                // Select the contour with an area greater than 100
                selectedContour = contour;
                break;
            }
        }
        // Update the latest frame
        latest_frame = frame.clone();
    } else {
        // If the frame is empty, use the original frame
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
    // Start the thread to capture camera frames
    std::thread capture_thread(capture_frames);
    // Detach the thread so that it can run independently
    capture_thread.detach();

    Server svr;

    // Define a route to handle video stream requests
    svr.Get("/video_feed", [](const Request &req, Response &res) {
        res.set_content(generate_frames(), "multipart/x-mixed-replace; boundary=frame");
    });

    // Start the server and listen on all network interfaces at port 5000
    svr.listen("0.0.0.0", 5000);

    return 0;
}