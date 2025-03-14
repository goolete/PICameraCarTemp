#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include "libs/httplib.h"
#include "libs/json.hpp"
#include <termios.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <pigpio.h>
#include <linux/i2c-dev.h>

// 从设备I2C地址
#define I2C_ADDR 0x5b
// 温度高8位寄存器地址
unsigned char TEMP_HIGH_REG = 0x04;
// 温度低8位寄存器地址
unsigned char TEMP_LOW_REG = 0x05;


// Use the cv and httplib namespaces to simplify code writing
using namespace cv;
using namespace httplib;
using namespace boost::placeholders;

// Global variable to store the latest frame
Mat latest_frame;
// Open the default camera (index 0)
VideoCapture cap(0);
// File descriptor for the serial port
int serialFd;
int file;

// 定义帧长度
const int FRAME_LENGTH = 15;
// 定义帧头
const unsigned char FRAME_HEADER_1 = 0x5a;
const unsigned char FRAME_HEADER_2 = 0x5a;
// 足够大的缓冲区来存储拼接的数据
char buffer_data[FRAME_LENGTH * 2];
// 当前 buffer_data 中有效数据的长度
size_t buffer_data_length = 0;

// New boolean flag to indicate the running status of the car
bool carRunStatus = false;
// Variable to store the previous running status of the car
bool precarRunStatus = false;
// Define GPIO pins
const int PIN_17 = 17;
const int PIN_27 = 27;
// 定义 Trig 和 Echo 引脚的 BCM 编码
const int TRIG_PIN = 19;
const int ECHO_PIN = 26;

// Define the PWM frequency (unit: Hz)
const int PWM_FREQUENCY = 80000;

// Initialize GPIO and PWM
void initPWM() {
    // Set the pins to output mode
    gpioSetMode(PIN_17, PI_OUTPUT);
    gpioSetMode(PIN_27, PI_OUTPUT);

    // Set the PWM frequency
    gpioSetPWMfrequency(PIN_17, PWM_FREQUENCY);
    gpioSetPWMfrequency(PIN_27, PWM_FREQUENCY);

    // Set the PWM range from 0 to 1000
    gpioSetPWMrange(PIN_17, 1000);
    gpioSetPWMrange(PIN_27, 1000);
}

void initmeasureDistance() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio library." << std::endl;
        exit(1);
    }

    // 设置 Trig 引脚为输出模式，Echo 引脚为输入模式
    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
}

// 该函数用于删除 char[] 数组的前 n 个元素
void removeFirstNChars(char arr[], size_t arrSize, size_t n) {
    // 将后面的元素向前移动 n 个位置
    for (size_t i = 0; i < arrSize - n; ++i) {
        arr[i] = arr[i + n];
    }
}

void inittemp() {
    // 打开I2C设备文件
    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) {
        std::cerr << "Failed to open the i2c bus" << std::endl;
    }

    // 设置从设备地址
    if (ioctl(file, I2C_SLAVE, I2C_ADDR) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave" << std::endl;
        close(file);
    }
}

float get_temp() {
    // 读取温度高8位数据
    if (write(file, &TEMP_HIGH_REG, 1) != 1) {
        std::cerr << "Failed to write to the i2c bus" << std::endl;
        close(file);
    }
    unsigned char temp_high;
    if (read(file, &temp_high, 1) != 1) {
        std::cerr << "Failed to read from the i2c bus" << std::endl;
        close(file);
    }

    // 读取温度低8位数据
    if (write(file, &TEMP_LOW_REG, 1) != 1) {
        std::cerr << "Failed to write to the i2c bus" << std::endl;
        close(file);
    }
    unsigned char temp_low;
    if (read(file, &temp_low, 1) != 1) {
        std::cerr << "Failed to read from the i2c bus" << std::endl;
        close(file);
    }

    // 合并高8位和低8位数据
    unsigned short temp_raw = (temp_high << 8) | temp_low;

    // 这里假设温度数据的转换公式，具体根据传感器手册调整
    float temperature = (float)temp_raw / 100.0;

    std::cout << "Temperature: " << temperature << " °C" << std::endl;

    return temperature;
}


void init() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio library." << std::endl;
    }
    initPWM();
    initmeasureDistance();
    inittemp();
}

// Set the PWM duty cycle for a specified pin
void setPWM_DutyCycle(int pin, double dutyCycle) {
    if (dutyCycle < 0.0) {
        dutyCycle = 0.0;
    } else if (dutyCycle > 1.0) {
        dutyCycle = 1.0;
    }

    // Convert the duty cycle from 0 - 1 to 0 - 1000
    int pwmValue = static_cast<int>(dutyCycle * 1000);
    gpioPWM(pin, pwmValue);
}

// Clean up GPIO resources
void cleanup() {
    gpioPWM(PIN_17, 0);
    gpioPWM(PIN_27, 0);
    gpioTerminate();
}

// 测距函数
float measureDistance() {
    // 触发传感器
    gpioWrite(TRIG_PIN, 0);
    usleep(2);
    gpioWrite(TRIG_PIN, 1);
    usleep(10);
    gpioWrite(TRIG_PIN, 0);

    // 等待回声信号开始
    uint32_t startTime;
    while (gpioRead(ECHO_PIN) == 0) {
        startTime = gpioTick();
    }

    // 等待回声信号结束
    uint32_t endTime;
    while (gpioRead(ECHO_PIN) == 1) {
        endTime = gpioTick();
    }

    // 计算回声信号的持续时间
    uint32_t duration = endTime - startTime;

    // 计算距离（单位：厘米）
    float distance = duration * 0.0343 / 2;

    return distance;
}


// Calculate the center of gravity of the line
cv::Point getLineCenter(const cv::Mat &binary) {
    // Calculate the moments of the binary image
    cv::Moments m = cv::moments(binary, true);
    cv::Point center;
    // If the zero - order moment is not zero, calculate the center coordinates
    if (m.m00 != 0) {
        center.x = static_cast<int>(m.m10 / m.m00);
        center.y = static_cast<int>(m.m01 / m.m00);
    } else {
        // If the zero - order moment is zero, set the center coordinates to (0, 0)
        center.x = 0;
        center.y = 0;
    }
    return center;
}

// Generate control commands based on the center point and frame width
int generateControlCommand(const cv::Point &center, int frameWidth) {
    int centerX = center.x;
    int halfWidth = frameWidth / 2;

    // Define PID parameters, which need to be adjusted according to the actual situation
    double Kp = 1.5625;
    double Ki = 0;
    double Kd = 0;

    // Initialize the integral term and the previous error
    static double integral = 0.0;
    static double previousError = 0.0;

    // Calculate the current error, modify the error calculation method so that the error is positive when the target is on the right
    double error = centerX - halfWidth;

    // Check if the error is within the range of 5
    if (std::abs(error) <= 5) {
        // If the error is within the range, reset the integral term to avoid integral accumulation
        integral = 0.0;
        previousError = 0.0;
        return 1500;
    }

    // Update the integral term
    integral += error;

    // Calculate the derivative term
    double derivative = error - previousError;

    // Calculate the PID output
    double pidOutput = Kp * error + Ki * integral + Kd * derivative;

    // Update the previous error
    previousError = error;

    // The basic output value is 1500
    int baseOutput = 1500;

    // The final output
    int finalOutput = baseOutput + static_cast<int>(pidOutput);

    // Limit the output range to 1000 - 2000
    finalOutput = std::max(1000, std::min(2000, finalOutput));

    return finalOutput;
}

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

// Function to make the car run
void car_run() {
    const char *message1 = "{#008P2000T0000!#009P1000T0000!}";
    // Write the message to the serial port
    ssize_t bytesWritten = write(serialFd, message1, strlen(message1));
    if (bytesWritten == -1) {
        std::cerr << "Failed to send message via serial port" << std::endl;
    }
}

// Function to make the car stop
void car_stop() {
    const char *message1 = "{#008P1500T0000!#009P1500T0000!}";
    // Write the message to the serial port
    ssize_t bytesWritten = write(serialFd, message1, strlen(message1));
    if (bytesWritten == -1) {
        std::cerr << "Failed to send message via serial port" << std::endl;
    }
}

// Timer handler function
void timer_handler(const boost::system::error_code & /*e*/,
                   boost::asio::steady_timer *timer) {
    // std::cout << "Timer triggered, current time: " << std::chrono::steady_clock::now().time_since_epoch().count() << std::endl;

    // Check if the running status of the car has changed
    if (carRunStatus != precarRunStatus) {
        if (carRunStatus) {
            // If the car should run, call the car_run function
            car_run();
        } else {
            // If the car should stop, call the car_stop function
            car_stop();
        }
        // Update the previous running status
        precarRunStatus = carRunStatus;
    }

    Mat frame;
    // Read a frame from the camera
    cap.read(frame);

    if (!frame.empty()) {
        // Check the running status of the car
        if (carRunStatus) {
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

            if (!selectedContour.empty()) {
                // Calculate the center of gravity of the line
                cv::Point center = getLineCenter(binary);

                // Generate control commands
                int command = generateControlCommand(center, binary.cols);
                const char *message_template = "{#010P%dT0000!}";
                char message[50];
                // Format the control command into a message
                snprintf(message, sizeof(message), message_template, command);
                // Write the message to the serial port
                ssize_t bytesWritten = write(serialFd, message, strlen(message));
                if (bytesWritten == -1) {
                    std::cerr << "Failed to send message via serial port" << std::endl;
                }
                // std::cout << "Output: " << command << std::endl;
                // std::cout << "Concatenated message: " << message << std::endl;

                // Find the centerline contour points of the trajectory line
                std::vector<cv::Point> centerlinePoints = findCenterlinePoints(binary, selectedContour);

                // Draw the centerline contour on the color image
                cv::Mat colorImageWithCenterline = frame.clone();
                drawCenterlineOnColorImage(colorImageWithCenterline, centerlinePoints);

                // Update the latest frame
                latest_frame = colorImageWithCenterline.clone();
            } else {
                // If no contour is selected, use the original frame
                latest_frame = frame.clone();
            }
        } else {
            // If the car is not running, use the original frame
            latest_frame = frame.clone();
        }
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
    // Open the serial port device
    serialFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd == -1) {
        std::cerr << "Failed to open serial port device" << std::endl;
    }

    // Configure serial port parameters
    struct termios options;
    // Get the current serial port settings
    tcgetattr(serialFd, &options);
    // Set the input and output baud rates to 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    // Enable local connection and receiving data
    options.c_cflag |= (CLOCAL | CREAD);
    // Disable parity checking
    options.c_cflag &= ~PARENB;
    // Use 1 stop bit
    options.c_cflag &= ~CSTOPB;
    // Clear the data bit setting
    options.c_cflag &= ~CSIZE;
    // Set the data bit to 8 bits
    options.c_cflag |= CS8;
    // Apply the new serial port settings
    tcsetattr(serialFd, TCSANOW, &options);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
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

void temp_control() {
    for (double dutyCycle = 0.0; dutyCycle <= 0.88; dutyCycle += 0.01) {
        std::cout << "Setting duty cycle to " << dutyCycle << " for both pins." << std::endl;
        setPWM_DutyCycle(PIN_17, dutyCycle);
        setPWM_DutyCycle(PIN_27, dutyCycle);
        time_sleep(1);
    }

    for (double dutyCycle = 0.8; dutyCycle >= 0.0; dutyCycle -= 0.01) {
        std::cout << "Setting duty cycle to " << dutyCycle << " for both pins." << std::endl;
        setPWM_DutyCycle(PIN_17, dutyCycle);
        setPWM_DutyCycle(PIN_27, dutyCycle);
        time_sleep(1);
    }

    cleanup();
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

void timer1_handler(const boost::system::error_code & /*e*/,
                    boost::asio::steady_timer *timer1) {
    float distance = measureDistance();
    // std::cout << "Distance: " << distance << " cm" << std::endl;

    // Reset the timer to trigger again after 30 milliseconds
    timer1->expires_at(timer1->expiry() + boost::asio::chrono::milliseconds(600));
    // Asynchronously wait for the timer to expire and call the timer_handler function
    timer1->async_wait(boost::bind(timer1_handler,
                                   boost::asio::placeholders::error,
                                   timer1));
}

void get_distance() {
    try {
        // Create an io_context object to handle asynchronous operations
        boost::asio::io_context io_context1;
        // Create a steady_timer object, initially set to trigger after 30 milliseconds
        boost::asio::steady_timer timer1(io_context1, boost::asio::chrono::milliseconds(600));

        // Asynchronously wait for the timer to expire and call the timer_handler function
        timer1.async_wait(boost::bind(timer1_handler,
                                      boost::asio::placeholders::error,
                                      &timer1));
        // Start the event loop to handle asynchronous operations
        io_context1.run();
    } catch (std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

int main(void) {
    init();

    // Start the thread to capture camera frames
    std::thread capture_thread(capture_frames);
    // Detach the thread so that it can run independently
    capture_thread.detach();

    std::thread temp_control_thread(temp_control);
    temp_control_thread.detach();

    std::thread distance_thread(get_distance);
    distance_thread.detach();

    Server svr;

    // Define a route to handle video stream requests
    svr.Get("/video_feed", [](const Request &req, Response &res) {
        res.set_content(generate_frames(), "multipart/x-mixed-replace; boundary=frame");
    });

    // New route to receive the switch status sent from the front - end
    svr.Post("/curtainSwitch", [](const Request &req, Response &res) {
        try {
            // Parse the JSON data from the request body
            auto json = nlohmann::json::parse(req.body);
            // Update the running status of the car
            carRunStatus = json["status"];
            std::cout << "Modified status: " << carRunStatus << std::endl;
            res.set_content("Success", "text/plain");
        } catch (...) {
            res.set_content("Error", "text/plain");
        }
    });

    // Start the server and listen on all network interfaces at port 5000
    svr.listen("0.0.0.0", 5000);

    return 0;
}
