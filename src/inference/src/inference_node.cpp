#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "/home/mp4d/ros2-final/src/inference/include/inference/xinference.h"
#include <chrono>

class ImageInferenceNode : public rclcpp::Node {
public:
    ImageInferenceNode() : Node("inference_node") {
        // Setting our inference component up
        XInference_Initialize(&xinference, "inference");

        // Starting our integer publisher and image subscriber
        subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "image_classify", 1, std::bind(&ImageInferenceNode::imageCallback, this, std::placeholders::_1));
        publisher_ = create_publisher<std_msgs::msg::Int32>("class_id", 1);
    }

    

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert the ROS image message to an OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        image_raw = cv_ptr->image;
        // Process the image (e.g., perform some OpenCV operations)
        // cv::cvtColor(cv_ptr->image, image_raw, cv::COLOR_BGR2GRAY);
        cv::resize(image_raw, image_raw, cv::Size(40,30));

        // Create an array to hold the pixel values
        uint32_t pixelArray[1200]; // 40x30 = 1200

        // Copy the pixel values from the image to the array
        for (int i = 0; i < 30; i++) {
            for (int j = 0; j < 40; j++) {
                pixelArray[i * 40 + j] = image_raw.at<char>(i, j);
            }
        }
        

        int output = 4;
        auto start_time = std::chrono::high_resolution_clock::now();

        // Send the image to the FPGA for processing
        XInference_Write_in_r_Words(&xinference,0,pixelArray,1200);
        XInference_Start(&xinference);

        // Wait till its done and get the output
        while(!XInference_IsDone(&xinference));

        output = XInference_Get_return(&xinference);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // And then publish the class id
        auto message = std_msgs::msg::Int32();

        message.data = output;

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Seen image and classified as class: %i \n", message.data);
        RCLCPP_INFO(get_logger(), "Time taken by the inference code: %ld microseconds\n", duration);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    sensor_msgs::msg::Image::SharedPtr msg_;
    static XInference xinference;
    cv::Mat image_raw;
};

XInference ImageInferenceNode::xinference;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageInferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}