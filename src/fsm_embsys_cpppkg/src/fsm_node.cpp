#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <functional>

//For using ROS2
#include "rclcpp/rclcpp.hpp"

//CV2
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//Message and service types
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/image.hpp" 
#include "embsys_interfaces/msg/fsm_state.hpp"
#include "embsys_interfaces/msg/motor_pos.hpp"
#include "embsys_interfaces/msg/bolt_print.hpp"
#include "embsys_interfaces/srv/fsm_enable.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

using namespace std::chrono_literals;

class EmbSysFSM : public rclcpp::Node
{
  public:
    explicit EmbSysFSM() : Node("embsys_fsm"), cap(0, cv::CAP_V4L){

      //INIT
      //QOS for debug messages
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      //QOS for motor ctrl
      this->declare_parameter("qos_depth", 10);
      int8_t qos_depth = 0;
      this->get_parameter("qos_depth", qos_depth);
      const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

      publisher_fsm_state = this->create_publisher<embsys_interfaces::msg::FSMState>(
        "/embsys_fsm/current_state", qos);  

      publisher_motor_pos = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
        "/set_position", QOS_RKL10V);

      publisher_cv_align = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_align", 1);

      publisher_classifier = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_classify", 1);

      subscriber_get_motor_pos = this->create_subscription<embsys_interfaces::msg::MotorPos>(
        "/cur_motor_position", qos, std::bind(&EmbSysFSM::updateMotorPositions, this, std::placeholders::_1));

      subscriber_adjust_angle = this->create_subscription<std_msgs::msg::Float32>(
        "/align_angle", 10, std::bind(&EmbSysFSM::handle_adjustAngle, this, std::placeholders::_1));

      subsriber_bolt_label = this->create_subscription<std_msgs::msg::Int32>(
        "/class_id", 1, std::bind(&EmbSysFSM::handle_getLabel, this, std::placeholders::_1));

      enable_service_ = this->create_service<std_srvs::srv::Trigger>(
        "enable_fsm", std::bind(&EmbSysFSM::enable_FSM, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      
      client_get_motor_pos = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>(
        "/get_position");

      publisher_bolt_print = this->create_publisher<embsys_interfaces::msg::BoltPrint>(
        "/found_bolts", 1
      );
      
      motor_1_pos = startup_getMotorPos(1);
      motor_2_pos = startup_getMotorPos(2);

      RCLCPP_INFO(this->get_logger(), "FSM Enable Service is ready!");

      // Initialize the state
      current_state = INIT;
      previous_state = INIT;
      fsm_en = false, state_init = false, wait_motor = false, labelRX = false;
      
      // Create timer to run the FSM
      control_loop_frequency = 60;
      control_loop_timer = this->create_wall_timer(std::chrono::microseconds((int64_t)round(1000000.0 / control_loop_frequency)), std::bind(&EmbSysFSM::fsm_control, this));
    }

  private:
  // ------------------------------------------------
  // Control loop of FSM
    void fsm_control()
    {
    // Publish state
      if (current_state != previous_state)
          RCLCPP_INFO(this->get_logger(), "State: %s -> %s", StateString[previous_state], StateString[current_state]);
      embsys_interfaces::msg::FSMState state_message;
      dynamixel_sdk_custom_interfaces::msg::SetPosition motor_msg;

      state_message.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      state_message.current_state = current_state;
      state_message.previous_state = previous_state;
      publisher_fsm_state->publish(state_message);

    // ------------------
    // State actions and state changes
      switch (current_state){
        case INIT:
          if(fsm_en){                           //If FSM allowed to start
            fsm_en = false;

            state_init = true;
            motor_msg.id = (uint8_t)1;
            motor_msg.position = tool_init;
            publisher_motor_pos->publish(motor_msg);

            motor_msg.id = (uint8_t)2;
            motor_msg.position = arm_init;
            publisher_motor_pos->publish(motor_msg);
          }
          else if(state_init){
            if(!waitForMotor(1000)){
              state_init = false;
              current_state = FIND_BOLT_START;
            }
          }
          previous_state = INIT;
          break;
        
        case FIND_BOLT_START:
          if(previous_state != current_state){  //If first time coming into this state
            //current_state = FIND_BOLT_END;
            //break;
          }

          if(edgeDetect(true)){
            wrk_nut_start = motor_2_pos;
            RCLCPP_INFO(this->get_logger(), "Nut start detected at: %d", wrk_nut_start);
            current_state = FIND_BOLT_END;
            break;
          }
          
          if(!waitForMotor(500)){
            int32_t goal_pos = motor_2_pos + 5;
            if(goal_pos < motor_2_max && goal_pos > motor_2_min){
              motor_msg.id = (uint8_t)2;
              motor_msg.position = goal_pos;
              publisher_motor_pos->publish(motor_msg);
            }
            else{
              RCLCPP_INFO(this->get_logger(), "Motor 2 is at the end!");
            }
          }
          previous_state = FIND_BOLT_START;
          break;

        case FIND_BOLT_END:
          if(previous_state != current_state){  //If first time coming into this state
            //current_state = GOTO_BOLT_MIDDLE;
          }

          if(edgeDetect(false) && motor_2_pos - wrk_nut_start > 40){
            wrk_nut_end = motor_2_pos;
            RCLCPP_INFO(this->get_logger(), "Nut end detected at: %d", wrk_nut_end);
            current_state = GOTO_BOLT_MIDDLE;
            break;
          }

          if(!waitForMotor(500)){
            int32_t goal_pos = motor_2_pos + 5;
            if(goal_pos < motor_2_max && goal_pos > motor_2_min){
              motor_msg.id = (uint8_t)2;
              motor_msg.position = goal_pos;
              publisher_motor_pos->publish(motor_msg);
            }
            else{
              RCLCPP_INFO(this->get_logger(), "Motor 2 is at the end!");
            }
          }
          previous_state = FIND_BOLT_END;
          break;

        case GOTO_BOLT_MIDDLE:
          if(previous_state != current_state){  //If first time coming into this state
            //current_state = GET_BOLT_LABEL;
            wrk_nut_mid = motor_2_pos - ((wrk_nut_end - wrk_nut_start) / 2);
            if(wrk_nut_mid < motor_2_max && wrk_nut_mid > motor_2_min){
              motor_msg.id = (uint8_t)2;
              motor_msg.position = wrk_nut_mid;
              publisher_motor_pos->publish(motor_msg);
              state_init = true;
            }
            else{
              RCLCPP_INFO(this->get_logger(), "Error wrk_nut_mid: %d", wrk_nut_mid);
            }
          }
          else if(state_init){
            if(!waitForMotor(1500)){
              state_init = false;
              current_state = GET_BOLT_LABEL;   //GET_BOLT_LABEL
            }
          }
          previous_state = GOTO_BOLT_MIDDLE;
          break;
      
        case GET_BOLT_LABEL:
          if(previous_state != current_state){  //If first time coming into this state
            cv::Mat gray_msg;
            for(int i = 0; i < 10; i++){
              if(cap.read(frame)){
                cap >> frame;
              }
            }
            cv::cvtColor(frame, gray_msg, cv::COLOR_BGR2GRAY);
            cv::resize(gray_msg, gray_msg, cv::Size(40,30));
            auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_msg).toImageMsg();
            publisher_classifier->publish(*message);
          }

          if(labelRX){
            labelRX = false;
            current_state = GOTO_BOLT_END;
          }
          //Wait for Label and then create bolt_info (Struct)
          previous_state = GET_BOLT_LABEL;
          break;

        case GOTO_BOLT_END:
          if(previous_state != current_state){  //If first time coming into this state
            //current_state = GOTO_HEX_BOLT_MIDDLE;
              if(found_bolts.size() == 4){
                current_state = GOTO_HEX_BOLT_MIDDLE; 
                break;  
              }
              else{
                motor_msg.id = (uint8_t)2;
                motor_msg.position = wrk_nut_end + 30;
                publisher_motor_pos->publish(motor_msg);
                state_init = true;
              }
          }
          else if(state_init){
            if(!waitForMotor(1000)){
              state_init = false;
              current_state = FIND_BOLT_START;
            }
          }

          previous_state = GOTO_BOLT_END;
          break;

        case GOTO_HEX_BOLT_MIDDLE:
          if(previous_state != current_state){  //If first time coming into this state
            int32_t hex_pos;
            for(long unsigned int i = 0; i < found_bolts.size(); i++){
              if(found_bolts[i].bolt == BoltType::HEX){
                hex_pos = found_bolts[i].position;
                break;
              }
            }
            motor_msg.id = (uint8_t)2;
            motor_msg.position = hex_pos;
            publisher_motor_pos->publish(motor_msg);
            state_init = true;
          }
          else if(state_init){
            if(!waitForMotor(1500)){
              state_init = false;
              current_state = COMPARE_ALIGN;
            }
          }

          previous_state = GOTO_HEX_BOLT_MIDDLE;
          break;

        case COMPARE_ALIGN:
          if(previous_state != current_state){  //If first time coming into this state
              for(int i = 0; i < 10; i++){
                if(cap.read(frame)){
                  cap >> frame;
                }
              }
              auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
              publisher_cv_align->publish(*message);
          }
          
          if(angleRX){
            angleRX = false;
            current_state = ROTATE_END_EFFECTOR;
            break;
          }
          previous_state = COMPARE_ALIGN;
          break;

        case ROTATE_END_EFFECTOR:
          if(previous_state != current_state){  //If first time coming into this state
              motor_msg.id = (uint8_t)1;
              float adjust_scaled = adjust_angle * angle_scalar;

              motor_msg.position = motor_1_pos - (int32_t)adjust_scaled;
              RCLCPP_INFO(this->get_logger(), "Adjusting M1 by: %f", adjust_scaled);
              publisher_motor_pos->publish(motor_msg);
          }

          if(!waitForMotor(1000)){   //Wait for motor to finish, then allow state change. (Angle to adjust * 50ms)
            current_state = PROG_END;
          }
          previous_state = ROTATE_END_EFFECTOR;
          break;

        case PROG_END:
          if(previous_state != current_state){  //If first time coming into this state
            RCLCPP_INFO(this->get_logger(), "Done.");
          }

          previous_state = PROG_END;
          break;

        default:
          RCLCPP_INFO(this->get_logger(), "ERROR: Case not found ('%d')", current_state);
          break;
      }
    }

    bool waitForMotor(int timeout_ms = 500){
      bool wait = true;
      auto duration = std::chrono::system_clock::now() - motor_timeout;
      if(wait_motor && std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() > timeout_ms){
        wait = false;
        wait_motor = false;
      }
      else if(wait_motor){
        wait = true;
      }
      else{
        wait_motor = true;
        motor_timeout = std::chrono::system_clock::now();
        wait = true;
      }

      return wait;
    }

    int32_t startup_getMotorPos(int req_id){
      int motor_pos = 0;

      auto motor_req = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
      motor_req->id = (uint8_t)req_id;

      while (!client_get_motor_pos->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto result = client_get_motor_pos->async_send_request(motor_req);
      // Wait for the result. HER GÅR DET GALT!
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor %d position: %ld", req_id, result.get()->position);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /get_position");
      }
      motor_pos = result.get()->position;

      return motor_pos;
    }

    void updateMotorPositions(const embsys_interfaces::msg::MotorPos::SharedPtr msg){
        if(msg->motor_1 < 1024){
          motor_1_pos = (int32_t)msg->motor_1;
        }
        if(msg->motor_2 < 1024){
          motor_2_pos = (int32_t)msg->motor_2;
        }
    }
    
    void handle_adjustAngle(const std_msgs::msg::Float32::SharedPtr msg){
      angleRX = true;
      adjust_angle = msg->data;
      RCLCPP_INFO(this->get_logger(), "Adjust angle data received: %f", adjust_angle);
    }

    void handle_getLabel(const std_msgs::msg::Int32::SharedPtr msg){
      labelRX = true;
      new_label = msg->data;
      BoltType bolt_type = static_cast<BoltType>(new_label);
      std::string bolt_name = BoltTypeString[bolt_type];
      int32_t bolt_position = motor_2_pos;
      bolt_info new_bolt = {bolt_type, bolt_name, bolt_position};
    
      found_bolts.push_back(new_bolt);
      RCLCPP_INFO(this->get_logger(), "Bolt/Nut is a: %s", BoltTypeString[bolt_type]);

      embsys_interfaces::msg::BoltPrint foundBolt;
      std_msgs::msg::String bolt;
      bolt.data = bolt_name;
      foundBolt.bolt_type = bolt;
      foundBolt.position = bolt_position;
      publisher_bolt_print->publish(foundBolt);
    }

    void enable_FSM(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request_header;
      (void)request;
      //Prepare the response:
      response->success = false;
      response->message = "";
      
      if(current_state == INIT){
        this->fsm_en = true;
        RCLCPP_INFO(this->get_logger(), "FSM has been enabled!");
        response->message = "FSM has been enabled!";
        response->success = true;
      }
      else if(current_state != INIT && current_state != PROG_END){
        this->current_state = PROG_END;
        RCLCPP_INFO(this->get_logger(), "FSM has been terminated!");
        response->message = "FSM has been terminated!";
        response->success = true;
      }
      else if(current_state == PROG_END){
        this->current_state = INIT;
        RCLCPP_INFO(this->get_logger(), "FSM has been reset!");
        response->message = "FSM has been reset!";
        response->success = true;
      }
    }

    bool edgeDetect(bool findWhite = true){
      bool edge_detected = false;

      for(int i = 0; i < 3; i++){
        if(cap.read(frame)){
          cap >> frame;
        }
      }
      cv::Mat grayscale;
      cv::cvtColor(frame, grayscale, cv::COLOR_BGR2GRAY);

      if (!grayscale.empty()){
        uchar lightvalue = grayscale.at<uchar>(frame.rows / 2, frame.cols / 2);
        //RCLCPP_INFO(this->get_logger(), "Lightvalue: %d", lightvalue);
        if(findWhite && lightvalue > whiteThreshold && !lastFrameWhite){
          edge_detected = true;
        }
        else if(!findWhite && lightvalue < whiteThreshold && lastFrameWhite){
          edge_detected = true;
        }
        else if(findWhite && lightvalue < whiteThreshold){
          lastFrameWhite = false;
        }
        else if(!findWhite && lightvalue > whiteThreshold){
          lastFrameWhite = true;
        }
      }

      return edge_detected;
    }

    cv::Mat getMostRecentFrame(cv::VideoCapture& cap) {
      cv::Mat frame;

      RCLCPP_INFO(this->get_logger(), "Entered 'getMostRecentFrame()'");

      if (!cap.isOpened()) {
          RCLCPP_ERROR(this->get_logger(), "Camera is not opened");
          return frame;
      }

      // Clear the buffer by reading and discarding frames
      while (cap.read(frame)) {
          // Optionally, do a quick check to see if the frame is valid
          if (frame.empty()) {
              RCLCPP_ERROR(this->get_logger(), "Captured frame is empty");
              break;
          }
      }
        // The frame is now the most recent one
        return frame;
    }

      // ------------------------------------------------
    // State machine / Control loop

    // Control loop configuration
    const char* StateString[14] = {
        "INIT",
        "FIND_BOLT_START",
        "FIND_BOLT_END",
        "GOTO_BOLT_MIDDLE",
        "GET_BOLT_LABEL",
        "GOTO_BOLT_END",
        "GOTO_HEX_BOLT_MIDDLE",
        "COMPARE_ALIGN",
        "ROTATE_END_EFFECTOR",
        "PROG_END"
    };
    enum State{
        INIT,
        FIND_BOLT_START,
        FIND_BOLT_END,
        GOTO_BOLT_MIDDLE,
        GET_BOLT_LABEL,
        GOTO_BOLT_END,
        GOTO_HEX_BOLT_MIDDLE,
        COMPARE_ALIGN,
        ROTATE_END_EFFECTOR,
        PROG_END
    };

    //CV Variables:
    cv::VideoCapture cap;
    cv::Mat frame;
    bool angleRX;
    float adjust_angle;
    bool lastFrameWhite;
    const int whiteThreshold = 230;
    int32_t wrk_nut_start, wrk_nut_end, wrk_nut_mid;

    //Motor Variables:
    const int32_t motor_2_min = 450;
    const int32_t motor_2_max = 850;
    const int32_t tool_init = 300;
    const int32_t arm_init = motor_2_min;
    const float angle_scalar = (1024 / 360);
    int32_t motor_1_pos, motor_2_pos;
    
    //State Variables:
    State current_state;
    State previous_state;
    bool fsm_en, wait_motor, state_init;
    double control_loop_frequency;
	  rclcpp::TimerBase::SharedPtr control_loop_timer;
    std::chrono::time_point<std::chrono::system_clock> motor_timeout;
    dynamixel_sdk_custom_interfaces::srv::GetPosition::Request motor_req;

    //Result Memory:
    int32_t new_label;
    bool labelRX;
    enum BoltType{ALLEN, FREARSON, HEX, SLOTTED};
    const char* BoltTypeString[4] = {"ALLEN", "FREARSON", "HEX", "SLOTTED"};
    struct bolt_info{
      BoltType bolt;
      std::string bolt_name;
      int32_t position;};
    std::vector<bolt_info> found_bolts;

    // Publishers, subscribers, services and clients:
    rclcpp::Publisher<embsys_interfaces::msg::FSMState>::SharedPtr publisher_fsm_state;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_motor_pos;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_cv_align;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr client_get_motor_pos;
    rclcpp::Subscription<embsys_interfaces::msg::MotorPos>::SharedPtr subscriber_get_motor_pos;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_adjust_angle;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_classifier;
    rclcpp::Publisher<embsys_interfaces::msg::BoltPrint>::SharedPtr publisher_bolt_print;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subsriber_bolt_label;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating FSM");
  rclcpp::spin(std::make_shared<EmbSysFSM>());
  rclcpp::shutdown();
  return 0;
}
