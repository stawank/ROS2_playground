    #include "rclcpp/rclcpp.hpp"
    #include "custom_interfaces/msg/hardware_status.hpp"
    using namespace std::chrono_literals;
     
    class HardwareStatusPublisherNode : public rclcpp::Node 
    {
    public:
        HardwareStatusPublisherNode() : Node("hardware_status_publisher") 
        {
            hardware_status_publisher_ = this->create_publisher<custom_interfaces::msg::HardwareStatus>("hardware_status",10);
            timer_ = this->create_wall_timer(1s, std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this));
            RCLCPP_INFO(this->get_logger(),"Hardware status publisher is started..");
        }
     
    private:
        rclcpp::Publisher<custom_interfaces::msg::HardwareStatus>::SharedPtr hardware_status_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        void publishHardwareStatus(){
            auto msg = custom_interfaces::msg::HardwareStatus();
            msg.temperature = 54.3;
            msg.are_motors_ready = true;
            msg.debug_message = "Nothing special";
            hardware_status_publisher_->publish(msg);


        }
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<HardwareStatusPublisherNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }