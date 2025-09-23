    #include "rclcpp/rclcpp.hpp"
    #include "custom_interfaces/srv/set_led.hpp"
    #include "custom_interfaces/msg/led_panel_state.hpp"
     #include "example_interfaces/msg/string.hpp"
    using namespace std::placeholders;
     
    class LedPanelNode : public rclcpp::Node 
    {
    public:
        LedPanelNode() : Node("led_panel_node") , led_panel_({"off" ,"off" ,"off"})
        {
            publisher_ = this->create_publisher<custom_interfaces::msg::LedPanelState>("led_panel_state", 10);
            led_server_ = this->create_service<custom_interfaces::srv::SetLed>("set_led",
            std::bind(&LedPanelNode::callbackSetLed, this,_1,_2 ));
            RCLCPP_INFO(this->get_logger(), "Server set_led running");
        }
    

    private:
    static const int nLed_ = 3;
    std::array<std::string, nLed_> led_panel_;
    rclcpp::Service<custom_interfaces::srv::SetLed>::SharedPtr led_server_;
    rclcpp::Publisher<custom_interfaces::msg::LedPanelState>::SharedPtr publisher_;
    void callbackSetLed(const custom_interfaces::srv::SetLed::Request::SharedPtr request_,
    const custom_interfaces::srv::SetLed::Response::SharedPtr response_){
        RCLCPP_INFO(this->get_logger(), "Setting Led %d to state %s", int(request_->led_number), request_->state.c_str());
        if (request_->led_number < 1 || request_->led_number > nLed_) {
        RCLCPP_WARN(this->get_logger(), "Invalid LED number: %d", int(request_->led_number));
        response_->success = false;
        return;
    }
        led_panel_[(request_->led_number)-1] = request_->state;
        response_->success= true;
        auto msg = custom_interfaces::msg::LedPanelState();
        msg.first = led_panel_[0].c_str();
        msg.second = led_panel_[1].c_str();
        msg.third = led_panel_[2].c_str();
        publisher_->publish(msg);

    }
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<LedPanelNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }