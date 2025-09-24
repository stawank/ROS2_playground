    #include "rclcpp/rclcpp.hpp"
    #include "custom_interfaces/srv/set_led.hpp"
    #include "custom_interfaces/msg/led_panel_state.hpp"
    #include "example_interfaces/msg/string.hpp"
    
    using namespace std::chrono_literals;
    using namespace std::placeholders;
  
     
    class BatteryNode : public rclcpp::Node 
    {
    public:
        BatteryNode() : Node("battery_client"), battery_state_(1), time_(0)
        {
            client_ = this->create_client<custom_interfaces::srv::SetLed>("set_led");
            timer_ = this->create_wall_timer(1s, std::bind(&BatteryNode::callbackTimerFunction, this));
        }
     
    private:
    int64_t battery_state_;
    rclcpp::Client<custom_interfaces::srv::SetLed>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int time_ ;
    void callbackTimerFunction(){
        if(time_ == 4 && battery_state_ !=0){
            battery_state_ = 0;
            time_ = 0;
            this->callSetLed(2, 1);
        }
        else if(time_ == 6 && battery_state_ != 1){
            battery_state_ = 1;
            time_ = 0;
            this->callSetLed(2, 0);
        }
        
        RCLCPP_INFO(this->get_logger(), "TIME: %d, BATTERY STATE: %ld", time_, battery_state_);
        time_ ++;

    }

    void callSetLed(int64_t led_number, int64_t state)
        {
        while (!client_-> wait_for_service(1s)){
            RCLCPP_WARN(this->get_logger(),"Waiting for the led panel server");
        }
        auto request = std::make_shared<custom_interfaces::srv::SetLed::Request>();
        request-> led_number = led_number;
        request-> state = state;
        client_-> async_send_request(request, std::bind(&BatteryNode::callbackSetLed, this, _1));

        }

        void callbackSetLed(rclcpp::Client<custom_interfaces::srv::SetLed>::SharedFuture future){
            auto response = future.get();
            
        }
   
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<BatteryNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }