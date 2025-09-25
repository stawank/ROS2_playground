    #include "rclcpp/rclcpp.hpp"
    #include "custom_interfaces/srv/set_led.hpp"
    #include "custom_interfaces/msg/led_panel_state.hpp"

    using namespace std::placeholders;
    using namespace std::chrono_literals;
     
    class LedPanelNode : public rclcpp::Node 
    {
    public:
        LedPanelNode() : Node("led_panel_node") 
        {
            this->declare_parameter<std::vector<int>>("led_states",{1,1,0});
            led_states_ = this->get_parameter("led_states").as_integer_array();

            publisher_ = this->create_publisher<custom_interfaces::msg::LedPanelState>("led_panel_state", 10);
            
            server_ = this->create_service<custom_interfaces::srv::SetLed>("set_led",
            std::bind(&LedPanelNode::callback_set_led, this,_1, _2));
            RCLCPP_INFO(this->get_logger(), "LED Panel node has been started");
        }
    

    private:

    
    std::vector<int64_t> led_states_;
    rclcpp::Publisher<custom_interfaces::msg::LedPanelState>::SharedPtr publisher_;
    rclcpp::Service<custom_interfaces::srv::SetLed>::SharedPtr server_;
 
    void callback_set_led(const custom_interfaces::srv::SetLed::Request::SharedPtr request, const custom_interfaces::srv::SetLed::Response::SharedPtr response){
        if(request->led_number>2||request->led_number<0){
           RCLCPP_WARN(this->get_logger(), "Invalid request number"); 
           response->success =  false;
        }
        if(request->state !=0 && request->state !=1){
            RCLCPP_WARN(this->get_logger(), "Invalid request state"); 
           response->success =  false;
        }
        else{
        this->led_states_[request->led_number]= request->state;
        response->success = true;
        this->publish_led_states();
        }
        

    }
    void publish_led_states(){
        auto msg = custom_interfaces::msg::LedPanelState();
        msg.led_states = this->led_states_;
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