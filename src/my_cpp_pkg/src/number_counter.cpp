    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/msg/int64.hpp"
    #include "example_interfaces/srv/set_bool.hpp"

    using namespace std::chrono_literals;
    using namespace std::placeholders;
     
    class NumberCounterNode : public rclcpp::Node 
    {
    public:
        NumberCounterNode() : Node("number_counter"), counter_(0)
        {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                        std::bind(&NumberCounterNode::CallbackNumber, this, _1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        RCLCPP_INFO(this->get_logger(),"Number counter has been started");
        reset_counter_server_ = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
        std::bind(&NumberCounterNode::set_counter_zero, this,_1,_2));
        RCLCPP_INFO(this->get_logger(), "Set counter Service has been started.");
        
        }
     
    private:
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_counter_server_;
    int counter_;
    void CallbackNumber(example_interfaces::msg::Int64::SharedPtr msg){
        example_interfaces::msg::Int64 ctr;
        counter_ = counter_ + msg->data;
        ctr.data = counter_;
        
        RCLCPP_INFO(get_logger(), "%d", counter_);
        publisher_->publish(ctr);

    }

    void set_counter_zero(example_interfaces::srv::SetBool::Request::SharedPtr request,
    example_interfaces::srv::SetBool::Response::SharedPtr response){
        if(request->data == 0){
            counter_ = 0;
            RCLCPP_INFO(this->get_logger(), "Setting counter to Zero.");
            response->success = 1;
        }
        else{
            response->success = 0;
        }
    }

    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<NumberCounterNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }