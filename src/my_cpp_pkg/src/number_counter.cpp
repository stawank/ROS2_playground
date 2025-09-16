    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/msg/int64.hpp"

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
        
        }
     
    private:
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    int counter_;
    void CallbackNumber(example_interfaces::msg::Int64::SharedPtr msg){
        example_interfaces::msg::Int64 ctr;
        counter_++;
        ctr.data = counter_;
        
        RCLCPP_INFO(get_logger(), "%d", counter_);
        publisher_->publish(ctr);

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