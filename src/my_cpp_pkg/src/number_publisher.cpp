    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/msg/int64.hpp"
    using namespace std::chrono_literals;
     
    class NumberPublisherNode : public rclcpp::Node 
    {
    public:
        NumberPublisherNode() : Node("number_publisher")
        {
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
            timer_ = this->create_wall_timer(0.1s,
                                            std::bind(&NumberPublisherNode::publishNumber, this));
            RCLCPP_INFO(this->get_logger(),"Number publisher has been started");
        
        }
     
    private:
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
        void publishNumber(){
            auto number = example_interfaces::msg::Int64();
            number.data = 2;
            publisher_->publish(number);
        }
        
        rclcpp::TimerBase::SharedPtr timer_;
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<NumberPublisherNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }