    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/msg/string.hpp"
    using namespace std::chrono_literals;
     
    class RobotNewsStationNode : public rclcpp::Node 
    {
    public:
        RobotNewsStationNode() : Node("robot_news_station")
        {
            this->declare_parameter("robot_name","R2D2");
            robot_name_ = this->get_parameter("robot_name").as_string();
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
            timer_ = this->create_wall_timer(0.5s,
                                            std::bind(&RobotNewsStationNode::publishNews, this));
            RCLCPP_INFO(this->get_logger(),"Robot news station has been started");
        
        }
     
    private:
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        void publishNews(){
            auto msg = example_interfaces::msg::String();
            msg.data = std::string("Hi, this is ")+ this->robot_name_+std::string(" from robot news station.");
            publisher_->publish(msg);
        }
        std::string robot_name_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<RobotNewsStationNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }