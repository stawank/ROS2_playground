    #include "rclcpp/rclcpp.hpp"
    #include "geometry_msgs/msg/twist.hpp"
    #include "turtlesim/msg/pose.hpp"
    #include "custom_interfaces/msg/turtle_array.hpp"
    #include "custom_interfaces/msg/turtle.hpp"
    #include "custom_interfaces/srv/catch_turtle.hpp"
    #include <cmath>
    

    using namespace std::chrono_literals;
    using namespace std::placeholders;
     
    class TurtleControllerNode : public rclcpp::Node 
    {
    public:
        TurtleControllerNode() : Node("turtle_controller") 
        {
            this->declare_parameter("timer_period", 1.0);
            double timer_period_ = this->get_parameter("timer_period").as_double();
            
            poseSubscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 100, 
            std::bind(&TurtleControllerNode::callbackPose, this,_1));
            aliveTurtlesSubscription_ = this->create_subscription<custom_interfaces::msg::TurtleArray>("alive_turtles", 100, 
            std::bind(&TurtleControllerNode::callbackAliveTurtleList, this,_1));

            timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period_),
            std::bind(&TurtleControllerNode::controlTurtle, this));
            cmdPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 100);
            catch_closest_turtle_first_ =  true;
            
            
            catch_turtle_client_ = this->create_client<custom_interfaces::srv::CatchTurtle>("catch_turtle");
            
            


        }
     
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSubscription_ ;
        rclcpp::Subscription<custom_interfaces::msg::TurtleArray>::SharedPtr aliveTurtlesSubscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPublisher_;
        bool turtlesim_up_;
        bool catch_closest_turtle_first_;
        custom_interfaces::msg::Turtle turtle_to_catch_;
        turtlesim::msg::Pose pose_;

        std::vector<custom_interfaces::msg::Turtle> turtleList_;
        rclcpp::Client<custom_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client_;

        void controlTurtle(){
            
            if (!turtlesim_up_ || turtle_to_catch_.name == "")
            {
                return;
            }
            double dist_x = turtle_to_catch_.x - this->pose_.x;
            double dist_y = turtle_to_catch_.y - this->pose_.y;
            double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);
            auto msg = geometry_msgs::msg::Twist();
            

            if (distance > 0.5)
            {
                // position
                msg.linear.x = 2 * distance;

                // orientation
                double steering_angle = std::atan2(dist_y, dist_x);
                double angle_diff = steering_angle - pose_.theta;
                if (angle_diff > M_PI)
                {
                    angle_diff -= 2 * M_PI;
                }
                else if (angle_diff < -M_PI)
                {
                    angle_diff += 2 * M_PI;
                }
                msg.angular.z = 2 * angle_diff;
            }
            else 
            {
                // target reached!
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                callCatchTurtleService(turtle_to_catch_.name);
                turtle_to_catch_.name = "";
                RCLCPP_INFO(this->get_logger(),"Turtle catched!");
            }


            cmdPublisher_->publish(msg);


        }
    double getDistanceFromCurrentPose(custom_interfaces::msg::Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }
    void callCatchTurtleService(std::string turtle_name)
    {
        while (!catch_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<custom_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;

        catch_turtle_client_->async_send_request(
            request, std::bind(&TurtleControllerNode::callbackCallCatchTurtleService, this, _1));
    }

    void callbackCallCatchTurtleService(rclcpp::Client<custom_interfaces::srv::CatchTurtle>::SharedFuture future)
    {
        auto response = future.get();
        
        if (!response->success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to remove turtle");
        }
    }

        
    void callbackPose(const turtlesim::msg::Pose::SharedPtr robotPos)
    {
        pose_ = *robotPos.get();
        turtlesim_up_ = true;
        

    }

        void callbackAliveTurtleList(const custom_interfaces::msg::TurtleArray::SharedPtr aliveTurtleArray)
        {
            if (!aliveTurtleArray->alive_turtles.empty())
            {
                if (catch_closest_turtle_first_)
                {
                    custom_interfaces::msg::Turtle closest_turtle = aliveTurtleArray->alive_turtles.at(0);
                    double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);

                    for (int i = 1; i < (int)aliveTurtleArray->alive_turtles.size(); i++)
                    {
                        double distance = getDistanceFromCurrentPose(aliveTurtleArray->alive_turtles.at(i));
                        if (distance < closest_turtle_distance)
                        {
                            closest_turtle = aliveTurtleArray->alive_turtles.at(i);
                            closest_turtle_distance = distance;
                        }
                    }

                    turtle_to_catch_ = closest_turtle;
                }
                else
                {
                    turtle_to_catch_ = aliveTurtleArray->alive_turtles.at(0);
                }
            }
        }

        
        
        
        
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<TurtleControllerNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }