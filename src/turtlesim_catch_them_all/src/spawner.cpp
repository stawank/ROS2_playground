    #include "rclcpp/rclcpp.hpp"
    #include "turtlesim/srv/spawn.hpp"
    #include "turtlesim/srv/kill.hpp"
    #include "random.hpp"
    #include "custom_interfaces/msg/turtle.hpp"
    #include "custom_interfaces/msg/turtle_array.hpp"
    #include "custom_interfaces/srv/catch_turtle.hpp"

 

    #include "turtlesim/msg/pose.hpp"

    using namespace std::chrono_literals;
    using namespace std::placeholders;

    

    class SpawnerNode : public rclcpp::Node 
    {
    public:
        SpawnerNode() : Node("spawner") 
        {
            spawnClient_ = this->create_client<turtlesim::srv::Spawn>("spawn");
            killClient_ = this->create_client<turtlesim::srv::Kill>("kill");
            timer_ = this->create_wall_timer(5s,std::bind(&SpawnerNode::spawnTurtle, this));
            catchTurtleServer_ = this->create_service<custom_interfaces::srv::CatchTurtle>(
                "catch_turtle", std::bind(&SpawnerNode::callbackCatchTurtle,this, _1,_2));
            
            aliveTurtlesPublisher_ = this->create_publisher<custom_interfaces::msg::TurtleArray>("alive_turtles",100);
            length_ = 0;
            
            
        }
     
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawnClient_;
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killClient_;
        rclcpp::Service<custom_interfaces::srv::CatchTurtle>::SharedPtr catchTurtleServer_;
        rclcpp::Publisher<custom_interfaces::msg::TurtleArray>::SharedPtr aliveTurtlesPublisher_;
        RandomNumberGenerator rng;
        std::vector<custom_interfaces::msg::Turtle> turtleList_;
        std::string turtle_to_remove_;
        int length_;


        void callbackCatchTurtle(const custom_interfaces::srv::CatchTurtle::Request::SharedPtr request_, const custom_interfaces::srv::CatchTurtle::Response::SharedPtr response_){
            auto killRequest_ = std::make_shared<turtlesim::srv::Kill::Request>();
            killRequest_->name = request_->name;
            killClient_->async_send_request(killRequest_, std::bind(&SpawnerNode::callbackKill, this, _1));
            response_->success =  true;
            turtle_to_remove_ = request_->name;

        }
        void callbackKill(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future){
            auto response = future.get();
            for (int i = 0; i < (int) turtleList_.size(); i++){
                if(turtleList_.at(i).name == turtle_to_remove_){
                    turtleList_.erase(turtleList_.begin()+i);
                    auto msg = custom_interfaces::msg::TurtleArray();
                    msg.alive_turtles = turtleList_;
                    aliveTurtlesPublisher_->publish(msg);

                }
            }
            
        }

        void spawnTurtle(){
            while (!spawnClient_-> wait_for_service(1s)){
            RCLCPP_WARN(this->get_logger(),"Waiting for the turtlesim service");
            }
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            
            float x = rng.Generate(0.0, 11.0);
            float y = rng.Generate(0.0, 11.0);
            float theta = rng.Generate(0.0, (2*M_PI));
            std::string name = "turtleSpawned"+ std::to_string(length_);
            request->x = x;
            request->y = y;
            request->theta = theta;
            request->name = name;
            
            custom_interfaces::msg::Turtle turtle;
            turtle.name = name;
            turtle.x = x;
            turtle.y = y;
            turtle.theta = theta;

            turtleList_.push_back(turtle);
            auto msg = custom_interfaces::msg::TurtleArray();
            msg.alive_turtles = turtleList_;
            aliveTurtlesPublisher_->publish(msg);
            
            spawnClient_-> async_send_request(request,std::bind(&SpawnerNode::callbackSpawn, this,_1));
            length_++;

        }

        void callbackSpawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future){
            auto response = future.get();
            
        }
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<SpawnerNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }