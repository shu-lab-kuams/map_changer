#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <string>
#include <vector>
#include <chrono>
#include <nav_msgs/msg/path.hpp>


class MapChangeNode : public rclcpp::Node {
public:
    MapChangeNode() : Node("map_change_node"){

        declare_parameter("map_path", std::string("/path/to/your/csv"));
        declare_parameter("pose_path", std::string("/path/to/your/csv"));
        declare_parameter("start_number", start_number_);

        get_parameter("map_path", map_path_);
        get_parameter("pose_path", pose_path_);
        get_parameter("start_number", start_number_);

        RCLCPP_INFO(get_logger(),"Map_path: %s ", map_path_.c_str());
        RCLCPP_INFO(get_logger(),"Pose_path: %s ", pose_path_.c_str());
        RCLCPP_INFO(get_logger(),"Start_number: %d",start_number_ );

        //pose_number and map_number
        pose_number_ = start_number_;
        map_number_ = start_number_;

        LoadInitialPose(pose_path_, pose_number_);
        LoadMapInfo(map_path_, map_number_);


        // 指定されたyamlファイルを使用するため、初期化時にサービスリクエストを送信
        map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

        //Publisher
        second_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/initial_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
            
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", rclcpp::SystemDefaultsQoS());

        //Subscriber
        waypoint_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "waypoint_id", rclcpp::QoS(10),
            std::bind(&MapChangeNode::WaypointCallback, this, std::placeholders::_1));        

    };

private:
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> LoadInitialPose(std::string pose_path, int pose_number){
                std::ifstream pose_file(pose_path, std::ios::in);

                if (!pose_file.is_open()) {
                    throw std::runtime_error("Failed to open pose file: " + pose_path);
                }

                std::string line;
                int currentLine = 0;
        
                while (std::getline(pose_file, line)) {
                    if (++currentLine == pose_number){
                        std::stringstream ss(line);
                        std::string token;
                        std::vector<std::string> tokens;
                        
                        while (std::getline(ss, token, ',')) {
                            tokens.push_back(token);
                        }

                        if (tokens.size() >= 7) {
                        initial_pose.pose.pose.position.x = std::stod(tokens[0]);
                        initial_pose.pose.pose.position.y = std::stod(tokens[1]);
                        initial_pose.pose.pose.position.z = std::stod(tokens[2]);
                        initial_pose.pose.pose.orientation.x = std::stod(tokens[3]);
                        initial_pose.pose.pose.orientation.y = std::stod(tokens[4]);
                        initial_pose.pose.pose.orientation.z = std::stod(tokens[5]);
                        initial_pose.pose.pose.orientation.w = std::stod(tokens[6]);

                    //Check the log
                    // RCLCPP_INFO(get_logger(),"Initial_Pose_x: %lf ", initial_pose.pose.pose.position.x);
                    // RCLCPP_INFO(get_logger(),"Initial_Pose_y: %lf ", initial_pose.pose.pose.position.y);
                    // RCLCPP_INFO(get_logger(),"Initial_Pose_z: %lf ", initial_pose.pose.pose.position.z);
                    // RCLCPP_INFO(get_logger(),"Initial_Pose_qx: %lf ", initial_pose.pose.pose.orientation.x);
                    // RCLCPP_INFO(get_logger(),"Initial_Pose_qy: %lf ", initial_pose.pose.pose.orientation.y);
                    // RCLCPP_INFO(get_logger(),"Initial_Pose_qz: %lf ", initial_pose.pose.pose.orientation.z);
                    // RCLCPP_INFO(get_logger(),"Initial_Pose_qw: %lf ", initial_pose.pose.pose.orientation.w);

                        pose_list.push_back(initial_pose);
                        }

                        break;
                    }
                }

                pose_file.close();

                return pose_list;

            }

    std::vector<std_msgs::msg::Int32> LoadMapInfo(std::string map_path, int map_number){
            std::ifstream map_file(map_path, std::ios::in);

            if (!map_file.is_open()) {
                throw std::runtime_error("Failed to open map file: " + map_path);
            }

            std::string line;
            int currentLine = 0;
                
            while (std::getline(map_file, line)) {
                if (++currentLine == map_number){
                    std::stringstream ss(line);
                    std::string token;
                    std::vector<std::string> tokens;
                        
                    while (std::getline(ss, token, ',')) {
                        tokens.push_back(token);
                    }

                    if (tokens.size() >= 4) {
                    change_point_id_ = std::stoi(tokens[0]);
                    pcd_file_path_ = tokens[1];
                    map_file_path_ = tokens[2];
                    wait_time_ = std::stoi(tokens[3]);

                    //Check the log
                    // RCLCPP_INFO(get_logger(),"Change_Point_ID: %d ", change_point_id_);
                    // RCLCPP_INFO(get_logger(),"PCD_path: %s ", pcd_file_path_.c_str());
                    // RCLCPP_INFO(get_logger(),"Map_path: %s ", map_file_path_.c_str());
                    // RCLCPP_INFO(get_logger(),"Wait_Time: %d Seconds ", wait_time_);

                    std_msgs::msg::Int32 id_msg;
                    id_msg.data = change_point_id_;
                    map_list.push_back(id_msg);
                    }

                    break;
                }
            }

                map_file.close();

                return map_list;
            
        }


    void SwitchMap(std::string map_file_path_){
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = map_file_path_;

        // サービスが利用可能になるまで待機
        if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Map server not available");
            return;
        }

        // サービスを非同期で呼び出し
        auto future = map_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Switching map completed");

    }

    void SwitchPCD(std::string pcd_file_path_) {

        // PCDファイルの読み込み
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud) == -1) {
           RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file");
           return;
        }

        second_cloud_ = cloud;

        //Publish PCD
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*second_cloud_, output_msg);
        output_msg.header.frame_id = "map";  // 適切なフレームIDを指定
        second_map_pub_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Switching PCD completed");


    }

    void SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){

        //Publish InitialPose
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_covariance;
        initial_pose_with_covariance = *msg;
        initial_pose_pub_->publish(initial_pose_with_covariance);
        RCLCPP_INFO(this->get_logger(), " InitialPose Published");

    }


    void WaypointCallback(const std_msgs::msg::Int32::SharedPtr msg){

        int waypoint_id = msg->data;
        RCLCPP_INFO(get_logger(),"Waypoint ID: %d", msg->data);

        if(waypoint_id == change_point_id_-1){
            RCLCPP_INFO(this->get_logger(), "Waypoint ID  matched ChangePoint_id. Switching map...");

            RCLCPP_INFO(this->get_logger(), "Waiting For %d Seconds...", wait_time_);

            timer_ = this->create_wall_timer(
                std::chrono::seconds(wait_time_),
                [this]() {
                    
                    RCLCPP_INFO(this->get_logger(), "%d seconds passed. Start map switching...", wait_time_);

                    SwitchPCD(pcd_file_path_);
                    SwitchMap(map_file_path_);

                    second_timer_ = this->create_wall_timer(
                        std::chrono::seconds(3),
                        [this](){

                            auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
                            msg->header.stamp = now();
                            msg->header.frame_id = "map";
                            msg->pose.pose = initial_pose.pose.pose;

                            SetInitialPose(msg);

                            second_timer_ ->cancel();

                            }
                        );

                    RCLCPP_INFO(this->get_logger(), "Switching ALL Map completed!!");

                    LoadInitialPose(pose_path_, pose_number_);
                    LoadMapInfo(map_path_, map_number_);
                    pose_number_ ++;
                    map_number_++;

                    timer_ ->cancel();

                }
            );

        } else {

            RCLCPP_INFO(this->get_logger(), "Waypoint ID  does not match ChangePoint_id.");

        }

    }



    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr second_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_id_sub_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud_;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr second_timer_;
    nav_msgs::msg::Path::SharedPtr path_ptr_;
    std::string pcd_path_, pcd_file_path_, map_path_, map_file_path_, pose_path_;
    int start_number_, pose_number_, map_number_, change_point_id_, wait_time_;
    std::vector<std_msgs::msg::Int32> map_list;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> pose_list;


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapChangeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
