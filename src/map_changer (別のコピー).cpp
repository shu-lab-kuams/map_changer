#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>

class MapChangeNode : public rclcpp::Node {
public:
    MapChangeNode() : Node("map_change_node") {

        declare_parameter("pcd_path", std::string("/path/to/your/pcd"));
        declare_parameter("map_path", std::string("/path/to/your/yaml"));
        declare_parameter("pose_path", std::string("/path/to/your/csv"));
        declare_parameter("change_point_id", change_point_id_);
        declare_parameter("set_initial_pose", false);
        declare_parameter("initial_pose_x", 0.0);
        declare_parameter("initial_pose_y", 0.0);
        declare_parameter("initial_pose_z", 0.0);
        declare_parameter("initial_pose_qx", 0.0);
        declare_parameter("initial_pose_qy", 0.0);
        declare_parameter("initial_pose_qz", 0.0);
        declare_parameter("initial_pose_qw", 1.0);


        get_parameter("pcd_path", pcd_path_);
        get_parameter("map_path", map_path_);
        get_parameter("pose_path", pose_path_);
        get_parameter("change_point_id", change_point_id_);
        get_parameter("set_initial_pose", set_initial_pose_);
        get_parameter("initial_pose_x", initial_pose_x_);
        get_parameter("initial_pose_y", initial_pose_y_);
        get_parameter("initial_pose_z", initial_pose_z_);
        get_parameter("initial_pose_qx", initial_pose_qx_);
        get_parameter("initial_pose_qy", initial_pose_qy_);
        get_parameter("initial_pose_qz", initial_pose_qz_);
        get_parameter("initial_pose_qw", initial_pose_qw_);


        RCLCPP_INFO(get_logger(),"PCD_path: %s ", pcd_path_.c_str());
        RCLCPP_INFO(get_logger(),"Map_path: %s ", map_path_.c_str());
        RCLCPP_INFO(get_logger(),"ChangePoint_id: %d ", change_point_id_);
        RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
        RCLCPP_INFO(get_logger(),"initial_pose_x: %lf", initial_pose_x_);
        RCLCPP_INFO(get_logger(),"initial_pose_y: %lf", initial_pose_y_);
        RCLCPP_INFO(get_logger(),"initial_pose_z: %lf", initial_pose_z_);
        RCLCPP_INFO(get_logger(),"initial_pose_qx: %lf", initial_pose_qx_);
        RCLCPP_INFO(get_logger(),"initial_pose_qy: %lf", initial_pose_qy_);
        RCLCPP_INFO(get_logger(),"initial_pose_qz: %lf", initial_pose_qz_);
        RCLCPP_INFO(get_logger(),"initial_pose_qw: %lf", initial_pose_qw_);


        // 指定されたyamlファイルを使用するため、初期化時にサービスリクエストを送信
        map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

        //Publisher
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/initial_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
            
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", rclcpp::SystemDefaultsQoS());

        //Subscriber
        waypoint_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "waypoint_id", rclcpp::QoS(10),
            std::bind(&MapChangeNode::WaypointCallback, this, std::placeholders::_1));

    }

private:
    void SwitchMap(std::string map_path_){
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = map_path_;

        // サービスが利用可能になるまで待機
        if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Map server not available");
            return;
        }

        // サービスを非同期で呼び出し
        auto future = map_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Switching map completed");

    }

    void SwitchPCD(std::string pcd_path_){

        // PCDファイルの読み込み
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud) == -1) {
           RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file");
           return;
        }

        second_cloud_ = cloud;

        //Publish PCD
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*second_cloud_, output_msg);
        output_msg.header.frame_id = "map";  // 適切なフレームIDを指定
        point_cloud_pub_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Switching PCD completed");


    }

    void SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){

        //Publish InitialPose
        initial_pose_stamped_ptr_ = msg;
        initial_pose_pub_->publish(*initial_pose_stamped_ptr_);
        RCLCPP_INFO(this->get_logger(), " InitialPose Published");

    }


    void WaypointCallback(const std_msgs::msg::Int32::SharedPtr msg){
        int waypoint_id = msg->data;
        RCLCPP_INFO(get_logger(),"Waypoint ID: %d", msg->data);

        if(waypoint_id == change_point_id_){
            RCLCPP_INFO(this->get_logger(), "Waypoint ID  matched ChangePoint_id. Switching map...");

        if (set_initial_pose_== 1) {
            auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

            msg->header.stamp = now();
            msg->header.frame_id = "map";
            msg->pose.pose.position.x = initial_pose_x_;
            msg->pose.pose.position.y = initial_pose_y_;
            msg->pose.pose.position.z = initial_pose_z_;
            msg->pose.pose.orientation.x = initial_pose_qx_;
            msg->pose.pose.orientation.y = initial_pose_qy_;
            msg->pose.pose.orientation.z = initial_pose_qz_;
            msg->pose.pose.orientation.w = initial_pose_qw_;
            geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
            pose_stamped->header.stamp = msg->header.stamp;
            pose_stamped->header.frame_id = "map";
            pose_stamped->pose = msg->pose.pose;

            SetInitialPose(msg);
        }

            SwitchPCD(pcd_path_);
            SwitchMap(map_path_);

            RCLCPP_INFO(this->get_logger(), "Switching ALL Map completed!!");

        } else {

            RCLCPP_INFO(this->get_logger(), "Waypoint ID  does not match ChangePoint_id.");

        }

    }


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_id_sub_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_client_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_stamped_ptr_;
    int change_point_id_;
    std::string pcd_path_;
    std::string map_path_;
    bool set_initial_pose_{false};
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_z_;
    double initial_pose_qx_;
    double initial_pose_qy_;
    double initial_pose_qz_;
    double initial_pose_qw_;


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapChangeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
