#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>

class MapChangeNode : public rclcpp::Node {
public:
    MapChangeNode() : Node("map_change_node") {
        // 各フロアごとのPGM/YAMLファイルのパス設定       
         map_files_[1] = "/home/shugo/3dnav_ws/src/kuams_navigation/maps/outdoor_5th_ex.yaml";
        // map_files_[2] = "/path/to/floor2.yaml";
        // map_files_[3] = "/path/to/floor3.yaml";

        // フロア情報をサブスクライブ
        floor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/current_floor", 10,
            std::bind(&MapChangeNode::floorCallback, this, std::placeholders::_1));

        // サービスクライアント作成
        map_client_ = this->create_client<std_srvs::srv::Empty>("nav2_map_server");

             std::string map_file_path = map_files_[1];
            // std::string map_file_path = map_files_[current_floor];
            RCLCPP_INFO(this->get_logger(), "Switching to floor 2 map: %s",  map_file_path.c_str());

            // `map_server`のパラメータを更新
            this->declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING); 
            this->set_parameters({
                rclcpp::Parameter("yaml_filename", rclcpp::PARAMETER_STRING)
            });

            // 地図をリロードするために`map_server`のサービスを呼び出し
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            if (!map_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Map server not available");
                return;
            }
            auto result = map_client_->async_send_request(request);
        
    }

private:
    void floorCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        int current_floor = msg->data;

        // フロアに対応する地図ファイルがあるか確認
        if (map_files_.find(current_floor) != map_files_.end()) {
             std::string map_file_path = map_files_[1];
            // std::string map_file_path = map_files_[current_floor];
            RCLCPP_INFO(this->get_logger(), "Switching to floor %d map: %s", current_floor, map_file_path.c_str());

            // `map_server`のパラメータを更新
            this->set_parameters({
                rclcpp::Parameter("yaml_filename", map_file_path)
            });

            // 地図をリロードするために`map_server`のサービスを呼び出し
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            if (!map_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Map server not available");
                return;
            }
            auto result = map_client_->async_send_request(request);
        } else {
            RCLCPP_WARN(this->get_logger(), "No map available for floor %d", current_floor);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr floor_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr map_client_;
    std::unordered_map<int, std::string> map_files_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapChangeNode>());
    rclcpp::shutdown();
    return 0;
}
