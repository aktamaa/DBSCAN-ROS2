#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <random>
#include <unordered_map>
#include "dbscan_ros/dbscan.hpp"

class DBScanNode : public rclcpp::Node {
public:
    DBScanNode() : Node("dbscan_node") {
        this->declare_parameter("eps", 0.5);
        this->declare_parameter("min_pts", 5);
        this->declare_parameter("input_topic", "/glim_ros/points");
        this->declare_parameter("output_topic", "/clustered_pointcloud");

        this->get_parameter("eps", eps_);
        this->get_parameter("min_pts", min_pts_);
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::QoS(10),
            std::bind(&DBScanNode::pointcloud_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "DBScanNode running. Subscribed to %s", input_topic_.c_str());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string input_topic_, output_topic_;
    float eps_;
    int min_pts_;

    std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> generate_cluster_colors(int num_clusters) {
        std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> colors;
        std::mt19937 rng(42); // fixed seed
        std::uniform_int_distribution<int> dist(0, 255);
        for (int i = 0; i < num_clusters; ++i) {
            colors[i] = {static_cast<uint8_t>(dist(rng)), static_cast<uint8_t>(dist(rng)), static_cast<uint8_t>(dist(rng))};
        }
        return colors;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        std::vector<point3> points;
        for (const auto& pt : pcl_cloud->points) {
            if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                points.push_back({pt.x, pt.y, pt.z});
            }
        }

        auto span_data = std::span<const point3>(points);
        auto clusters = dbscan(span_data, eps_, min_pts_);
        auto labels = label(clusters, points.size());

        int max_label = *std::max_element(labels.begin(), labels.end());
        auto color_map = generate_cluster_colors(max_label + 1);

        pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
        for (size_t i = 0; i < points.size(); ++i) {
            pcl::PointXYZRGB pcl_pt;
            pcl_pt.x = points[i].x;
            pcl_pt.y = points[i].y;
            pcl_pt.z = points[i].z;

            int label = labels[i];
            if (label >= 0 && color_map.count(label)) {
                auto [r, g, b] = color_map[label];
                pcl_pt.r = r;
                pcl_pt.g = g;
                pcl_pt.b = b;
            } else {
                pcl_pt.r = 128;
                pcl_pt.g = 128;
                pcl_pt.b = 128; // gray for noise
            }

            output_cloud.push_back(pcl_pt);
        }

        output_cloud.header.frame_id = msg->header.frame_id;
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(output_cloud, output_msg);
        output_msg.header.stamp = msg->header.stamp;
        publisher_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Published colored clustered point cloud with %zu points", output_cloud.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DBScanNode>());
    rclcpp::shutdown();
    return 0;
}
