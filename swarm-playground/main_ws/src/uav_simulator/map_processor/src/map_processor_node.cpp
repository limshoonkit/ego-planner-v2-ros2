#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <math.h>
#include <iostream>
#include <Eigen/Eigen>
#include <string>
#include <vector>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

enum class ObstacleShape
{
    SPHERE,
    CYLINDER,
    BOX,
    CONE
};

struct ObstacleConfig
{
    ObstacleShape shape;
    Eigen::Vector3f position;
    Eigen::Vector3f size; // radius/width, height, depth (usage depends on shape)
    float density;        // points per unit volume
};

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode() : Node("map_processor_node"), map_ok_(false),
                         gen_(std::random_device{}())
    {
        this->initializeParameters();
        this->initializeROSComms();
        this->loadMapFromFileAndProcess();

        if (!map_ok_)
        {
            RCLCPP_ERROR(this->get_logger(), "Map could not be loaded or processed.");
        }
        else
        {
            pub_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / param_rate_),
                std::bind(&MapProcessorNode::pubSensedPointsTimerCallback, this));
        }
    }

private:
    void initializeParameters()
    {
        this->declare_parameter("pcd_file_path", "default_map.pcd");
        this->declare_parameter("voxel_leaf_size", 0.1);
        this->declare_parameter("starting_point.x", 0.0);
        this->declare_parameter("starting_point.y", 0.0);
        this->declare_parameter("map_bounds.min_x", -25.0);
        this->declare_parameter("map_bounds.max_x", 25.0);
        this->declare_parameter("map_bounds.min_y", -25.0);
        this->declare_parameter("map_bounds.max_y", 25.0);
        this->declare_parameter("map_bounds.min_z", 0.0);
        this->declare_parameter("map_bounds.max_z", 15.0);
        this->declare_parameter("rate", 10.0);
        this->declare_parameter("frame_id", "map");

        this->declare_parameter("obstacles.enabled", false);
        this->declare_parameter("obstacles.count", 10);
        this->declare_parameter("obstacles.min_radius", 0.5);
        this->declare_parameter("obstacles.max_radius", 2.0);
        this->declare_parameter("obstacles.min_height", 1.0);
        this->declare_parameter("obstacles.max_height", 5.0);
        this->declare_parameter("obstacles.height_offset_min", 0.0);
        this->declare_parameter("obstacles.height_offset_max", 2.0);
        this->declare_parameter("obstacles.point_density", 100.0); // points per cubic meter
        this->declare_parameter("obstacles.margin_from_bounds", 2.0);
        this->declare_parameter("obstacles.shapes", std::vector<std::string>{"sphere", "cylinder", "box"});

        // Quadrant-based generation parameters
        this->declare_parameter("obstacles.quadrant_mode", "all"); // "all", "top_left", "top_right", "bottom_left", "bottom_right", "custom"
        this->declare_parameter("obstacles.quadrants", std::vector<std::string>{"top_left", "top_right", "bottom_left", "bottom_right"});

        this->get_parameter("pcd_file_path", param_pcd_file_path_);
        this->get_parameter("voxel_leaf_size", param_voxel_leaf_size_);
        this->get_parameter("starting_point.x", param_starting_point_x_);
        this->get_parameter("starting_point.y", param_starting_point_y_);

        this->get_parameter("map_bounds.min_x", param_map_bound_min_x_);
        this->get_parameter("map_bounds.max_x", param_map_bound_max_x_);
        this->get_parameter("map_bounds.min_y", param_map_bound_min_y_);
        this->get_parameter("map_bounds.max_y", param_map_bound_max_y_);
        this->get_parameter("map_bounds.min_z", param_map_bound_min_z_);
        this->get_parameter("map_bounds.max_z", param_map_bound_max_z_);
        this->get_parameter("rate", param_rate_);
        this->get_parameter("frame_id", param_frame_id_);

        this->get_parameter("obstacles.enabled", param_obstacles_enabled_);
        this->get_parameter("obstacles.count", param_obstacle_count_);
        this->get_parameter("obstacles.min_radius", param_min_radius_);
        this->get_parameter("obstacles.max_radius", param_max_radius_);
        this->get_parameter("obstacles.min_height", param_min_height_);
        this->get_parameter("obstacles.max_height", param_max_height_);
        this->get_parameter("obstacles.height_offset_min", param_height_offset_min_);
        this->get_parameter("obstacles.height_offset_max", param_height_offset_max_);
        this->get_parameter("obstacles.point_density", param_point_density_);
        this->get_parameter("obstacles.margin_from_bounds", param_margin_);
        this->get_parameter("obstacles.shapes", param_shape_names_);
        this->get_parameter("obstacles.quadrant_mode", param_quadrant_mode_);
        this->get_parameter("obstacles.quadrants", param_quadrant_names_);

        // Parse shape names
        param_enabled_shapes_.clear();
        for (const auto &shape_name : param_shape_names_)
        {
            if (shape_name == "sphere")
                param_enabled_shapes_.push_back(ObstacleShape::SPHERE);
            else if (shape_name == "cylinder")
                param_enabled_shapes_.push_back(ObstacleShape::CYLINDER);
            else if (shape_name == "box")
                param_enabled_shapes_.push_back(ObstacleShape::BOX);
            else if (shape_name == "cone")
                param_enabled_shapes_.push_back(ObstacleShape::CONE);
        }

        if (param_enabled_shapes_.empty())
        {
            param_enabled_shapes_.push_back(ObstacleShape::SPHERE); // fallback
        }
    }

    void initializeROSComms()
    {
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_cloud", 10);
        RCLCPP_INFO(this->get_logger(), "ROS Communications initialized.");
    }

    void loadMapFromFileAndProcess()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Load PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(param_pcd_file_path_, *raw_cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", param_pcd_file_path_.c_str());
            map_ok_ = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %ld points from %s", raw_cloud->size(), param_pcd_file_path_.c_str());

        // Get bounds for analysis/auto-calculation
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*raw_cloud, min_pt, max_pt);
        RCLCPP_INFO(this->get_logger(), "Original bounds: X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]",
                    min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);

        // Apply voxel filter to reduce data size
        pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud = raw_cloud;
        if (param_voxel_leaf_size_ > 0.0)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(raw_cloud);
            vg.setLeafSize(param_voxel_leaf_size_, param_voxel_leaf_size_, param_voxel_leaf_size_);
            vg.filter(*voxel_cloud);
            working_cloud = voxel_cloud;
            RCLCPP_INFO(this->get_logger(), "Downsampled: %ld -> %ld points", raw_cloud->size(), working_cloud->size());

            // Update bounds if we downsampled
            pcl::getMinMax3D(*working_cloud, min_pt, max_pt);
        }

        // Calculate translation offset if needed
        Eigen::Vector3f translation_offset(0.0f, 0.0f, 0.0f);
        translation_offset.x() = -(min_pt.x + max_pt.x) * 0.5f;
        translation_offset.y() = -(min_pt.y + max_pt.y) * 0.5f;
        translation_offset.z() = 0.0f;

        RCLCPP_INFO(this->get_logger(), "Translation offset: [%.3f,%.3f,%.3f]",
                    translation_offset.x(), translation_offset.y(), translation_offset.z());

        // Single-pass transform and crop into final cloud
        cloud_map_.points.clear();
        cloud_map_.points.reserve(working_cloud->size());

        // Transform + crop in one pass
        for (const auto &pt : working_cloud->points)
        {
            float x_new = pt.x + translation_offset.x() - param_starting_point_x_;
            float y_new = pt.y + translation_offset.y() - param_starting_point_y_;
            float z_new = pt.z + translation_offset.z();

            if (x_new >= param_map_bound_min_x_ && x_new <= param_map_bound_max_x_ &&
                y_new >= param_map_bound_min_y_ && y_new <= param_map_bound_max_y_ &&
                z_new >= param_map_bound_min_z_ && z_new <= param_map_bound_max_z_)
            {
                cloud_map_.points.emplace_back(x_new, y_new, z_new);
            }
        }

        // Add random obstacles if enabled
        if (param_obstacles_enabled_)
        {
            generateRandomObstacles();
        }

        // Set cloud properties
        cloud_map_.width = cloud_map_.points.size();
        cloud_map_.height = 1;
        cloud_map_.is_dense = true;

        if (cloud_map_.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No points remain after cropping! Check bounds overlap.");
            map_ok_ = false;
            return;
        }

        // Pre-convert to ROS message to avoid repeated conversion
        RCLCPP_INFO(this->get_logger(), "Converting to ROS message.");
        cached_ros_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud_map_, *cached_ros_msg_);
        cached_ros_msg_->header.frame_id = param_frame_id_;

        RCLCPP_INFO(this->get_logger(), "Map ready: %ld points, will publish at %.1f Hz",
                    cloud_map_.points.size(), param_rate_);
        map_ok_ = true;
    }

    void generateRandomObstacles()
    {
        RCLCPP_INFO(this->get_logger(), "Generating %d random obstacles in quadrant mode: %s",
                    param_obstacle_count_, param_quadrant_mode_.c_str());
        // Calculate the effective map bounds after all transformations
        // These bounds represent the final coordinate system where obstacles will be placed
        float effective_min_x = param_map_bound_min_x_ - param_starting_point_x_;
        float effective_max_x = param_map_bound_max_x_ - param_starting_point_x_;
        float effective_min_y = param_map_bound_min_y_ - param_starting_point_y_;
        float effective_max_y = param_map_bound_max_y_ - param_starting_point_y_;

        RCLCPP_INFO(this->get_logger(), "Effective obstacle bounds: X[%.2f,%.2f] Y[%.2f,%.2f]",
                    effective_min_x, effective_max_x, effective_min_y, effective_max_y);

        // Define quadrant boundaries (in the final transformed coordinate system)
        float center_x = (effective_min_x + effective_max_x) * 0.5f;
        float center_y = (effective_min_y + effective_max_y) * 0.5f;

        struct QuadrantBounds
        {
            float min_x, max_x, min_y, max_y;
            std::string name;
        };

        std::vector<QuadrantBounds> available_quadrants;

        // Define all quadrants
        std::map<std::string, QuadrantBounds> all_quadrants = {
            {"top_right", {center_x, effective_max_x, center_y, effective_max_y, "top_right"}},
            {"top_left", {effective_min_x, center_x, center_y, effective_max_y, "top_left"}},
            {"bottom_right", {center_x, effective_max_x, effective_min_y, center_y, "bottom_right"}},
            {"bottom_left", {effective_min_x, center_x, effective_min_y, center_y, "bottom_left"}}};

        // Select quadrants based on mode
        if (param_quadrant_mode_ == "all")
        {
            for (const auto &quad : all_quadrants)
            {
                available_quadrants.push_back(quad.second);
            }
        }
        else if (param_quadrant_mode_ == "custom")
        {
            for (const auto &quad_name : param_quadrant_names_)
            {
                if (all_quadrants.find(quad_name) != all_quadrants.end())
                {
                    RCLCPP_INFO(this->get_logger(), "Adding custom quadrant: %s", quad_name.c_str());
                    available_quadrants.push_back(all_quadrants[quad_name]);
                }
            }
        }
        else
        {
            // Single quadrant mode
            if (all_quadrants.find(param_quadrant_mode_) != all_quadrants.end())
            {
                available_quadrants.push_back(all_quadrants[param_quadrant_mode_]);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown quadrant mode: %s, using all quadrants",
                            param_quadrant_mode_.c_str());
                for (const auto &quad : all_quadrants)
                {
                    available_quadrants.push_back(quad.second);
                }
            }
        }

        if (available_quadrants.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid quadrants selected!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Selected %ld quadrants for obstacle placement", available_quadrants.size());
        for (const auto &quad : available_quadrants)
        {
            RCLCPP_INFO(this->get_logger(), "  - %s: X[%.2f,%.2f] Y[%.2f,%.2f]",
                        quad.name.c_str(), quad.min_x, quad.max_x, quad.min_y, quad.max_y);
        }

        std::uniform_real_distribution<float> radius_dist(param_min_radius_, param_max_radius_);
        std::uniform_real_distribution<float> height_dist(param_min_height_, param_max_height_);
        std::uniform_real_distribution<float> offset_dist(param_height_offset_min_, param_height_offset_max_);
        std::uniform_int_distribution<int> shape_dist(0, param_enabled_shapes_.size() - 1);
        std::uniform_int_distribution<int> quadrant_dist(0, available_quadrants.size() - 1);

        size_t initial_point_count = cloud_map_.points.size();

        for (int i = 0; i < param_obstacle_count_; ++i)
        {
            // Select a random quadrant
            const auto &selected_quadrant = available_quadrants[quadrant_dist(gen_)];

            // Generate position within the selected quadrant with margins
            std::uniform_real_distribution<float> x_dist(
                selected_quadrant.min_x + param_margin_,
                selected_quadrant.max_x - param_margin_);
            std::uniform_real_distribution<float> y_dist(
                selected_quadrant.min_y + param_margin_,
                selected_quadrant.max_y - param_margin_);

            ObstacleConfig config;
            config.shape = param_enabled_shapes_[shape_dist(gen_)];
            config.position.x() = x_dist(gen_) - param_starting_point_x_;
            config.position.y() = y_dist(gen_) - param_starting_point_y_;

            float height_offset = offset_dist(gen_);
            config.position.z() = param_map_bound_min_z_ + height_offset;

            config.size.x() = radius_dist(gen_); // radius/width
            config.size.y() = height_dist(gen_); // height
            config.size.z() = config.size.x();   // depth (for box)
            config.density = param_point_density_;

            RCLCPP_DEBUG(this->get_logger(), "Obstacle %d in %s: pos(%.2f, %.2f, %.2f), size(%.2f, %.2f)",
                         i, selected_quadrant.name.c_str(),
                         config.position.x(), config.position.y(), config.position.z(),
                         config.size.x(), config.size.y());

            generateObstacle(config);
        }

        size_t obstacle_points = cloud_map_.points.size() - initial_point_count;
        RCLCPP_INFO(this->get_logger(), "Added %ld obstacle points", obstacle_points);
    }

    void generateObstacle(const ObstacleConfig &config)
    {
        switch (config.shape)
        {
        case ObstacleShape::SPHERE:
            generateSphere(config);
            break;
        case ObstacleShape::CYLINDER:
            generateCylinder(config);
            break;
        case ObstacleShape::BOX:
            generateBox(config);
            break;
        case ObstacleShape::CONE:
            generateCone(config);
            break;
        }
    }

    void generateSphere(const ObstacleConfig &config)
    {
        float radius = config.size.x();
        float volume = (4.0f / 3.0f) * M_PI * radius * radius * radius;
        int num_points = static_cast<int>(volume * config.density);

        std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);
        std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * M_PI);

        for (int i = 0; i < num_points; ++i)
        {
            // Generate random point inside sphere using rejection sampling
            float x, y, z;
            do
            {
                x = 2.0f * uniform_dist(gen_) - 1.0f;
                y = 2.0f * uniform_dist(gen_) - 1.0f;
                z = 2.0f * uniform_dist(gen_) - 1.0f;
            } while (x * x + y * y + z * z > 1.0f);

            // Scale by radius and translate
            pcl::PointXYZ point;
            point.x = config.position.x() + x * radius;
            point.y = config.position.y() + y * radius;
            point.z = config.position.z() + z * radius;

            // Check bounds
            if (isPointInBounds(point))
            {
                cloud_map_.points.push_back(point);
            }
        }
    }

    void generateCylinder(const ObstacleConfig &config)
    {
        float radius = config.size.x();
        float height = config.size.y();
        float volume = M_PI * radius * radius * height;
        int num_points = static_cast<int>(volume * config.density);

        std::uniform_real_distribution<float> r_dist(0.0f, radius);
        std::uniform_real_distribution<float> theta_dist(0.0f, 2.0f * M_PI);
        std::uniform_real_distribution<float> z_dist(0.0f, height);

        for (int i = 0; i < num_points; ++i)
        {
            float r = sqrt(r_dist(gen_)); // sqrt for uniform distribution
            float theta = theta_dist(gen_);
            float z = z_dist(gen_);

            pcl::PointXYZ point;
            point.x = config.position.x() + r * cos(theta);
            point.y = config.position.y() + r * sin(theta);
            point.z = config.position.z() + z;

            if (isPointInBounds(point))
            {
                cloud_map_.points.push_back(point);
            }
        }
    }

    void generateBox(const ObstacleConfig &config)
    {
        float width = config.size.x() * 2.0f; // full width
        float height = config.size.y();
        float depth = config.size.z() * 2.0f; // full depth
        float volume = width * height * depth;
        int num_points = static_cast<int>(volume * config.density);

        std::uniform_real_distribution<float> x_dist(-config.size.x(), config.size.x());
        std::uniform_real_distribution<float> y_dist(-config.size.z(), config.size.z());
        std::uniform_real_distribution<float> z_dist(0.0f, height);

        for (int i = 0; i < num_points; ++i)
        {
            pcl::PointXYZ point;
            point.x = config.position.x() + x_dist(gen_);
            point.y = config.position.y() + y_dist(gen_);
            point.z = config.position.z() + z_dist(gen_);

            if (isPointInBounds(point))
            {
                cloud_map_.points.push_back(point);
            }
        }
    }

    void generateCone(const ObstacleConfig &config)
    {
        float base_radius = config.size.x();
        float height = config.size.y();
        float volume = (1.0f / 3.0f) * M_PI * base_radius * base_radius * height;
        int num_points = static_cast<int>(volume * config.density);

        std::uniform_real_distribution<float> z_dist(0.0f, height);
        std::uniform_real_distribution<float> theta_dist(0.0f, 2.0f * M_PI);
        std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

        for (int i = 0; i < num_points; ++i)
        {
            float z = z_dist(gen_);
            float radius_at_z = base_radius * (1.0f - z / height); // Linear taper
            float r = sqrt(uniform_dist(gen_)) * radius_at_z;
            float theta = theta_dist(gen_);

            pcl::PointXYZ point;
            point.x = config.position.x() + r * cos(theta);
            point.y = config.position.y() + r * sin(theta);
            point.z = config.position.z() + z;

            if (isPointInBounds(point))
            {
                cloud_map_.points.push_back(point);
            }
        }
    }

    bool isPointInBounds(const pcl::PointXYZ &point)
    {
        return point.x >= param_map_bound_min_x_ && point.x <= param_map_bound_max_x_ &&
               point.y >= param_map_bound_min_y_ && point.y <= param_map_bound_max_y_ &&
               point.z >= param_map_bound_min_z_ && point.z <= param_map_bound_max_z_;
    }

    void pubSensedPointsTimerCallback()
    {
        if (!map_ok_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Map not ready, skipping publication");
            return;
        }

        // Just update timestamp and publish
        cached_ros_msg_->header.stamp = this->now();
        map_pub_->publish(*cached_ros_msg_);
    }

    // ROS Comms
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> cached_ros_msg_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    // PCL & Map Data
    pcl::PointCloud<pcl::PointXYZ> cloud_map_;

    // State
    bool map_ok_;

    // Random number generation
    std::mt19937 gen_;

    // Existing parameters
    std::string param_pcd_file_path_;
    std::string param_frame_id_;
    double param_rate_;
    double param_voxel_leaf_size_;
    double param_starting_point_x_, param_starting_point_y_;
    double param_map_bound_min_x_, param_map_bound_max_x_;
    double param_map_bound_min_y_, param_map_bound_max_y_;
    double param_map_bound_min_z_, param_map_bound_max_z_;

    // Obstacle parameters
    bool param_obstacles_enabled_;
    int param_obstacle_count_;
    double param_min_radius_, param_max_radius_;
    double param_min_height_, param_max_height_;
    double param_height_offset_min_, param_height_offset_max_;
    double param_point_density_;
    double param_margin_;
    std::vector<std::string> param_shape_names_;
    std::vector<ObstacleShape> param_enabled_shapes_;
    std::string param_quadrant_mode_;
    std::vector<std::string> param_quadrant_names_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}