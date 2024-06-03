#include <chrono>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>


using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;
// typedef boost::shared_ptr<pcl::PointCloud<PointT>> PointCloudPtr;

class VoxelGrid_filter : public rclcpp::Node
{
  public:
    VoxelGrid_filter()
    : Node("minimal_publisher")
    {
      marker_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
      subscription_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/points", 10, std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

      publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("cylinder_seg", 10);

    }

      private:
    
        void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
        {

          pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>) ;

          pcl::fromROSMsg(*input_cloud, *pcl_cloud);

          pcl::PointCloud<PointT>::Ptr passthrough_cloud (new pcl::PointCloud<PointT>) ;
          
          // Along X Axis
          pcl::PassThrough<PointT> passing_x;
          passing_x.setInputCloud(pcl_cloud);
          passing_x.setFilterFieldName("x");
          passing_x.setFilterLimits(-0.5,0.5);
          passing_x.filter(*passthrough_cloud);

          // Along Y Axis
          pcl::PassThrough<PointT> passing_y;
          passing_y.setInputCloud(passthrough_cloud);
          passing_y.setFilterFieldName("y");
          passing_y.setFilterLimits(-3,3);
          passing_y.filter(*passthrough_cloud);

          // Voxel Filter
          pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>) ;
          pcl::VoxelGrid<PointT> voxel_filter;
          voxel_filter.setInputCloud(pcl_cloud);
          voxel_filter.setLeafSize(0.02 , 0.02, 0.02 );
          voxel_filter.filter(*voxel_cloud);

          // pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>) ;
          // pcl::VoxelGrid<PointT> voxel_filter;
          // voxel_filter.setInputCloud(pcl_cloud);
          // voxel_filter.setLeafSize(0.01 , 0.01, 0.01);
          // voxel_filter.filter(*voxel_cloud_viz);

          //==================================== Road Segmentation  ====================================
          pcl::NormalEstimation<PointT, pcl::Normal> normal_extractor;
          pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
          pcl::PointCloud<pcl::Normal>::Ptr road_normals(new pcl::PointCloud<pcl::Normal>);

          pcl::SACSegmentationFromNormals<PointT, pcl::Normal> rplane_surface_seg_frm_normals;
          pcl::PointIndices::Ptr plane_surface_inliers(new pcl::PointIndices);
          pcl::ModelCoefficients::Ptr plane_surface_coefficients(new pcl::ModelCoefficients);
          pcl::ExtractIndices<PointT> plane_surface_indices;
          pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>);


          // Normals Extractions
          normal_extractor.setSearchMethod(tree);
          normal_extractor.setInputCloud(voxel_cloud);
          normal_extractor.setKSearch(30);
          normal_extractor.compute(*road_normals);

          // Parameters for Planar Segmentation
          rplane_surface_seg_frm_normals.setOptimizeCoefficients(true);
          rplane_surface_seg_frm_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
          rplane_surface_seg_frm_normals.setMethodType(pcl::SAC_RANSAC);
          rplane_surface_seg_frm_normals.setNormalDistanceWeight(0.5);
          rplane_surface_seg_frm_normals.setMaxIterations(100);
          rplane_surface_seg_frm_normals.setDistanceThreshold(0.4);
          rplane_surface_seg_frm_normals.setInputCloud(voxel_cloud);
          rplane_surface_seg_frm_normals.setInputNormals(road_normals);
          rplane_surface_seg_frm_normals.segment(*plane_surface_inliers,*plane_surface_coefficients);

          //Extracting Cloud based on Inliers indices
          plane_surface_indices.setInputCloud(voxel_cloud);
          plane_surface_indices.setIndices(plane_surface_inliers);
          plane_surface_indices.setNegative(true);
          plane_surface_indices.filter(*road_cloud);
            
            


        // ********************************     Cylinder Segmentation
          
          
    // pcl::ModelCoefficients::Ptr cylinder_co(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr cylinder_in(new pcl::PointIndices);
    // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::Kdtree<PointT> ());
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // pcl::PointCloud<PointT>::Ptr cylinder_cloud(new pcl::PointCloud<PointT> ());

    // pcl::NormalEstimation<PointT, pcl::Normal> normals_estimator;
    // pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
    // pcl::ExtractIndices<PointT> cylinder_indices_extractor;

          
          
          
          pcl::PointCloud<PointT>::Ptr segmented_cluster (new pcl::PointCloud<PointT>);
          pcl::PointCloud<PointT>::Ptr all_clusters (new pcl::PointCloud<PointT>);
          tree->setInputCloud (road_cloud);
          std::vector<pcl::PointIndices> cluster_indices;
          pcl::EuclideanClusterExtraction<PointT> ec;


          struct BBox
          {
            float x_min;
            float x_max;
            float y_min;
            float y_max;
            float z_min;
            float z_max;
            double r = 1.0;
            double g = 0.0;
            double b = 0.0;
          };
          ec.setClusterTolerance (0.2); // 2cm
          ec.setMinClusterSize (10);
          ec.setMaxClusterSize (130);
          ec.setSearchMethod (tree);
          ec.setInputCloud (road_cloud);
          ec.extract (cluster_indices);
          std::vector<BBox> bboxes;

          size_t min_reasonable_size = 10;
          size_t max_reasonable_size = 1000;

          // int num_reasonable_clusters = 0;
          for (size_t i = 0; i < cluster_indices.size(); i++)
          {
              if (cluster_indices[i].indices.size() > min_reasonable_size && cluster_indices[i].indices.size() < max_reasonable_size)
              {
                  pcl::PointCloud<PointT>::Ptr reasonable_cluster (new pcl::PointCloud<PointT>);
                  pcl::ExtractIndices<PointT> extract;
                  pcl::IndicesPtr indices(new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));
                  extract.setInputCloud (road_cloud);
                  extract.setIndices(indices);
                  extract.setNegative (false);
                  extract.filter (*reasonable_cluster);
                  all_clusters->operator+=(*reasonable_cluster);
                  // num_reasonable_clusters++;

                  Eigen::Vector4f min_pt, max_pt;
                  pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

                  pcl::PointXYZ center((min_pt[0] + max_pt[0]) / 2.0, (min_pt[1] + max_pt[1]) / 2.0, (min_pt[2] + max_pt[2]) / 2.0);
                  BBox bbox;
                  bbox.x_min = min_pt[0];
                  bbox.y_min = min_pt[1];
                  bbox.z_min = min_pt[2];
                  bbox.x_max = max_pt[0];
                  bbox.y_max = max_pt[1];
                  bbox.z_max = max_pt[2];

                  bboxes.push_back(bbox);
              }
          }

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const std_msgs::msg::Header& inp_header = input_cloud->header;
    // Create a marker for each bounding box
    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::msg::Marker top_square_marker;
        top_square_marker.header = inp_header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::msg::Marker::ADD;
        top_square_marker.pose.orientation.w = 1.0;
        top_square_marker.scale.x = 0.01;
        top_square_marker.color.r = 0.0;
        top_square_marker.color.g = 1.0;
        top_square_marker.color.b = 0.0;
        top_square_marker.color.a = 1.0;

        // Create the marker for the bottom square
        visualization_msgs::msg::Marker bottom_square_marker;
        bottom_square_marker.header = inp_header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::msg::Marker::ADD;
        bottom_square_marker.pose.orientation.w = 1.0;
        bottom_square_marker.scale.x = 0.01;
        bottom_square_marker.color.r = 0.0;
        bottom_square_marker.color.g = 1.0;
        bottom_square_marker.color.b = 0.0;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;

        // Add the points to the bottom square marker
        geometry_msgs::msg::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        struct Point3D {
          double x;
          double y;
          double z;
        };

        Point3D points[] = {
          {p1.x, p1.y, p1.z},
          {p2.x, p2.y, p2.z},
          {p3.x, p3.y, p3.z},
          {p4.x, p4.y, p4.z},
          {p5.x, p5.y, p5.z},
          {p6.x, p6.y, p6.z},
          {p7.x, p7.y, p7.z},
          {p8.x, p8.y, p8.z},
        };

        size_t num_points = sizeof(points) / sizeof(points[0]);

        double angle = 42.0;
        char axis = 'X';

        Eigen::Vector3d centroid(0, 0, 0);
        for (size_t i = 0; i < num_points; ++i) {
          centroid += Eigen::Vector3d(points[i].x, points[i].y, points[i].z);
        }
        centroid /= static_cast<double>(num_points);

        // Convert angle to radians
        double radians = angle * M_PI / 180.0;

        // Define rotation matrix based on axis
        Eigen::Matrix3d rotation;
        if (axis == 'X') {
          rotation = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitX());
        } else if (axis == 'Y') {
          rotation = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitY());
        } else if (axis == 'Z') {
          rotation = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ());
        }

        // Translate points to origin, rotate, and translate back
        for (size_t i = 0; i < num_points; ++i) {
          Eigen::Vector3d point(points[i].x, points[i].y, points[i].z);
          point = (point) + rotation * (point - centroid);
          point = centroid + (point - centroid)*0.55;
          points[i].x = point[0];
          points[i].y = point[1];
          points[i].z = point[2];
        }

        p1.x = points[0].x; p1.y = points[0].y; p1.z = points[0].z;
        p2.x = points[1].x; p2.y = points[1].y; p2.z = points[1].z;
        p3.x = points[2].x; p3.y = points[2].y; p3.z = points[2].z;
        p4.x = points[3].x; p4.y = points[3].y; p4.z = points[3].z;

        p5.x = points[4].x; p5.y = points[4].y; p5.z = points[4].z;
        p6.x = points[5].x; p6.y = points[5].y; p6.z = points[5].z;
        p7.x = points[6].x; p7.y = points[6].y; p7.z = points[6].z;
        p8.x = points[7].x; p8.y = points[7].y; p8.z = points[7].z;

        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::msg::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        connecting_lines_marker.pose.orientation.w = 1.0;
        connecting_lines_marker.scale.x = 0.01;
        connecting_lines_marker.color.r = 0.0;
        connecting_lines_marker.color.g = 1.0;
        connecting_lines_marker.color.b = 0.0;
        connecting_lines_marker.color.a = 1.0;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::msg::Marker corner_marker;
        corner_marker.header = inp_header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
        corner_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.01;
        corner_marker.scale.y = 0.01;
        corner_marker.scale.z = 0.01;
        corner_marker.color.r = 0.0;
        corner_marker.color.g = 1.0;
        corner_marker.color.b = 0.0;
        corner_marker.color.a = 1.0;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        // geometry_msgs::msg::Point centroid_;
        // centroid_.x = centroid.x, centroid_.y = centroid.y, centroid_.z = centroid.z; 
        // corner_marker.pose.position = centroid_;
        // corner_marker.id = id++;
        // marker_array.markers.push_back(corner_marker);

        marker_pub->publish(marker_array);
    }


          // Convert cloud to ros2 message
          sensor_msgs::msg::PointCloud2 voxel_cloud_ros2;
          pcl::toROSMsg(*all_clusters, voxel_cloud_ros2);
          voxel_cloud_ros2.header = input_cloud->header;

          publisher_->publish(voxel_cloud_ros2);

        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

        size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGrid_filter>());
  rclcpp::shutdown();
  return 0;
}
