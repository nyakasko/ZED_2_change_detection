// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include <math.h>
#include <algorithm>
#include "GLViewer.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/common/io.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <windows.h>
#include <iostream>
#include <valarray>
#include <iterator>

class ChangeDetector {
public:
	struct DetectedObject {
		int tracking_id; // id, assigned by the object detector to be able to track
		std::string label; // detection label
		int confidence;     // detection confidence percentage
		sl::float3 position;  // object 3D centroid
		std::vector<sl::uint2> bounding_box_2d; // 2D bounding box corners
		std::vector<sl::float3> bounding_box_3d; // 3D bounding box corners
		std::vector<sl::float3> bounding_box_3d_sum; // 3D bounding box corners summed up from previous detections
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_3d_pointcloud;
		bool has_object_3d_pointcloud;
		std::map<std::string, int> label_confidence_pair;
		std::map<std::string, int> label_detection_num_pair;
		int detection_num;
		int overall_detection_num;
		bool ismoving;
	};
	void measurement_to_pcl(sl::Mat measurement, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
	float convertColor(float colorIn);
	void show_object_detection_on_point_cloud(std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer, sl::Objects objects, int &id);
	void segment_and_show_bounding_box_from_point_cloud(std::shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer, sl::Objects objects,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl, int& id);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_bounding_box(std::vector<sl::float3> bounding_box_3d, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud);
	void show_object_detection_on_image(sl::Objects objects, cv::Mat image_zed_ocv, sl::Resolution display_resolution, sl::Resolution camera_resolution);
	cv::Point resize_boundingbox_coordinates(int x, int y, sl::Resolution display_resolution, sl::Resolution camera_resolution);
	std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	float knn_search(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ref, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_sample, int distance);
	std::vector<int> return_closest_objects(std::vector<ChangeDetector::DetectedObject> DetectedObjects, ChangeDetector::DetectedObject newDetectedObject, int centroid_distance, bool verbose);
	void data_association_of_detected_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, sl::Objects objects,
		std::vector<ChangeDetector::DetectedObject> &DetectedObjects, int eucl_dist, int kd_dis, bool verbose);
	void class_label_and_confidence_decision(std::vector<ChangeDetector::DetectedObject>& DetectedObjects);
	void visualize_end_result(std::string input_pointcloud_path, std::vector<ChangeDetector::DetectedObject>& DetectedObjects);
	void registerNewObject(sl::ObjectData zedObject, ChangeDetector::DetectedObject& newDetectedObject, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, bool verbose);
};