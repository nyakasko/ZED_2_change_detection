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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

static bool sortbysec(std::tuple<int, float>& a, std::tuple<int, float>& b);

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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_3d_pointcloud; // segmented (by the bounding box) point cloud of the object
		bool has_object_3d_pointcloud; // if the detected object has 3d coordinates in its point cloud
		std::map<std::string, int> label_confidence_pair; // map with the object's detected label and confidence -- needed for post-processing
		std::map<std::string, int> label_detection_num_pair; // map with the object's detected label and number of detections -- needed for post-processing
		int detection_num; // number of detections
		int overall_detection_num; // overall number of detections -- after post-processing
		bool ismoving; // indicates if the object is moving or not
		std::string path_to_pointcloud; // path to the stored point cloud file of the object
	};
	struct ZEDParameters {
		sl::InitParameters init_parameters; // camera initial parameters
		sl::CameraParameters calib_param_; // camera calibration parameters
		sl::Resolution display_resolution; // display resolution
		sl::Resolution resolution; // camera resolution
		float ratio_width;
		float ratio_height;
	};
	std::map <std::string, sl::OBJECT_SUBCLASS> get_sl_subclass{
		{"Person", sl::OBJECT_SUBCLASS::PERSON },
		{"Bicycle", sl::OBJECT_SUBCLASS::BICYCLE },
		{"Car", sl::OBJECT_SUBCLASS::CAR },
		{"Motorbike", sl::OBJECT_SUBCLASS::MOTORBIKE },
		{"Bus", sl::OBJECT_SUBCLASS::BUS },
		{"Truck", sl::OBJECT_SUBCLASS::TRUCK },
		{"Boat", sl::OBJECT_SUBCLASS::BOAT },
		{"Backpack", sl::OBJECT_SUBCLASS::BACKPACK },
		{"Handbag", sl::OBJECT_SUBCLASS::HANDBAG },
		{"Suitcase", sl::OBJECT_SUBCLASS::SUITCASE },
		{"Bird", sl::OBJECT_SUBCLASS::BIRD },
		{"Cat", sl::OBJECT_SUBCLASS::CAT },
		{"Dog", sl::OBJECT_SUBCLASS::DOG },
		{"Horse", sl::OBJECT_SUBCLASS::HORSE },
		{"Sheep", sl::OBJECT_SUBCLASS::SHEEP },
		{"Cow", sl::OBJECT_SUBCLASS::COW },
		{"Cellphone", sl::OBJECT_SUBCLASS::CELLPHONE },
		{"Laptop", sl::OBJECT_SUBCLASS::LAPTOP },
		{"Banana", sl::OBJECT_SUBCLASS::BANANA },
		{"Apple", sl::OBJECT_SUBCLASS::APPLE },
		{"Orange", sl::OBJECT_SUBCLASS::ORANGE },
		{"Carrot", sl::OBJECT_SUBCLASS::CARROT}
	};

	void measurement_to_pcl(sl::Mat measurement, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
	float convertColor(float colorIn);
	void show_object_on_image(ChangeDetector::DetectedObject object, cv::Mat& image_zed_ocv, cv::Point top_left, cv::Point bottom_right);
	void show_object_detection_on_point_cloud(std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer, sl::Objects objects, int &id);
	void segment_and_show_bounding_box_from_point_cloud(std::shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer, sl::Objects objects,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl, int& id);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_bounding_box(std::vector<sl::float3> bounding_box_3d, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud);
	void show_object_detection_on_image(sl::Objects objects, cv::Mat image_zed_ocv, int& detection_confidence);
	cv::Point resize_coordinates(int x, int y);
	std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	float knn_search(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ref, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_sample, int distance);
	template <class T>
	std::vector<int> return_closest_objects(std::vector<ChangeDetector::DetectedObject> DetectedObjects, T newDetectedObject, int centroid_distance, bool verbose);
	void data_association_of_detected_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, sl::Objects objects,
		std::vector<ChangeDetector::DetectedObject> &DetectedObjects, int eucl_dist, int kd_dis, bool verbose);
	void class_label_and_confidence_decision(std::vector<ChangeDetector::DetectedObject>& DetectedObjects);
	void save_and_visualize_end_result(std::string input_pointcloud_path, std::vector<ChangeDetector::DetectedObject>& DetectedObjects);
	void registerNewObject(sl::ObjectData zedObject, ChangeDetector::DetectedObject& newDetectedObject, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, bool verbose);
	void read_previously_saved_detected_objects(std::string saved_xml_file_path, std::vector<ChangeDetector::DetectedObject>& PreviouslyDetectedObjects);
	sl::Translation transform_p_world_to_p_cam(sl::Translation current_pos, sl::Pose cam_pose);
	cv::Point _3d_point_to_2d_pixel(sl::Translation new_position);
	void find_and_reproject_previous_detections_onto_image(ChangeDetector::DetectedObject& prev_obj, cv::Mat& image_zed_ocv, sl::Pose cam_pose, cv::Point Pixel);
	sl::ObjectData add_previous_detections_to_sl_objects(ChangeDetector::DetectedObject prev_obj, sl::Pose cam_pose, std::vector<sl::ObjectData>& object_list, cv::Point Pixel);
	template <class T>
	void display_change_or_no_change_of_object(cv::Mat& image_zed_ocv, T found_prev, bool change);
	void compare_for_change(pcl::PointXYZRGB min_, pcl::PointXYZRGB max_, std::vector<ChangeDetector::DetectedObject>& PreviouslyDetectedObjects, sl::Pose cam_pose,
		cv::Mat& image_zed_ocv, std::vector<sl::ObjectData>& object_list, std::vector<ChangeDetector::DetectedObject>& DetectedObjects);
	void setZedParameters(sl::InitParameters init_parameters, sl::CameraParameters calib_param_, sl::Resolution display_resolution, float ratio_width, float ratio_height);

};

/**
 * This is a comparison function to sort the vector elements by second element of tuples
 * input1: first tuple to compare
 * input2: second tuple to compare
 * return: True if a < b, otherwise False
 **/
static bool sortbysec(std::tuple<int, float>& a, std::tuple<int, float>& b) {
    return (std::get<1>(a) < std::get<1>(b));
}

/**
 * This function returns those objects' ids whose centroids' Eucledian distance is below the 'centroid_distance' threshold
 * input1: list of detected objects
 * input2: new detected object
 * input3: centroid distance threshold
 * input4: verbosity level for debugging
 * return: ids of the closest previously detected objects
 **/
template <class T>
std::vector<int> ChangeDetector::return_closest_objects(std::vector<ChangeDetector::DetectedObject> DetectedObjects, T newDetectedObject, int centroid_distance, bool verbose = false) {
	std::vector<int> closest_objects;
	std::vector<std::tuple<int, float>> pairedIndicesAndDistances;
	auto detected_posi = newDetectedObject.position;
	for (int i = 0; i < DetectedObjects.size(); i++) {
		float distance = sqrt(pow(detected_posi.x - DetectedObjects[i].position.x, 2) +
			pow(detected_posi.y - DetectedObjects[i].position.y, 2) +
			pow(detected_posi.z - DetectedObjects[i].position.z, 2) * 1.0);
		pairedIndicesAndDistances.push_back(std::make_tuple(i, distance));
	}
	sort(pairedIndicesAndDistances.begin(), pairedIndicesAndDistances.end(), sortbysec);
	for (int elem = 0; elem < DetectedObjects.size(); elem++) {
		if (verbose) {
			std::cout << "Distance to " << DetectedObjects[std::get<0>(pairedIndicesAndDistances[elem])].label
				<< " " << std::get<0>(pairedIndicesAndDistances[elem]) << " stored element: " << std::get<1>(pairedIndicesAndDistances[elem]) << std::endl;
		}
		if (std::get<1>(pairedIndicesAndDistances[elem]) < centroid_distance) {
			closest_objects.push_back(std::get<0>(pairedIndicesAndDistances[elem]));
		}
	}
	return closest_objects;
}


/**
 * This function displays the change or match between 2 consecutive runs
 * input1: opencv image
 * input2: previously found object
 * input3: whether there was a change or a match between 2 runs
 **/
template <class T>
void ChangeDetector::display_change_or_no_change_of_object(cv::Mat& image_zed_ocv, T found_prev, bool change = true) {
	//SetConsoleTextAttribute(hConsole, 5);
	//printf("Detected change - NEW OBJECT %s, because there were no newly detected close objects.\n", (std::string)sl::toString(found_prev.sublabel));
	//SetConsoleTextAttribute(hConsole, 15);
	cv::Mat overlay = image_zed_ocv.clone();
	cv::Point top_left_corner;
	cv::Point top_right_corner;
	cv::Point bottom_right_corner;
	cv::Point bottom_left_corner;
	if (found_prev.id != -10) {
		top_left_corner = resize_coordinates(found_prev.bounding_box_2d[0].x, found_prev.bounding_box_2d[0].y);
		top_right_corner = resize_coordinates(found_prev.bounding_box_2d[1].x, found_prev.bounding_box_2d[1].y);
		bottom_right_corner = resize_coordinates(found_prev.bounding_box_2d[2].x, found_prev.bounding_box_2d[2].y);
		bottom_left_corner = resize_coordinates(found_prev.bounding_box_2d[3].x, found_prev.bounding_box_2d[3].y);
	}
	else {
		top_left_corner = cv::Point(found_prev.bounding_box_2d[0].x, found_prev.bounding_box_2d[0].y);
		top_right_corner = cv::Point(found_prev.bounding_box_2d[1].x, found_prev.bounding_box_2d[1].y);
		bottom_right_corner = cv::Point(found_prev.bounding_box_2d[2].x, found_prev.bounding_box_2d[2].y);
		bottom_left_corner = cv::Point(found_prev.bounding_box_2d[3].x, found_prev.bounding_box_2d[3].y);
	}

	// scaled ROI
	cv::Rect roi(top_left_corner, bottom_right_corner);
	if (roi.x >= 0 && roi.y >= 0 && roi.width + roi.x < image_zed_ocv.cols && roi.height + roi.y < image_zed_ocv.rows) {
		if (change) overlay(roi).setTo(cv::Scalar(0, 165, 255, 255));
		else overlay(roi).setTo(cv::Scalar(34, 139, 34, 255));
		cv::addWeighted(image_zed_ocv, 0.5, overlay, 0.5, 0.0, image_zed_ocv);
		cv::Point below = bottom_left_corner + cv::Point(0, 15);
		if (change)	cv::putText(image_zed_ocv, "CHANGE", below, cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 165, 255, 255), 1.5);
		else cv::putText(image_zed_ocv, "MATCH", below, cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(34, 139, 34, 255), 1.5);
	}
}