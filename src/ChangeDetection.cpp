#include "ChangeDetection.hpp"
#include <charconv>
#include <vector>
#include <string>

using boost::property_tree::ptree;
using boost::property_tree::xml_writer_settings;
void save_data_association_result(ChangeDetector::DetectedObject& DetectedObject, std::string writePath, ptree& object);
ChangeDetector::ZEDParameters ZEDParameter;

/**
 * This function is a helper for adding 2 pointclouds together
 * input1: point1 
 * input2: point2
 * return: a boolean value corresponding to the relationship of the 2 inputs
 **/
bool comparePoint(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {
    if (p1.x != p2.x)
        return p1.x > p2.x;
    else if (p1.y != p2.y)
        return  p1.y > p2.y;
    else
        return p1.z > p2.z;
}

/**
 * This function is a helper for adding 2 pointclouds together
 * input1: point1
 * input2: point2
 * return: a boolean value for deciding whether 2 point1 and point2 are the same or not
 **/
bool equalPoint(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {
    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        return true;
    return false;
}

/**
 * This function convert a RGBA color packed into a packed RGB PCL compatible format
 * input: ZED rgba float data
 * output: PCL compatible rgb data
 **/
float ChangeDetector::convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t*)&colorIn;
    unsigned char* color_uchar = (unsigned char*)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float*> (&color_uint);
}

/**
 * This function converts the pointcloud measured by ZED to a PCL compatible format
 * input1: pointcloud measured by ZED
 * input2: the pcl pointcloud where the result of the conversion will be saved
 **/
void ChangeDetector::measurement_to_pcl(sl::Mat measurement, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud) {
    p_pcl_point_cloud->points.resize(measurement.getHeight() * measurement.getWidth());
    int index = 0;
    for (int w = 0; w < measurement.getWidth(); w++) {
        for (int h = 0; h < measurement.getHeight(); h++) {
            sl::float4 pixel;
            measurement.getValue(w, h, &pixel);
            //if (!isValidMeasure(pixel.x)) {
            //    p_pcl_point_cloud->points[index].x = 0;
            //    p_pcl_point_cloud->points[index].y = 0;
            //    p_pcl_point_cloud->points[index].z = 0;
            //    p_pcl_point_cloud->points[index].rgb = 0;
            //}
            //else {
            if (isValidMeasure(pixel.x)) {
                p_pcl_point_cloud->points[index].x = pixel.x;
                p_pcl_point_cloud->points[index].y = pixel.y;
                p_pcl_point_cloud->points[index].z = pixel.z;
                p_pcl_point_cloud->points[index].rgb = convertColor(pixel[3]);
                index += 1;
            }

        }
    }
    p_pcl_point_cloud->points.resize(index + 1);
}

/**
 * This function shows the detected object bounding boxes on the pcl pointcloud
 * input1: pcl viewer where to the render the bounding boxes
 * input2: the objects detected by ZED
 * input3: bounding box id for pcl to keep track which objects to update/remove
 **/
void ChangeDetector::show_object_detection_on_point_cloud(std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer, sl::Objects objects, int &id) {
    for (int num = 0; num < id; num++) {
        pcl_viewer->removeShape(std::to_string(num));
    }
    if (!objects.object_list.empty()) {
        for (int index = 0; index < objects.object_list.size(); index++) {
            auto bb = objects.object_list[index].bounding_box;
            if (!bb.empty()) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                bounding_box_cloud->points.resize(bb.size());
                for (int pont = 0; pont < bb.size(); pont++) {
                    bounding_box_cloud->points[pont].x = bb[pont].x;
                    bounding_box_cloud->points[pont].y = bb[pont].y;
                    bounding_box_cloud->points[pont].z = bb[pont].z;
                }
                pcl::PointXYZRGB min_point_AABB;
                pcl::PointXYZRGB max_point_AABB;
                pcl::getMinMax3D(*bounding_box_cloud, min_point_AABB, max_point_AABB);
                if (objects.object_list[index].id == -10) {
                    pcl_viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 0, 0, 255, std::to_string(id));
                    pcl_viewer->setRepresentationToWireframeForAllActors();
                }
                else {
                    pcl_viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 255, 0, 0, std::to_string(id));
                    pcl_viewer->setRepresentationToWireframeForAllActors();
                }
                id += 1;
            }
        }
    }
    pcl_viewer->resetCamera();
    pcl_viewer->spinOnce();
}

/**
 * This function shows the segmented pointclouds of the detected objects
 * input1: pcl viewer where to the render the segmented pointclouds
 * input2: the objects detected by ZED
 * input3: pcl pointcloud - where to segment the objects from
 * input4: pcl pointcloud - where to save the segmented pointclouds
 * input5: bounding box id for pcl to keep track which objects to segment and show
 **/
void ChangeDetector::segment_and_show_bounding_box_from_point_cloud(std::shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer, sl::Objects objects,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl, int& id) {
    for (int num = 0; num < id; num++) {
        filter_viewer->removePointCloud(std::to_string(num));
    }
    if (!objects.object_list.empty()) {
        for (int index = 0; index < objects.object_list.size(); index++) {
            auto bb = objects.object_list[index].bounding_box;
            if (!bb.empty()) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                bounding_box_cloud->points.resize(bb.size());
                pcl::PointXYZRGB min_point_AABB;
                pcl::PointXYZRGB max_point_AABB;
                for (int pont = 0; pont < bb.size(); pont++) {
                    bounding_box_cloud->points[pont].x = bb[pont].x;
                    bounding_box_cloud->points[pont].y = bb[pont].y;
                    bounding_box_cloud->points[pont].z = bb[pont].z;
                }
                pcl::getMinMax3D(*bounding_box_cloud, min_point_AABB, max_point_AABB);

                pcl::CropBox<pcl::PointXYZRGB> crop;
                Eigen::Vector4f min_point = Eigen::Vector4f(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z, 0);
                Eigen::Vector4f max_point = Eigen::Vector4f(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z, 0);
                crop.setMin(min_point);
                crop.setMax(max_point);
                crop.setInputCloud(p_pcl_point_cloud);
                crop.filter(*filtered_pcl);
                filter_viewer->addPointCloud(filtered_pcl, std::to_string(id));

                id += 1;
            }
        }
    }
    filter_viewer->spinOnce();
}

/**
 * This function segments an object from a pointcloud based on its bounding box
 * input1: 3D bounding box
 * input2: pcl pointcloud to segment from
 * return: pcl pointcloud to save the segmented object
 **/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ChangeDetector::segment_bounding_box(std::vector<sl::float3> bounding_box_3d, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    bounding_box_cloud->points.resize(bounding_box_3d.size());
    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    for (int pont = 0; pont < bounding_box_3d.size(); pont++) {
        bounding_box_cloud->points[pont].x = bounding_box_3d[pont].x;
        bounding_box_cloud->points[pont].y = bounding_box_3d[pont].y;
        bounding_box_cloud->points[pont].z = bounding_box_3d[pont].z;
    }
    pcl::getMinMax3D(*bounding_box_cloud, min_point_AABB, max_point_AABB);

    pcl::CropBox<pcl::PointXYZRGB> crop;
    Eigen::Vector4f min_point = Eigen::Vector4f(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);
    crop.setInputCloud(p_pcl_point_cloud);
    crop.filter(*filtered_pcl);
    return filtered_pcl;
}

/**
 * This function shows the 2D bounding boxes of detected objects on an OpenCV window
 * input1: detected objects
 * input2: captured image by ZED - draw the bounding boxes on this image
 * input3: top left pixel of object
 * input4: btotom right pixel of object
 **/
void ChangeDetector::show_object_on_image(ChangeDetector::DetectedObject object, cv::Mat& image_zed_ocv, cv::Point top_left, cv::Point bottom_right) {
    auto label = object.label;
    auto confidence = object.confidence;
    auto bounding_box = object.bounding_box_2d;
    std::string cv_text = "Previously detected " + label + " " + std::to_string((int)confidence) + "%";

    cv::rectangle(image_zed_ocv, top_left, bottom_right, cv::Scalar(255, 0, 0));

    cv::putText(image_zed_ocv, cv_text, cv::Point(top_left.x, top_left.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
}

/**
 * This function shows the 2D bounding boxes of detected objects on an OpenCV window
 * input1: detected objects
 * input2: captured image by ZED - draw the bounding boxes on this image
 * input3: detection confidence
 **/
void ChangeDetector::show_object_detection_on_image(sl::Objects objects, cv::Mat image_zed_ocv, int& detection_confidence) {
    if (!objects.object_list.empty()) {
        for (int index = 0; index < objects.object_list.size(); index++) {
            auto label = objects.object_list[index].sublabel;
            auto confidence = objects.object_list[index].confidence;
            auto bounding_box = objects.object_list.at(index).bounding_box_2d;
            std::string cv_text = (std::string)sl::toString(label) + " " + std::to_string((int)confidence) + "%";
            cv::rectangle(image_zed_ocv, resize_coordinates(bounding_box[0].x, bounding_box[0].y),
                resize_coordinates(bounding_box[2].x, bounding_box[2].y), cv::Scalar(255, 0, 0));
            cv::putText(image_zed_ocv, cv_text, resize_coordinates(bounding_box[0].x, bounding_box[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
        }
    }
    cv::createTrackbar("Confidence", "ZED View", &detection_confidence, 100);
    cv::imshow("ZED View", image_zed_ocv);
    cv::waitKey(1);
}

/**
 * This function resizes the bounding box coordinates of detected objects to fit the desired resolution of the display window
 * input1: x coordinate
 * input2: y coordinate
 * return: resized (x, y) coordinates
 **/
cv::Point ChangeDetector::resize_coordinates(int x, int y) {
    float x_new = (float)((float)x * ZEDParameter.ratio_width);
    float y_new = (float)((float)y * ZEDParameter.ratio_height);
    return cv::Point(int(x_new), int(y_new));
}

/**
 *  This function creates a PCL visualizer
 **/
std::shared_ptr<pcl::visualization::PCLVisualizer> ChangeDetector::createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

/**
 * This function does a KdTree-search to find the nearest neighbor of each point of 'pcl_sample' in 'pcl_ref'
 * input1: reference pointcloud, where we look for the closest neigbours
 * input2: sample pointcloud, whose closest neighbours we want to find
 * input3: distance threshold
 * return: percentage of point-pairs, whose distance is below the distance threshold
 **/
float ChangeDetector::knn_search(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ref, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_sample, int distance) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ref_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_sample_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pcl_ref, *pcl_ref_xyz);
    pcl::copyPointCloud(*pcl_sample, *pcl_sample_xyz);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pcl_ref_xyz);
    int K = 1; // K nearest neighbor search
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    pcl::PointXYZ searchPoint;
    int within_distance_num = 0;
    for (int nIndex = 0; nIndex < pcl_sample_xyz->points.size(); nIndex++)
    {
        searchPoint.x = pcl_sample_xyz->points[nIndex].x;
        searchPoint.y = pcl_sample_xyz->points[nIndex].y;
        searchPoint.z = pcl_sample_xyz->points[nIndex].z;
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
                //std::cout << "    " << (*proba_filter1)[pointIdxNKNSearch[i]].x
                //<< " " << (*proba_filter1)[pointIdxNKNSearch[i]].y
                //<< " " << (*proba_filter1)[pointIdxNKNSearch[i]].z
                //<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
                if (pointNKNSquaredDistance[i] < distance) {
                    within_distance_num += 1;
                }
            }
        }
    }
    float percentage = (float)within_distance_num / pcl_sample_xyz->points.size();
    return percentage;
}

/**
 * This function registers the object detected by zed according to the change detection object structure
 * input1: object detected by ZED
 * input2: change detection object structure instance
 * input3: currently measured pointcloud
 * input4: verbosity level for debugging
 **/
void ChangeDetector::registerNewObject(sl::ObjectData zedObject, ChangeDetector::DetectedObject & newDetectedObject, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, bool verbose) {
    newDetectedObject.label = (std::string)sl::toString(zedObject.sublabel);
    newDetectedObject.tracking_id = zedObject.id;
    newDetectedObject.position = zedObject.position;
    newDetectedObject.confidence = (int)zedObject.confidence;
    newDetectedObject.bounding_box_2d = zedObject.bounding_box_2d;
    newDetectedObject.bounding_box_3d = zedObject.bounding_box;
    newDetectedObject.label_confidence_pair.insert(std::make_pair(newDetectedObject.label, newDetectedObject.confidence));
    newDetectedObject.label_detection_num_pair.insert(std::make_pair(newDetectedObject.label, 1));
    newDetectedObject.overall_detection_num = 1;
    newDetectedObject.ismoving = (zedObject.action_state == sl::OBJECT_ACTION_STATE::MOVING) ? true : false;
    newDetectedObject.bounding_box_3d_sum = zedObject.bounding_box;
    if (!newDetectedObject.bounding_box_3d.empty()) {
        newDetectedObject.has_object_3d_pointcloud = true;
        newDetectedObject.object_3d_pointcloud = segment_bounding_box(newDetectedObject.bounding_box_3d, p_pcl_point_cloud);
        if (verbose) {
            printf("label: %s\n", newDetectedObject.label);
            printf("position: %f %f %f\n", newDetectedObject.position[0], newDetectedObject.position[1], newDetectedObject.position[2]);
            printf("confidence: %d\n", newDetectedObject.confidence);
        }
    }
    else {
        newDetectedObject.has_object_3d_pointcloud = false;
        if (verbose) std::cout << "NO 3D BOUNDING BOX AVAILABLE" << std::endl;
    }
}

/**
 * This function handles the data association of consecutive object detections
 * input1: pointcloud to segment the objects from
 * input2: new detected objects
 * input3: already detected, stored objects
 * input4: eucledian distance threshold between the object point cloud centroids
 * input5: kd nearest neighbour search treshold
 * input6: verbosity level for debugging
 **/
void ChangeDetector::data_association_of_detected_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, sl::Objects objects,
    std::vector<ChangeDetector::DetectedObject>& DetectedObjects, int eucl_dist, int kd_dis, bool verbose = false) {
    HANDLE  hConsole;
    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    int track_index = -1;
    bool should_add = true;
    if (!objects.object_list.empty()) {
        for (int index = 0; index < objects.object_list.size(); index++) {
            //auto now = std::chrono::high_resolution_clock::now();
            if (objects.object_list[index].id == -10) continue; // don't save one of the previously detected objects (those have an id=-10)
            ChangeDetector::DetectedObject newDetectedObject;
            registerNewObject(objects.object_list[index], newDetectedObject, p_pcl_point_cloud, verbose);

            if (newDetectedObject.has_object_3d_pointcloud == true && newDetectedObject.object_3d_pointcloud->points.size() > 100 && newDetectedObject.ismoving == false) {
                if (DetectedObjects.size() > 0) {
                    auto ids = return_closest_objects<ChangeDetector::DetectedObject>(DetectedObjects, newDetectedObject, eucl_dist, verbose); // distance of bounding box centroids
                    //std::string writePath = "D:/zed codes/zed_change_pcl/build/obj_pcls/" + newDetectedObject.label + "_" + std::to_string(DetectedObjects.size()) + ".ply";
                    //pcl::io::savePLYFileBinary(writePath, *newDetectedObject.object_3d_pointcloud);

                    int track = newDetectedObject.tracking_id;
                    auto talalt_it = std::find_if(DetectedObjects.begin(), DetectedObjects.end(), [&track](const ChangeDetector::DetectedObject& obj) {return obj.tracking_id == track; });
                    if (talalt_it != DetectedObjects.end()) track_index = std::distance(DetectedObjects.begin(), talalt_it);

                    if (ids.size() == 0 && talalt_it == DetectedObjects.end()) {
                        SetConsoleTextAttribute(hConsole, 2);
                        printf("Added new object %s with id [%d], because there were no close objects.\n", newDetectedObject.label, newDetectedObject.tracking_id);
                        SetConsoleTextAttribute(hConsole, 15);
                        DetectedObjects.push_back(newDetectedObject);
                    }
                    else {
                        if (ids.empty()) ids.push_back(track_index);
                        if (verbose) std::cout << "Close objects: " << std::endl;
                        should_add = true;
                        for (int i = 0; i < ids.size(); i++) {
                            if (verbose) std::cout << DetectedObjects[ids[i]].label << " " << DetectedObjects[ids[i]].tracking_id << " ";
                            float percentage = knn_search(DetectedObjects[ids[i]].object_3d_pointcloud, newDetectedObject.object_3d_pointcloud, kd_dis); // squared distance of neighbourpoints
                            if (verbose) printf("Percentage of points within %d distance: %f\n", kd_dis, percentage);
                            if (percentage > 0.5 || newDetectedObject.tracking_id == DetectedObjects[ids[i]].tracking_id) {
                                std::map<std::string, int>::iterator it = DetectedObjects[ids[i]].label_confidence_pair.find(newDetectedObject.label);
                                // key already present in the map
                                if (it != DetectedObjects[ids[i]].label_confidence_pair.end()) {
                                    it->second += newDetectedObject.confidence;    // increment map's value for key newDetectedObject.label
                                    std::map<std::string, int>::iterator it2 = DetectedObjects[ids[i]].label_detection_num_pair.find(newDetectedObject.label);
                                    it2->second += 1;
                                }
                                // key not found
                                else {
                                    DetectedObjects[ids[i]].label_confidence_pair.insert(std::make_pair(newDetectedObject.label, newDetectedObject.confidence));
                                    DetectedObjects[ids[i]].label_detection_num_pair.insert(std::make_pair(newDetectedObject.label, 1));
                                }

                                DetectedObjects[ids[i]].overall_detection_num += 1;
                                // updating bounding box coordinates
                                for (int pont = 0; pont < DetectedObjects[ids[i]].bounding_box_3d_sum.size(); pont++) {
                                    DetectedObjects[ids[i]].bounding_box_3d_sum[pont].x += newDetectedObject.bounding_box_3d_sum[pont].x;
                                    DetectedObjects[ids[i]].bounding_box_3d_sum[pont].y += newDetectedObject.bounding_box_3d_sum[pont].y;
                                    DetectedObjects[ids[i]].bounding_box_3d_sum[pont].z += newDetectedObject.bounding_box_3d_sum[pont].z;

                                    DetectedObjects[ids[i]].bounding_box_3d[pont] = DetectedObjects[ids[i]].bounding_box_3d_sum[pont] / DetectedObjects[ids[i]].overall_detection_num;
                                }

                                // Updating object point cloud
                                auto current_pcl_size = DetectedObjects[ids[i]].object_3d_pointcloud->points.size();
                                auto new_pcl_size = newDetectedObject.object_3d_pointcloud->points.size();
                                if (new_pcl_size > current_pcl_size) {
                                    DetectedObjects[ids[i]].object_3d_pointcloud = newDetectedObject.object_3d_pointcloud;
                                }
                                // NAGYON LASSÚ

                                //DetectedObjects[ids[i]].object_3d_pointcloud->points.resize(current_pcl_size + newDetectedObject.object_3d_pointcloud->points.size());
                                //for (int nIndex = 0; nIndex < newDetectedObject.object_3d_pointcloud->points.size(); nIndex++){
                                //    int bigIndex = current_pcl_size + nIndex;
                                //    DetectedObjects[ids[i]].object_3d_pointcloud->points[bigIndex] = newDetectedObject.object_3d_pointcloud->points[nIndex];
                                //}
                                //std::sort(DetectedObjects[ids[i]].object_3d_pointcloud->points.begin(), DetectedObjects[ids[i]].object_3d_pointcloud->points.end(), comparePoint);
                                //auto unique_end = std::unique(DetectedObjects[ids[i]].object_3d_pointcloud->points.begin(), DetectedObjects[ids[i]].object_3d_pointcloud->points.end(), equalPoint);
                                //DetectedObjects[ids[i]].object_3d_pointcloud->points.erase(unique_end, DetectedObjects[ids[i]].object_3d_pointcloud->points.end());

                                SetConsoleTextAttribute(hConsole, 3);
                                printf("Updated already existing object %s with id [%d].\n", DetectedObjects[ids[i]].label, DetectedObjects[ids[i]].tracking_id);
                                if (newDetectedObject.tracking_id == DetectedObjects[ids[i]].tracking_id) {
                                    printf("(because they had the same tracking id)\n");
                                SetConsoleTextAttribute(hConsole, 15);

                                }
                                for (auto& e : DetectedObjects[ids[i]].label_confidence_pair) {
                                    std::map<std::string, int>::iterator it3 = DetectedObjects[ids[i]].label_detection_num_pair.find(e.first);
                                    if (verbose) std::cout << '{' << e.first << ", " << e.second/it3->second << ", " <<  it3->second << '}' << '\n';
                                }
                                should_add = false;
                                break;
                            }
                        }
                        if (should_add) {

                            SetConsoleTextAttribute(hConsole, 4);
                            printf("Added new object %s with id [%d], because the nearby objects' pointclouds differ too much.\n", newDetectedObject.label, newDetectedObject.tracking_id);
                            SetConsoleTextAttribute(hConsole, 15);

                            DetectedObjects.push_back(newDetectedObject);
                        }
                    }
                }
                else {
                    SetConsoleTextAttribute(hConsole, 2);
                    printf("Added new object %s with id [%d], because it is the first detected object.\n", newDetectedObject.label, newDetectedObject.tracking_id);
                    SetConsoleTextAttribute(hConsole, 15);
                    DetectedObjects.push_back(newDetectedObject);
                }
            }
            //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - now).count();
            //std::cout << duration << std::endl;
        }
    }
}

/**
 * This function finalizes the saved class labels and confidences -> assigns one label and confidence if there has been more linked to an object
 * input1: already detected, stored objects
 **/
void ChangeDetector::class_label_and_confidence_decision(std::vector<ChangeDetector::DetectedObject>& DetectedObjects) {
    for (int i = DetectedObjects.size() - 1; i >= 0; i--) {
        if (DetectedObjects[i].overall_detection_num < 6) {
            DetectedObjects.erase(DetectedObjects.begin() + i);
            continue;
        }
        int max_num = 0;
        std::string max_label;
        int det_num = 0;
        for (auto& e : DetectedObjects[i].label_confidence_pair) {
            if (e.second > max_num) {
                max_num = e.second;
                max_label = e.first;
                std::map<std::string, int>::iterator it3 = DetectedObjects[i].label_detection_num_pair.find(e.first);
                det_num = it3->second;
            }
        }
        DetectedObjects[i].label = max_label;
        DetectedObjects[i].confidence = max_num / det_num;
        DetectedObjects[i].detection_num = det_num;
    }
}

/**
 * This function visualizes the object-map on the fused-pointcloud created by ZED
 * input1: path to the fused pointcloud
 * input2: already detected, stored objects
 **/
void ChangeDetector::save_and_visualize_end_result(std::string input_pointcloud_path, std::vector<ChangeDetector::DetectedObject>& DetectedObjects) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile(input_pointcloud_path, *final_output);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_final(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the PCL point cloud visualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_final_viewer = createRGBVisualizer(final_output);
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*final_output, centroid);
    pcl_final_viewer->setCameraPosition(centroid[0], centroid[1], centroid[2], 0, 0, 1, 0, 1, 0);
    //pcl_final_viewer->setCameraClipDistances(0.1, 1000);
    std::ofstream file("data_association.xml");
    boost::property_tree::ptree tree;
    ptree& input_pcl = tree.add("library.inputPointcloud", "");
    input_pcl.add("path", input_pointcloud_path);

    for (auto detectedObject = DetectedObjects.begin(); detectedObject != DetectedObjects.end(); detectedObject++) {
        if (detectedObject->detection_num > 5)
        {
            segment_final = segment_bounding_box(detectedObject->bounding_box_3d, final_output);
            if (segment_final->size() > 200) {
                std::string writePath = "D:/zed codes/zed_change_pcl/build/final_objects/" + detectedObject->label + "_" + std::to_string(detectedObject->tracking_id) + ".ply";
                pcl::io::savePLYFileBinary(writePath, *segment_final);

                save_data_association_result(*detectedObject, writePath, tree);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                bounding_box_cloud->points.resize(detectedObject->bounding_box_3d.size());
                for (int pont = 0; pont < detectedObject->bounding_box_3d.size(); pont++) {
                    bounding_box_cloud->points[pont].x = detectedObject->bounding_box_3d[pont].x;
                    bounding_box_cloud->points[pont].y = detectedObject->bounding_box_3d[pont].y;
                    bounding_box_cloud->points[pont].z = detectedObject->bounding_box_3d[pont].z;
                }
                pcl::PointXYZRGB min_point_AABB;
                pcl::PointXYZRGB max_point_AABB;
                pcl::getMinMax3D(*bounding_box_cloud, min_point_AABB, max_point_AABB);
                pcl_final_viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 255, 0, 0, std::to_string(detectedObject->tracking_id));
                pcl_final_viewer->addText3D(detectedObject->label + "_" + std::to_string(detectedObject->tracking_id), detectedObject->position, 50, 255, 0, 0, detectedObject->label + "_" + std::to_string(detectedObject->tracking_id), 0);
                pcl_final_viewer->setRepresentationToWireframeForAllActors();
            }
            else DetectedObjects.erase(detectedObject);
        }
        else DetectedObjects.erase(detectedObject);
    }

    boost::property_tree::write_xml(
        file, tree,
        boost::property_tree::xml_writer_make_settings<std::string>('\t', 1));

    while (!pcl_final_viewer->wasStopped())
    {
        pcl_final_viewer->spinOnce();
    }
}

/**
 * This function saved the final object map into an xml file
 * input1: detected objects
 * input2: filepath where the xml file will be saved
 * input3: boost tree object
 **/
void save_data_association_result(ChangeDetector::DetectedObject& DetectedObject, std::string writePath, ptree& tree) {
    ptree& object = tree.add("library.objects.object", "");
    object.add("label", DetectedObject.label);
    object.add("<xmlattr>.id", DetectedObject.tracking_id);
    object.add("confidence", DetectedObject.confidence);
    object.add("numberOfDetections", DetectedObject.overall_detection_num);
    object.add("position", DetectedObject.position);
    object.add("has3DPointCloud", DetectedObject.has_object_3d_pointcloud);
    object.add("pathTo3DPointCloud", writePath);
    std::stringstream result;
    std::vector<int> bb_out;
    for (int coord = 0; coord < DetectedObject.bounding_box_2d.size(); coord++) {
        bb_out.push_back(DetectedObject.bounding_box_2d[coord].x);
        bb_out.push_back(DetectedObject.bounding_box_2d[coord].y);
    }

    std::copy(bb_out.begin(), bb_out.end(), std::ostream_iterator<int>(result, " "));
    object.add("2DBoundingBox", result.str());
    std::vector<int> bb3_out;
    std::stringstream result2;
    for (int coord = 0; coord < DetectedObject.bounding_box_3d.size(); coord++) {
        bb3_out.push_back(DetectedObject.bounding_box_3d[coord].x);
        bb3_out.push_back(DetectedObject.bounding_box_3d[coord].y);
        bb3_out.push_back(DetectedObject.bounding_box_3d[coord].z);
    }
    std::copy(bb3_out.begin(), bb3_out.end(), std::ostream_iterator<int>(result2, " "));
    object.add("3DBoundingBox", result2.str());
}

/**
 * This function reads the previously saved object map from an xml file
 * input1: filepath where the xml file was saved
 * input2: where to store the previously detected objects after reading the file
 **/
void ChangeDetector::read_previously_saved_detected_objects(std::string saved_xml_file_path, std::vector<ChangeDetector::DetectedObject>& PreviouslyDetectedObjects) {
    boost::property_tree::ptree pt1;
    boost::property_tree::read_xml(saved_xml_file_path, pt1);
    // Traverse property tree
    BOOST_FOREACH(boost::property_tree::ptree::value_type const& node, pt1.get_child("library.objects"))
    {
        boost::property_tree::ptree subtree = node.second;
        if (node.first == "object")
        {
            ChangeDetector::DetectedObject PreviouslyDetectedObject;
            PreviouslyDetectedObject.tracking_id = subtree.get<int>("<xmlattr>.id");
            subtree.get_child("");
            PreviouslyDetectedObject.label = subtree.get<std::string>("label");
            PreviouslyDetectedObject.confidence = subtree.get<int>("confidence");
            PreviouslyDetectedObject.overall_detection_num = subtree.get<int>("numberOfDetections");
            std::string xml_posi = subtree.get<std::string>("position");
            std::vector<std::string> numbers;
            boost::algorithm::split(numbers, xml_posi, boost::algorithm::is_any_of(" "));
            for (int i = 0; i < numbers.size(); i++) {
                PreviouslyDetectedObject.position[i] = atof(numbers[i].c_str());
            }
            PreviouslyDetectedObject.has_object_3d_pointcloud = subtree.get<bool>("has3DPointCloud");
            PreviouslyDetectedObject.path_to_pointcloud = subtree.get<std::string>("pathTo3DPointCloud");
            std::string xml_2d_bb = subtree.get<std::string>("2DBoundingBox");
            std::vector<std::string> numbers2;
            boost::algorithm::split(numbers2, xml_2d_bb, boost::algorithm::is_any_of(" "));
            for (int i = 0, j = 0; i < 4; i++, j += 2) {
                PreviouslyDetectedObject.bounding_box_2d.push_back({ stoul(numbers2[j]), stoul(numbers2[j + 1]) });
            }
            std::string xml_3d_bb = subtree.get<std::string>("3DBoundingBox");
            std::vector<std::string> numbers3;
            boost::algorithm::split(numbers3, xml_3d_bb, boost::algorithm::is_any_of(" "));
            for (int i = 0, j = 0; i < 8; i++, j += 3) {
                PreviouslyDetectedObject.bounding_box_3d.push_back({ stof(numbers3[j]), stof(numbers3[j + 1]), stof(numbers3[j + 2]) });
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::io::loadPLYFile(PreviouslyDetectedObject.path_to_pointcloud, *load_pcl);
            PreviouslyDetectedObject.object_3d_pointcloud = load_pcl;

            PreviouslyDetectedObjects.push_back(PreviouslyDetectedObject);
        }
    }
}

/**
 * This function transforms an object's world coordinates to the current camera coordinates
 * input1: current 3d position of object wrt world frame
 * input2: current camera pose wrt world frame
 * returns new position that is wrt camera frame
 **/
sl::Translation ChangeDetector::transform_p_world_to_p_cam(sl::Translation current_pos,  sl::Pose cam_pose) {
    auto cam_to_world = cam_pose;
    cam_to_world.pose_data.inverse();
    sl::Translation new_pos = current_pos * cam_to_world.getOrientation() + cam_to_world.getTranslation();
    return new_pos;
}

/**
 * This function maps a 3d point (wrt to camera frame) to image pixels
 * input1: current 3d position of object wrt camera frame
 * returns 2d pixel value (x,y) mapped from the 3d point
 **/
cv::Point ChangeDetector::_3d_point_to_2d_pixel(sl::Translation new_position) {
    int pixelX = ZEDParameter.calib_param_.cx - (new_position.x * ZEDParameter.calib_param_.fx) / new_position.z;
    int pixelY = ZEDParameter.calib_param_.cy + (new_position.y * ZEDParameter.calib_param_.fy) / new_position.z;
    return cv::Point(pixelX, pixelY);
}

/**
 * This function queries the previously detected objects and reprojects them onto the new run's image 
 * input1: the previously detected object
 * input2: opencv image
 * input3: camera pose
 * input4: the pixel(x,y) which is the reprojection of the object's 3d location onto the 2d image plane
 **/
void ChangeDetector::find_and_reproject_previous_detections_onto_image(ChangeDetector::DetectedObject& prev_obj, cv::Mat& image_zed_ocv, sl::Pose cam_pose, cv::Point Pixel) {

    cv::Point resized_pix = resize_coordinates(Pixel.x, Pixel.y);
    cv::Point top_left = cv::Point(ZEDParameter.display_resolution.width, ZEDParameter.display_resolution.height);
    cv::Point top_right = cv::Point(0, ZEDParameter.display_resolution.height);
    cv::Point bottom_right = cv::Point(0, 0);
    cv::Point bottom_left = cv::Point(ZEDParameter.display_resolution.width, 0);
    // Transform 3d points to 2d pixels and draw them onto 2D image
    for (auto points_ : prev_obj.object_3d_pointcloud->points) {
        sl::Translation point_tr = { points_.x, points_.y, points_.z };
        sl::Translation new_position = transform_p_world_to_p_cam(point_tr, cam_pose);
        cv::Point Pixel_ = _3d_point_to_2d_pixel(new_position);
        auto new_pixel = resize_coordinates(Pixel_.x, Pixel_.y);
        if (new_pixel.x < image_zed_ocv.cols && new_pixel.x > 0 && new_pixel.y < image_zed_ocv.rows && new_pixel.y > 0) {
            if (new_pixel.x <= top_left.x) top_left.x = new_pixel.x;
            if (new_pixel.y <= top_left.y) top_left.y = new_pixel. y;
            if (new_pixel.x >= bottom_right.x) bottom_right.x = new_pixel.x;
            if (new_pixel.y >= bottom_right.y) bottom_right.y = new_pixel.y;
            if (new_pixel.x >= top_right.x) top_right.x = new_pixel.x;
            if (new_pixel.y <= top_right.y) top_right.y = new_pixel.y;
            if (new_pixel.x <= bottom_left.x) bottom_left.x = new_pixel.x;
            if (new_pixel.y >= bottom_left.y) bottom_left.y = new_pixel.y;
            image_zed_ocv.at<cv::Vec4b>(new_pixel.y, new_pixel.x)[0] = points_.b;
            image_zed_ocv.at<cv::Vec4b>(new_pixel.y, new_pixel.x)[1] = points_.g;
            image_zed_ocv.at<cv::Vec4b>(new_pixel.y, new_pixel.x)[2] = points_.r;
            image_zed_ocv.at<cv::Vec4b>(new_pixel.y, new_pixel.x)[3] = points_.a;
        }
    }

    // Draw bounding box around the previously detected object
    show_object_on_image(prev_obj, image_zed_ocv, top_left, bottom_right);

    sl::uint2 A = { (unsigned int)top_left.x, (unsigned int)top_left.y };
    sl::uint2 B = { (unsigned int)top_right.x, (unsigned int)top_right.y };
    sl::uint2 C = { (unsigned int)bottom_right.x, (unsigned int)bottom_right.y };
    sl::uint2 D = { (unsigned int)bottom_left.x, (unsigned int)bottom_left.y };
    prev_obj.bounding_box_2d = { A, B, C, D };
}

/**
 * This function adds a previously detected object to the list of currently detected objects if that object is visible in the new run 
 * input1: the previously detected object
 * input2: camera pose
 * input3: new, detected list of objects
 * input4: the Pixel which is the reprojection of the object's 3d location onto the 2d image plane
 **/
sl::ObjectData ChangeDetector::add_previous_detections_to_sl_objects(ChangeDetector::DetectedObject prev_obj, sl::Pose cam_pose, std::vector<sl::ObjectData>& object_list, cv::Point Pixel) {

    // Convert into sl::Object
    sl::ObjectData prev_Obj;
    auto it = get_sl_subclass.find(prev_obj.label);
    prev_Obj.label = sl::getObjectClass(it->second);
    prev_Obj.sublabel = it->second;
    prev_Obj.id = -10;
    prev_Obj.tracking_state = sl::OBJECT_TRACKING_STATE::OK;
    prev_Obj.action_state = sl::OBJECT_ACTION_STATE::IDLE;
    prev_Obj.position = prev_obj.position;
    prev_Obj.bounding_box_2d = prev_obj.bounding_box_2d;
    prev_Obj.bounding_box = prev_obj.bounding_box_3d;
    prev_Obj.confidence = (float)prev_obj.confidence;
    object_list.push_back(prev_Obj);

    return prev_Obj;
}

/**
 * This function compares the previously and the currently detected objects and decides if they are a match or if there has been a change
 * input1: min point of the pointcloud
 * input2: max point of the point cloud
 * input3: previously detected vector of objects
 * input4: camera pose
 * input5: opencv image, on which the results are displayed
 * input6: currently detected list of sl::objects
 * input7: currently detected vector of objects 
 **/
void ChangeDetector::compare_for_change(pcl::PointXYZRGB min_, pcl::PointXYZRGB max_, std::vector<ChangeDetector::DetectedObject>& PreviouslyDetectedObjects, sl::Pose cam_pose,
    cv::Mat& image_zed_ocv, std::vector<sl::ObjectData>& object_list, std::vector<ChangeDetector::DetectedObject>& DetectedObjects) {
    // Querying the PREVIOUS DETECTIONS to see if they are visible in the current state and comparing them with the current detections
    for (auto prev_obj : PreviouslyDetectedObjects) {
        auto posi_ = prev_obj.position;
        if ((posi_.x < max_.x && posi_.x > min_.x) && (posi_.y < max_.y && posi_.y > min_.y) && (posi_.z < max_.z && posi_.z > min_.z)) {
            sl::Translation new_position = transform_p_world_to_p_cam(posi_, cam_pose);
            if (new_position.z > 0 || new_position.z < (-1) * (ZEDParameter.init_parameters.depth_maximum_distance)) continue;
            cv::Point Pixel = _3d_point_to_2d_pixel(new_position);
            if (Pixel.x < 0 || Pixel.y < 0 || Pixel.x >= ZEDParameter.calib_param_.image_size.width || Pixel.y >= ZEDParameter.calib_param_.image_size.height) continue;
            // Displaying previous detections on the new run's IMAGE
            find_and_reproject_previous_detections_onto_image(prev_obj, image_zed_ocv, cam_pose, Pixel);
            // Adding previous detections to the object list and displaying them later on the new run's POINTCLOUD
            sl::ObjectData found_prev = add_previous_detections_to_sl_objects(prev_obj, cam_pose, object_list, Pixel);
            if (DetectedObjects.size() > 0) {
                auto close_ids = return_closest_objects<sl::ObjectData>(DetectedObjects, found_prev, 1500, false); // distance of bounding box centroids
                if (close_ids.size() == 0) { // IF A PREVIOUSLY DETECTED OBJECT IS NOT FOUND ANYMORE
                    display_change_or_no_change_of_object<sl::ObjectData>(image_zed_ocv, found_prev, true);
                }
                else {
                    for (int talalt_id = 0; talalt_id < close_ids.size(); talalt_id++) {
                        float percentage = knn_search(DetectedObjects[close_ids[talalt_id]].object_3d_pointcloud, prev_obj.object_3d_pointcloud, 10000); // squared distance of neighbourpoints
                        if (percentage > 0.4) {
                            // SAME OBJECT
                            display_change_or_no_change_of_object<sl::ObjectData>(image_zed_ocv, found_prev, false);
                        }
                        else {
                            // NOT THE SAME OBJECT
                            display_change_or_no_change_of_object<sl::ObjectData>(image_zed_ocv, found_prev, true);
                        }
                    }
                }
            }
            else { // IF A PREVIOUSLY DETECTED OBJECT IS NOT FOUND ANYMORE
                display_change_or_no_change_of_object<sl::ObjectData>(image_zed_ocv, found_prev, true);
            }
        }
    }
    // Querying the CURRENT DETECTIONS to see if they match the previous detections or not
    if (DetectedObjects.size() > 0) {
        for (auto detected_obj : object_list) {
            if (!std::isfinite(detected_obj.position.z)) continue;
            auto ids = return_closest_objects<sl::ObjectData>(PreviouslyDetectedObjects, detected_obj, 1000, false); // distance of bounding box centroids
            if (ids.size() == 0) { // IF A NEWLY DETECTED OBJECT WAS NOT THERE IN THE PREVIOUS RUN
                display_change_or_no_change_of_object<sl::ObjectData>(image_zed_ocv, detected_obj, true);
            }
        }
    }
}

/**
 * This function sets the initial parameters of the project
 * input1: initial parameters of the camera
 * input2: calibration parameters of the camera
 * input3: display resolution of the image output
 * input4: width ratio between camera and image output resolution
 * input5: height ratio between camera and image output resolution
 **/
 void ChangeDetector::setZedParameters(sl::InitParameters init_parameters, sl::CameraParameters calib_param_, sl::Resolution display_resolution, float ratio_width, float ratio_height) {
    ZEDParameter.init_parameters = init_parameters;
    ZEDParameter.calib_param_ = calib_param_;
    ZEDParameter.display_resolution = display_resolution;
    ZEDParameter.ratio_width = ratio_width;
    ZEDParameter.ratio_height = ratio_height;
}