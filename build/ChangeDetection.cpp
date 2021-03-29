#include "ChangeDetection.hpp"

bool sortbysec(std::tuple<int, float>& a, std::tuple<int, float>& b);

/**
 * This function convert a RGBA color packed into a packed RGBA PCL compatible format
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
            if (!isValidMeasure(pixel.x)) {
                p_pcl_point_cloud->points[index].x = 0;
                p_pcl_point_cloud->points[index].y = 0;
                p_pcl_point_cloud->points[index].z = 0;
                p_pcl_point_cloud->points[index].rgb = 0;
            }
            else {
                p_pcl_point_cloud->points[index].x = pixel.x;
                p_pcl_point_cloud->points[index].y = pixel.y;
                p_pcl_point_cloud->points[index].z = pixel.z;
                p_pcl_point_cloud->points[index].rgb = convertColor(pixel[3]);
            }
            index += 1;
        }
    }
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
                pcl_viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 255, 0, 0, std::to_string(id));
                pcl_viewer->setRepresentationToWireframeForAllActors();
                id += 1;
            }
        }
    }
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
 * input3: pcl pointcloud to save the segmented object
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
 * input3: resolution of the desired window
 * input4: resolution of the camera and thus the image
 **/
void ChangeDetector::show_object_detection_on_image(sl::Objects objects, cv::Mat image_zed_ocv, sl::Resolution display_resolution, sl::Resolution camera_resolution) {
    if (!objects.object_list.empty()) {
        for (int index = 0; index < objects.object_list.size(); index++) {
            auto label = objects.object_list[index].sublabel;
            auto confidence = objects.object_list[index].confidence;
            auto bounding_box = objects.object_list.at(index).bounding_box_2d;
            std::string cv_text = (std::string)sl::toString(label) + " " + std::to_string((int)confidence) + "%";
            cv::rectangle(image_zed_ocv, resize_boundingbox_coordinates(bounding_box[0].x, bounding_box[0].y, display_resolution, camera_resolution),
                resize_boundingbox_coordinates(bounding_box[2].x, bounding_box[2].y, display_resolution, camera_resolution), cv::Scalar(255, 0, 0));
            cv::putText(image_zed_ocv, cv_text, resize_boundingbox_coordinates(bounding_box[0].x, bounding_box[0].y - 10, display_resolution, camera_resolution), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
        }
    }
    cv::imshow("ZED View", image_zed_ocv);
    cv::waitKey(15);
}

/**
 * This function resizes the bounding box coordinates of detected objects to fit the desired resolution of the display window
 * input1: x coordinate of a bounding box corner
 * input2: y coordinate of a bounding box corner
 * input3: resolution of the desired window
 * input4: resolution of the camera and thus the image
 * return: resized (x, y) coordinates
 **/
cv::Point ChangeDetector::resize_boundingbox_coordinates(int x, int y, sl::Resolution display_resolution, sl::Resolution camera_resolution) {
    float ratio_width = (float)((float)display_resolution.width / camera_resolution.width);
    float ratio_height = (float)((float)display_resolution.height / camera_resolution.height);
    float x_new = (float)((float)x * ratio_width);
    float y_new = (float)((float)y * ratio_height);
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
 * input1: reference pointcloud, where we look for the closes neigbor
 * input2: sample pointcloud, whose closest neighbors we want to find
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
 * This function returns those objects' ids whose centroids' Eucledian distance is below the 'centroid_distance' threshold
 * input1: list of detected objects
 * input2: new detected object
 * input3: centroid distance threshold
 * return: ids of the closest previously detected objects
 **/
std::vector<int> ChangeDetector::return_closest_objects(std::vector<ChangeDetector::DetectedObject> DetectedObjects, ChangeDetector::DetectedObject newDetectedObject, int centroid_distance, bool verbose = false) {
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
 * This is a comparison function to sort the vector elements by second element of tuples
 * input1: first tuple to compare
 * input2: second tuple to compare
 * return: True if a < b, otherwise False
 **/
bool sortbysec(std::tuple<int, float>& a, std::tuple<int, float>& b) {
    return (std::get<1>(a) < std::get<1>(b));
}

/**
 * This function handles the data association of consecutive object detections
 * input1: pointcloud to segment the objects from
 * input2: new detected objects
 * input3: already detected, stored objects
 **/
void ChangeDetector::data_association_of_detected_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, sl::Objects objects,
    std::vector<ChangeDetector::DetectedObject>& DetectedObjects, int eucl_dist, int kd_dis, bool verbose = false) {
    HANDLE  hConsole;
    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

    bool should_print = true;
    if (!objects.object_list.empty()) {
        for (int index = 0; index < objects.object_list.size(); index++) {
            ChangeDetector::DetectedObject newDetectedObject;

            newDetectedObject.label = (std::string)sl::toString(objects.object_list[index].sublabel);
            newDetectedObject.position = objects.object_list[index].position;
            newDetectedObject.confidence = (int)objects.object_list[index].confidence;
            newDetectedObject.bounding_box_2d = objects.object_list[index].bounding_box_2d;
            newDetectedObject.bounding_box_3d = objects.object_list[index].bounding_box;
            newDetectedObject.label_confidence_pair.insert(std::make_pair(newDetectedObject.label, newDetectedObject.confidence));
            newDetectedObject.label_detection_num_pair.insert(std::make_pair(newDetectedObject.label, 1));
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
            if (newDetectedObject.has_object_3d_pointcloud == true && newDetectedObject.object_3d_pointcloud->points.size() > 0) {
                if (DetectedObjects.size() > 0) {
                    auto ids = return_closest_objects(DetectedObjects, newDetectedObject, eucl_dist, verbose); // distance of bounding box centroids
                    std::string writePath = "D:/zed codes/zed_change_pcl/build/obj_pcls/" + newDetectedObject.label + "_" + std::to_string(DetectedObjects.size()) + ".ply";
                    pcl::io::savePLYFileBinary(writePath, *newDetectedObject.object_3d_pointcloud);
                    if (ids.size() == 0) {
                        if (verbose) {
                            SetConsoleTextAttribute(hConsole, 2);
                            printf("Added new object %s with id [%d], because there were no close objects.\n", newDetectedObject.label, DetectedObjects.size());
                            SetConsoleTextAttribute(hConsole, 15);
                        }
                        DetectedObjects.push_back(newDetectedObject);
                    }
                    else {
                        if (verbose) std::cout << "Close objects: " << std::endl;
                        should_print = true;
                        for (int i = 0; i < ids.size(); i++) {
                            if (verbose) std::cout << DetectedObjects[ids[i]].label << " " << ids[i] << " ";
                            float percentage = knn_search(DetectedObjects[ids[i]].object_3d_pointcloud, newDetectedObject.object_3d_pointcloud, kd_dis); // squared distance of neighbourpoints
                            if (verbose) printf("Percentage of points within %d distance: %f\n", kd_dis, percentage);
                            if (percentage > 0.5) {
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

                                if (verbose) {
                                    SetConsoleTextAttribute(hConsole, 3);
                                    printf("Updated already existing object %s with id [%d].\n", DetectedObjects[ids[i]].label, ids[i]);
                                    SetConsoleTextAttribute(hConsole, 15);
                                }
                                for (auto& e : DetectedObjects[ids[i]].label_confidence_pair) {
                                    std::map<std::string, int>::iterator it3 = DetectedObjects[ids[i]].label_detection_num_pair.find(e.first);
                                    if (verbose) std::cout << '{' << e.first << ", " << e.second/it3->second << ", " <<  it3->second << '}' << '\n';
                                }
                                should_print = false;
                                break;
                            }
                        }
                        if (should_print) {
                            if (verbose) {
                                SetConsoleTextAttribute(hConsole, 4);
                                printf("Added new object %s with id [%d], because the nearby objects' pointclouds differ too much.\n", newDetectedObject.label, DetectedObjects.size());
                                SetConsoleTextAttribute(hConsole, 15);
                            }
                            DetectedObjects.push_back(newDetectedObject);
                        }
                    }
                }
                else {
                    if (verbose) {
                        SetConsoleTextAttribute(hConsole, 2);
                        printf("Added new object %s with id [%d], because it is the first detected object.\n", newDetectedObject.label, DetectedObjects.size());
                        SetConsoleTextAttribute(hConsole, 15);
                    }
                    DetectedObjects.push_back(newDetectedObject);
                }

            }
        }
    }
}

/**
 * This function finalizes the saved class labels and confidences -> assigns one label and confidence if there has been more linked to an object
 * input1: already detected, stored objects
 **/
void ChangeDetector::class_label_and_confidence_decision(std::vector<ChangeDetector::DetectedObject>& DetectedObjects) {
    for (int i = 0; i < DetectedObjects.size(); i++) {
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
void ChangeDetector::visualize_end_result(std::string input_pointcloud_path, std::vector<ChangeDetector::DetectedObject>& DetectedObjects) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile(input_pointcloud_path, *final_output);

    // Create the PCL point cloud visualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_final_viewer = createRGBVisualizer(final_output);
    pcl_final_viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    pcl_final_viewer->setCameraClipDistances(0.1, 1000);

    for (int i = 0; i < DetectedObjects.size(); i++) {
        if (DetectedObjects[i].detection_num > 1)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            bounding_box_cloud->points.resize(DetectedObjects[i].bounding_box_3d.size());
            for (int pont = 0; pont < DetectedObjects[i].bounding_box_3d.size(); pont++) {
                bounding_box_cloud->points[pont].x = DetectedObjects[i].bounding_box_3d[pont].x;
                bounding_box_cloud->points[pont].y = DetectedObjects[i].bounding_box_3d[pont].y;
                bounding_box_cloud->points[pont].z = DetectedObjects[i].bounding_box_3d[pont].z;
            }
            pcl::PointXYZRGB min_point_AABB;
            pcl::PointXYZRGB max_point_AABB;
            pcl::getMinMax3D(*bounding_box_cloud, min_point_AABB, max_point_AABB);
            pcl_final_viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 255, 0, 0, std::to_string(i));
            pcl_final_viewer->addText3D(DetectedObjects[i].label + "_" + std::to_string(i), DetectedObjects[i].position, 50, 255, 0, 0, DetectedObjects[i].label + "_" + std::to_string(i), 0);
            pcl_final_viewer->setRepresentationToWireframeForAllActors();
        }
    }
    while (!pcl_final_viewer->wasStopped())
    {
        pcl_final_viewer->spinOnce();
    }
}