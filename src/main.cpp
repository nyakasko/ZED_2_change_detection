// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"
#include "TrackingViewer.hpp"
#include "ChangeDetection.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <algorithm>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/common/io.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <filesystem>
#include <iostream>
//#include <pcl/surface/convex_hull.h>
//#include <libqhullcpp/Qhull.h>

// Using std and sl namespaces
using namespace std;
using namespace sl;

// macro for showing the 3d pointclouds segmented by their 3d bounding boxes
#define SHOW_SEGMENTED 0
// macro for deciding if the code runs for the t[i] or t[i+1] time - t[i] -> data association, t[i+1] ->change detection
#define first_run 0
// macro for deciding if the 3d pointcloud should be shown using GLViewer or PCLviewer
#define show_pointcloud_in_pcl 0

void print(std::string msg_prefix, sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS, std::string msg_suffix = "");
void parse_args(int argc, char **argv,InitParameters& param);
void print_object_map(std::vector<ChangeDetector::DetectedObject>& DetectedObjects);

ChangeDetector changedetector;
string saved_file_name = "debug_map";

int main(int argc, char **argv) {
    Mat data_cloud; // container for ZED 2 pointcloud 
    std::vector<ChangeDetector::DetectedObject> DetectedObjects;
    std::vector<ChangeDetector::DetectedObject> PreviouslyDetectedObjects;
    int id = 0; // Bounding box PCL id
    const string saved_xml_file_path = "D:/zed codes/zed_change_pcl/build/data_association.xml";
    /************************************************/
             /*Camera init*/

    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    init_parameters.camera_resolution = RESOLUTION::HD1080;
    init_parameters.depth_maximum_distance = 5.0f * 1000.0f;
    init_parameters.depth_minimum_distance = 0;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed 
    init_parameters.coordinate_units = UNIT::MILLIMETER;
    parse_args(argc, argv, init_parameters);
    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {// Quit if an error occurred
        print("Open Camera", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }
    /************************************************/
    sl::FusedPointCloud map;
    auto camera_infos = zed.getCameraInformation();
#if !show_pointcloud_in_pcl
    // Point cloud viewer
    GLViewer viewer;
    // Initialize point cloud viewer
    GLenum errgl = viewer.init(argc, argv, camera_infos.camera_configuration.calibration_parameters.left_cam, &map, camera_infos.camera_model);
    if (errgl != GLEW_OK)
        print("Error OpenGL: " + std::string((char*)glewGetErrorString(errgl)));
#endif



    /************************************************/
                /*Positional tracking init*/

    Pose pose;
    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;
    PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.enable_area_memory = true;
#if !first_run
    positional_tracking_parameters.area_file_path = (saved_file_name + ".area").c_str();
    // Read the previously created xml file that contains the detected objects in run t[i]
    changedetector.read_previously_saved_detected_objects(saved_xml_file_path, PreviouslyDetectedObjects);
    std::cout << "Previously detected objects have been successfully loaded from the xml file." << std::endl;
#endif

    returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Enabling positional tracking failed: ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }
    /************************************************/


    /************************************************/
                 /*Object detection init*/
    ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    detection_parameters.enable_mask_output = false; // designed to give person pixel mask
    detection_parameters.detection_model = DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;
    print("Object Detection: Loading Module...");
    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }
    // Detection output
    Objects objects;
    // Detection runtime parameters
    // default detection threshold, apply to all object class
    int detection_confidence = 50;
    ObjectDetectionRuntimeParameters detection_parameters_rt(detection_confidence);
    // To select a set of specific object classes:
    //detection_parameters_rt.object_class_filter = {OBJECT_CLASS::VEHICLE, OBJECT_CLASS::PERSON };
    // To set a specific threshold
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = detection_confidence + 20 < 100 ? detection_confidence + 20 : 100;
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::VEHICLE] = detection_confidence;
    /************************************************/


    /************************************************/
                /*Spatial mapping init*/

    // Set spatial mapping parameters
    SpatialMappingParameters spatial_mapping_parameters;
    // Request a Point Cloud
    spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
    // Set mapping range, it will set the resolution accordingly (a higher range, a lower resolution)
    spatial_mapping_parameters.set(SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
    spatial_mapping_parameters.set(SpatialMappingParameters::MAPPING_RESOLUTION::HIGH);
    // Request partial updates only (only the lastest updated chunks need to be re-draw)
    spatial_mapping_parameters.use_chunk_only = false;
    // Start the spatial mapping
    zed.enableSpatialMapping(spatial_mapping_parameters);
    /************************************************/

    // Timestamp of the last fused point cloud requested
    chrono::high_resolution_clock::time_point ts_last; 

    // Setup runtime parameters
    RuntimeParameters runtime_parameters;
    // Use low depth confidence avoid introducing noise in the constructed model
    runtime_parameters.confidence_threshold = 100;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;


    auto resolution = camera_infos.camera_configuration.resolution;
    // Define display resolution and check that it fit at least the image resolution
    Resolution display_resolution(min((int)resolution.width, 720), min((int)resolution.height, 404));
    Resolution tracks_resolution(400, display_resolution.height);
    // create a global image to store both image and tracks view
    cv::Mat global_image(display_resolution.height, display_resolution.width + tracks_resolution.width, CV_8UC4);
    // Create a Mat to contain the left image and its opencv ref
    auto image_zed_ocv = global_image(cv::Rect(0, 0, display_resolution.width, display_resolution.height));
    Mat image_zed(display_resolution, MAT_TYPE::U8_C4, image_zed_ocv.data, image_zed_ocv.step);
    // retrieve ref on tracks view part
    auto image_track_ocv = global_image(cv::Rect(display_resolution.width, 0, tracks_resolution.width, tracks_resolution.height));
    sl::float2 img_scale(display_resolution.width / (float)resolution.width, display_resolution.height / (float)resolution.height);

    // 2D tracks
    TrackingViewer track_view_generator(tracks_resolution, camera_infos.camera_configuration.fps, init_parameters.depth_maximum_distance);
    track_view_generator.setCameraCalibration(camera_infos.camera_configuration.calibration_parameters);

    string window_name = "ZED| 2D View and Birds view";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL); // Create Window
    cv::createTrackbar("Confidence", window_name, &detection_confidence, 100);

    char key = ' ';

    /************************************************/
                     /*PCL init*/

    // Allocate PCL point cloud at the resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
#if show_pointcloud_in_pcl
    // Create the PCL point cloud visualizer
    shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer = changedetector.createRGBVisualizer(p_pcl_point_cloud);

    // Set Viewer initial position
    pcl_viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    pcl_viewer->setCameraClipDistances(0.1, 1000);
#endif
#if SHOW_SEGMENTED
    shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer = changedetector.createRGBVisualizer(filtered_pcl);
    filter_viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    filter_viewer->setCameraClipDistances(0.1, 1000);
#endif

    /************************************************/
    auto calib_param_ = camera_infos.camera_configuration.calibration_parameters.left_cam;

    bool quit = false;
    int pcl_id = 5000;
    Pose cam_pose;
    HANDLE  hConsole;
    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    int found_area_relocalozation = 0;
    // Start the main loop
#if show_pointcloud_in_pcl
    while (!pcl_viewer->wasStopped() && !quit) {
#else
    while (viewer.isAvailable() && !quit){
#endif
        if ((char)cv::waitKey(1) == 27) break; // ESC PRESSED
        // Grab a new image
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

            returned_state = zed.retrieveObjects(objects, detection_parameters_rt);
            detection_parameters_rt.detection_confidence_threshold = detection_confidence;
            // Retrieve the left image
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, display_resolution);
            // Retrieve the camera pose data
            tracking_state = zed.getPosition(pose, REFERENCE_FRAME::WORLD);
#if !first_run
            // Finding the reference frame, based on the previous run's area map
            if (found_area_relocalozation == 0 && tracking_state == POSITIONAL_TRACKING_STATE::SEARCHING) {
                SetConsoleTextAttribute(hConsole, 14);
                cout << "Searching for position, based on previous area map....." << endl;
                SetConsoleTextAttribute(hConsole, 15);
                found_area_relocalozation = 1;
            }
            if (found_area_relocalozation == 1 && tracking_state == POSITIONAL_TRACKING_STATE::OK) {
                SetConsoleTextAttribute(hConsole, 2);
                cout << "Found position, based on previous area map" << endl;
                SetConsoleTextAttribute(hConsole, 15);
                found_area_relocalozation = 0;
            }

#endif

#if !show_pointcloud_in_pcl
            viewer.updatePose(pose, tracking_state);
            viewer.updateData(objects.object_list, pose.pose_data);
#endif
            if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {                
                auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();
                
                if (init_parameters.svo_real_time_mode == true) {
                    // Ask for a fused point cloud update if 500ms have elapsed since last request
#if !show_pointcloud_in_pcl
                    if ((duration > 100) && viewer.chunksUpdated()) { // 100 or 500 if svo_real_time_mode = true
#else
                    if (duration > 100){
#endif
                        // Ask for a point cloud refresh
                        zed.requestSpatialMapAsync();
                        ts_last = chrono::high_resolution_clock::now();
                    }
                }
                else {
                    // Ask for a fused point cloud update if 500ms have elapsed since last request
#if !show_pointcloud_in_pcl
                    if ((duration > 100) && viewer.chunksUpdated()) { // 100 or 500 if svo_real_time_mode = true
#else
                    if (duration > 10) {
#endif
                        // Ask for a point cloud refresh
                        zed.requestSpatialMapAsync();
                        ts_last = chrono::high_resolution_clock::now();
                    }
                }

                
                // If the point cloud is ready to be retrieved
                if(zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS) {                    
                    zed.retrieveSpatialMapAsync(map);
#if !show_pointcloud_in_pcl
                    viewer.updateChunks();
#endif
                }
            }
#if SHOW_SEGMENTED 
            // Crop bounding boxes of object from point cloud
            segment_and_show_bounding_box_from_point_cloud(filter_viewer, objects, p_pcl_point_cloud, filtered_pcl, id);
#endif
            // Show 2D bounding boxes on image frames
            // changedetector.show_object_detection_on_image(objects, image_zed_ocv, display_resolution, resolution, detection_confidence);
            detection_parameters_rt.detection_confidence_threshold = detection_confidence;
            // Retrieve pointcloud generated by ZED2
            zed.retrieveMeasure(data_cloud, MEASURE::XYZRGBA, MEM::CPU, display_resolution);

            // Convert ZED2 pointcloud to PCL format
            changedetector.measurement_to_pcl(data_cloud, p_pcl_point_cloud);
#if show_pointcloud_in_pcl
            if (init_parameters.svo_real_time_mode == false && zed.getSVOPosition() % 10 == 0) {
                // Send PCL point cloud to PCL viewer
                pcl_viewer->addPointCloud(p_pcl_point_cloud, std::to_string(pcl_id));
                //pcl_viewer->resetCamera();
                //pcl_viewer->resetCameraViewpoint(std::to_string(pcl_id));
                // Show 3D bounding boxes on point cloud
                changedetector.show_object_detection_on_point_cloud(pcl_viewer, objects, id);
                pcl_id += 1;
            }
#endif
            zed.getPosition(cam_pose, REFERENCE_FRAME::WORLD);
#if first_run
            // Data association
            changedetector.data_association_of_detected_objects(p_pcl_point_cloud, objects, DetectedObjects, 1000, 10000, false); //eucl_dist and kd_dist and verbose
#else
            // Displaying previous detections on the new run's IMAGE
            changedetector.find_and_reproject_previous_detections_onto_image(image_zed_ocv, p_pcl_point_cloud, PreviouslyDetectedObjects, cam_pose,
                init_parameters, calib_param_, display_resolution, resolution);
#endif
#if !show_pointcloud_in_pcl && !first_run
            // Displaying previous detections on the new run's POINTCLOUD
            changedetector.find_and_show_previous_detections_on_pointcloud(p_pcl_point_cloud, PreviouslyDetectedObjects, cam_pose, init_parameters, calib_param_, objects.object_list);
            viewer.updateData(objects.object_list, pose.pose_data);
#endif

            // as image_zed_ocv is a ref of image_left, it contains directly the new grabbed image
            render_2D(image_zed_ocv, img_scale, objects.object_list, cam_pose, true);

            // update birds view of tracks based on camera position and detected objects
            track_view_generator.generate_view(objects, cam_pose, image_track_ocv, objects.is_tracked);
            cv::imshow(window_name, global_image);
            key = cv::waitKey(10);
            if (key == 'i') {
                track_view_generator.zoomIn();
            }
            else if (key == 'o') {
                track_view_generator.zoomOut();
            }
            else if (key == 'q') {
                quit = true;
            }
        }
    }

#if first_run
    // Final assignment of labels and confidences
    changedetector.class_label_and_confidence_decision(DetectedObjects);

    // Save generated point cloud
    map.save(saved_file_name.c_str(), MESH_FILE_FORMAT::PLY);

    // Save and visualize the result, but leave out objects with number of detections less than 5 and pointcloud points less than 200
    changedetector.save_and_visualize_end_result("D:/zed codes/zed_change_pcl/build/" + saved_file_name + ".ply", DetectedObjects);

    // Save area map so that in the second run the camera will be able relocalize itself
    zed.saveAreaMap((saved_file_name + ".area").c_str());
    std::cout << "Area map has been successfully saved." << std::endl;
    // Print the final and cleaned object map that got saved into the xml file.
    print_object_map(DetectedObjects);
#else
    map.save((saved_file_name + "_2").c_str(), MESH_FILE_FORMAT::PLY);
#endif

    // Free allocated memory before closing the camera
    image_zed.free();
    // Close the ZED
    zed.close();

    return 0;
}

void print_object_map(std::vector<ChangeDetector::DetectedObject>& DetectedObjects) {
    printf("Printing the entire objectmap\n");
    for (int i = 0; i < DetectedObjects.size(); i++) {
        printf("id: %d\n", DetectedObjects[i].tracking_id);
        printf("label: %s\n", DetectedObjects[i].label);
        printf("num of detections: %d\n", DetectedObjects[i].overall_detection_num);
        printf("confidence: %d\n", DetectedObjects[i].confidence);
        printf("position: %f %f %f\n", DetectedObjects[i].position[0], DetectedObjects[i].position[1], DetectedObjects[i].position[2]);
        printf("has pcl %d\n", DetectedObjects[i].has_object_3d_pointcloud);
    }
}

void parse_args(int argc, char **argv,InitParameters& param){
    if (argc > 1 && string(argv[1]).find(".svo")!=string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        param.svo_real_time_mode=false; 

        cout<<"[Sample] Using SVO File input: "<<argv[1]<<endl;
    } else if (argc > 1 && string(argv[1]).find(".svo")==string::npos) {
        string arg = string(argv[1]);
        unsigned int a,b,c,d,port;
        if (sscanf(arg.c_str(),"%u.%u.%u.%u:%d", &a, &b, &c, &d,&port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a)+"."+to_string(b)+"."+to_string(c)+"."+to_string(d);
            param.input.setFromStream(String(ip_adress.c_str()),port);
            cout<<"[Sample] Using Stream input, IP : "<<ip_adress<<", port : "<<port<<endl;
        }
        else  if (sscanf(arg.c_str(),"%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(String(argv[1]));
            cout<<"[Sample] Using Stream input, IP : "<<argv[1]<<endl;
        }
        else if (arg.find("HD2K")!=string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout<<"[Sample] Using Camera in resolution HD2K"<<endl;
        } else if (arg.find("HD1080")!=string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout<<"[Sample] Using Camera in resolution HD1080"<<endl;
        } else if (arg.find("HD720")!=string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout<<"[Sample] Using Camera in resolution HD720"<<endl;
        } else if (arg.find("VGA")!=string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout<<"[Sample] Using Camera in resolution VGA"<<endl;
        }
    } else {
        // Default
    }
}

void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
    cout <<"[Sample]";
    if (err_code != sl::ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    else
        cout<<" ";
    cout << msg_prefix << " ";
    if (err_code != sl::ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}