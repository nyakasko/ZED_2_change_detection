// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include <math.h>
#include <algorithm>
#include "GLViewer.hpp"
#include "ChangeDetection.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/common/io.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>

// Using std and sl namespaces
using namespace std;
using namespace sl;

#define SHOW_SEGMENTED 0

void print(std::string msg_prefix, sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS, std::string msg_suffix = "");
void parse_args(int argc, char **argv,InitParameters& param);
void print_object_map(std::vector<ChangeDetector::DetectedObject>& DetectedObjects);

ChangeDetector changedetector;

int main(int argc, char **argv) {
    Mat data_cloud; // container for ZED 2 pointcloud measurement

    std::vector<ChangeDetector::DetectedObject> DetectedObjects;
    int id = 0; // Bounding box PCL id
    /************************************************/
             /*Camera init*/

    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD2K;
    init_parameters.depth_maximum_distance = 5.0f * 1000.0f;
    init_parameters.depth_minimum_distance = 1000.0f;
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

    // Point cloud viewer
    //GLViewer viewer;
    auto camera_infos = zed.getCameraInformation();
    // Initialize point cloud viewer
    FusedPointCloud map;
    //GLenum errgl = viewer.init(argc, argv, camera_infos.camera_configuration.calibration_parameters.left_cam, &map, camera_infos.camera_model);
    //if (errgl!=GLEW_OK)
    //    print("Error OpenGL: "+std::string((char*)glewGetErrorString(errgl)));


    /************************************************/
                /*Positional tracking init*/

    Pose pose;
    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;
    PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.enable_area_memory = false;
    //positional_tracking_parameters.area_file_path = "otthon.area";
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
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = detection_confidence;
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::VEHICLE] = detection_confidence;
    /************************************************/


    /************************************************/
                /*Spatial mapping init*/

    // Set spatial mapping parameters
    SpatialMappingParameters spatial_mapping_parameters;
    // Request a Point Cloud
    spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
    // Set mapping range, it will set the resolution accordingly (a higher range, a lower resolution)
    spatial_mapping_parameters.set(SpatialMappingParameters::MAPPING_RANGE::LONG);
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
    // Create a Mat to contain the left image and its opencv ref
    Mat image_zed(display_resolution, MAT_TYPE::U8_C4);
    cv::Mat image_zed_ocv(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(MEM::CPU));


    /************************************************/
                     /*PCL init*/

    // Allocate PCL point cloud at the resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the PCL point cloud visualizer
    shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer = changedetector.createRGBVisualizer(p_pcl_point_cloud);

    // Set Viewer initial position
    // pcl_viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    pcl_viewer->setCameraClipDistances(0.1, 1000);
#if SHOW_SEGMENTED
    shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer = changedetector.createRGBVisualizer(filtered_pcl);
    filter_viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    filter_viewer->setCameraClipDistances(0.1, 1000);
#endif

    /************************************************/
    int pcl_id = 5000;
    // Start the main loop
    while (!pcl_viewer->wasStopped()) { // viewer.isAvailable()
        if ((char)cv::waitKey(1) == 27) break; // ESC PRESSED
        // Grab a new image
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

            returned_state = zed.retrieveObjects(objects, detection_parameters_rt);

            // Retrieve the left image
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, display_resolution);
            // Retrieve the camera pose data
            tracking_state = zed.getPosition(pose);
            //viewer.updatePose(pose, tracking_state);
            //viewer.updateData(objects.object_list, pose.pose_data);
            if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {                
                auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();
                
                // Ask for a fused point cloud update if 500ms have elapsed since last request
                if((duration > 100)){//&& viewer.chunksUpdated()) {
                    // Ask for a point cloud refresh
                    zed.requestSpatialMapAsync();
                    ts_last = chrono::high_resolution_clock::now();
                }
                
                // If the point cloud is ready to be retrieved
                if(zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS) {                    
                    zed.retrieveSpatialMapAsync(map);
                    //viewer.updateChunks();
                }
            }
#if SHOW_SEGMENTED 
            // Crop bounding boxes of object from point cloud
            segment_and_show_bounding_box_from_point_cloud(filter_viewer, objects, p_pcl_point_cloud, filtered_pcl, id);
#endif
            // Show 2D bounding boxes on image frames
            changedetector.show_object_detection_on_image(objects, image_zed_ocv, display_resolution, resolution);
            // Retrieve pointcloud generated by ZED2
            zed.retrieveMeasure(data_cloud, MEASURE::XYZRGBA, MEM::CPU, display_resolution);

            // Convert ZED2 pointcloud to PCL format
            changedetector.measurement_to_pcl(data_cloud, p_pcl_point_cloud);

            // Send PCL point cloud to PCL viewer
            pcl_viewer->addPointCloud(p_pcl_point_cloud, std::to_string(pcl_id));
            //pcl_viewer->resetCamera();
            //pcl_viewer->resetCameraViewpoint(std::to_string(pcl_id));
            pcl_id += 1;
            // Show 3D bounding boxes on point cloud
            changedetector.show_object_detection_on_point_cloud(pcl_viewer, objects, id);

            // Data association
            changedetector.data_association_of_detected_objects(p_pcl_point_cloud, objects, DetectedObjects, 1000, 10000, false); //eucl_dist and kd_dist and verbose
        }
    }
    // Final assignment of labels and confidences 
    changedetector.class_label_and_confidence_decision(DetectedObjects);

    print_object_map(DetectedObjects);

    // Save generated point cloud
    map.save("otthon2", MESH_FILE_FORMAT::PLY);

    changedetector.visualize_end_result("D:/zed codes/zed_change_pcl/build/otthon2.ply", DetectedObjects);

    //zed.saveAreaMap("otthon.area");

    // Free allocated memory before closing the camera
    image_zed.free();
    // Close the ZED
    zed.close();

    return 0;
}

void print_object_map(std::vector<ChangeDetector::DetectedObject>& DetectedObjects) {
    printf("Printing the entire objectmap\n");
    for (int i = 0; i < DetectedObjects.size(); i++) {
        printf("id: %d\n", i);

        //printf("label-conf-detection trio\n");
        //for (auto& e : DetectedObjects[i].label_confidence_pair) {
        //    std::map<std::string, int>::iterator it3 = DetectedObjects[i].label_detection_num_pair.find(e.first);
        //    std::cout << '{' << e.first << ", " << e.second / it3->second << ", " << it3->second << '}' << '\n';
        //}

        printf("label: %s\n", DetectedObjects[i].label);
        printf("num of detections: %d\n", DetectedObjects[i].overall_detection_num);
        printf("confidence: %d\n", DetectedObjects[i].confidence);
        printf("position: %f %f %f\n", DetectedObjects[i].position[0], DetectedObjects[i].position[1], DetectedObjects[i].position[2]);
        //printf("bounding box 2d\n");
        //for (int bb = 0; bb < DetectedObjects[i].bounding_box_2d.size(); bb++) {
        //    std::cout << DetectedObjects[i].bounding_box_2d[bb].x << " " << DetectedObjects[i].bounding_box_2d[bb].y << std::endl;
        //}
        //printf("bounding box 3d\n");
        //for (int bb = 0; bb < DetectedObjects[i].bounding_box_3d.size(); bb++) {
        //    std::cout << DetectedObjects[i].bounding_box_3d[bb].x << " " << DetectedObjects[i].bounding_box_3d[bb].y << " " << DetectedObjects[i].bounding_box_3d[bb].z << std::endl;
        //}
        printf("has pcl %d\n", DetectedObjects[i].has_object_3d_pointcloud);
    }
}

void parse_args(int argc, char **argv,InitParameters& param){
    if (argc > 1 && string(argv[1]).find(".svo")!=string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        param.svo_real_time_mode=true;

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