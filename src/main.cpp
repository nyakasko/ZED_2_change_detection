///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**********************************************************************************
 ** This sample demonstrates how to capture a live 3D reconstruction of a scene  **
 ** as a fused point cloud and display the result in an OpenGL window.           **
 **********************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <algorithm>
#include <pcl/filters/crop_box.h>
// Using std and sl namespaces
using namespace std;
using namespace sl;

void parse_args(int argc, char **argv,InitParameters& param);
cv::Point resize_boundingbox_coordinates(int x, int y, Resolution display_resolution, Resolution camera_resolution);
void print(std::string msg_prefix, sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS, std::string msg_suffix = "");
void show_object_detection_on_image(Objects objects, cv::Mat image_zed_ocv, Resolution display_resolution, Resolution camera_resolution);
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
inline float convertColor(float colorIn);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr measurement_to_pcl(sl::Mat measurement, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud);
void segment_bounding_box_from_point_cloud(shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer, sl::Objects objects,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl);
void show_object_detection_on_point_cloud(shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer, sl::Objects objects);

Mat data_cloud; // container for ZED 2 pointcloud measurement
int id = 0; // Bounding box PCL id

int main(int argc, char **argv) {

    /************************************************/
             /*Camera init*/
    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD2K;
    init_parameters.depth_maximum_distance = 10.0f * 1000.0f;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed    
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
    int detection_confidence = 70;
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
    // Request partial updates only (only the lastest updated chunks need to be re-draw)
    spatial_mapping_parameters.use_chunk_only = true;
    // Start the spatial mapping
    zed.enableSpatialMapping(spatial_mapping_parameters);
    /************************************************/

    // Timestamp of the last fused point cloud requested
    chrono::high_resolution_clock::time_point ts_last; 

    // Setup runtime parameters
    RuntimeParameters runtime_parameters;
    // Use low depth confidence avoid introducing noise in the constructed model
    runtime_parameters.confidence_threshold = 50;


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
    shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer = createRGBVisualizer(p_pcl_point_cloud);
    shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer = createRGBVisualizer(filtered_pcl);

    // Set Viewer initial position
    pcl_viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    pcl_viewer->setCameraClipDistances(0.1, 1000);
    filter_viewer->setCameraPosition(0, 0, 5, 0, 0, 1, 0, 1, 0);
    filter_viewer->setCameraClipDistances(0.1, 1000);

    /************************************************/

    // Start the main loop
    while (!pcl_viewer->wasStopped()) { // viewer.isAvailable()
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
            // Show 2D bounding boxes on image frames
            show_object_detection_on_image(objects, image_zed_ocv, display_resolution, resolution);
            // Retrieve pointcloud generated by ZED2
            zed.retrieveMeasure(data_cloud, MEASURE::XYZRGBA, MEM::CPU, display_resolution);
            // Convert ZED2 pointcloud to PCL format
            p_pcl_point_cloud = measurement_to_pcl(data_cloud, p_pcl_point_cloud);
            // Send PCL point cloud to PCL viewer
            pcl_viewer->updatePointCloud(p_pcl_point_cloud);
            // Show 3D bounding boxes on point cloud
            show_object_detection_on_point_cloud(pcl_viewer, objects);
            // Crop bounding boxes of object from point cloud
            segment_bounding_box_from_point_cloud(filter_viewer, objects, p_pcl_point_cloud, filtered_pcl);
        }
    }

    // Save generated point cloud
    map.save("otthon");
    //zed.saveAreaMap("otthon.area");

    // Free allocated memory before closing the camera
    image_zed.free();
    // Close the ZED
    zed.close();

    return 0;
}

void show_object_detection_on_point_cloud(shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer, sl::Objects objects) {
    for (int num = 0; num < id; num++){
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

void segment_bounding_box_from_point_cloud(shared_ptr<pcl::visualization::PCLVisualizer> filter_viewer, sl::Objects objects,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pcl) {
    for (int num = 0; num < id; num++){
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr measurement_to_pcl(sl::Mat measurement, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud) {
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
    return p_pcl_point_cloud;
}
void show_object_detection_on_image(Objects objects, cv::Mat image_zed_ocv, Resolution display_resolution, Resolution camera_resolution) {
    if (!objects.object_list.empty()) {
        for (int index = 0; index < objects.object_list.size(); index++) {
            auto label = objects.object_list[index].sublabel;
            auto confidence = objects.object_list[index].confidence;
            auto bounding_box = objects.object_list.at(index).bounding_box_2d;
            string cv_text = (string)sl::toString(label) + " " + std::to_string((int)confidence) + "%";
            cv::rectangle(image_zed_ocv, resize_boundingbox_coordinates(bounding_box[0].x, bounding_box[0].y, display_resolution, camera_resolution),
                resize_boundingbox_coordinates(bounding_box[2].x, bounding_box[2].y, display_resolution, camera_resolution), cv::Scalar(255, 0, 0));
            cv::putText(image_zed_ocv, cv_text, resize_boundingbox_coordinates(bounding_box[0].x, bounding_box[0].y - 10, display_resolution, camera_resolution), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
        }
    }
    cv::imshow("ZED View", image_zed_ocv);
    cv::waitKey(15);
}

cv::Point resize_boundingbox_coordinates(int x, int y, Resolution display_resolution, Resolution camera_resolution) {
    float ratio_width = (float)((float)display_resolution.width / camera_resolution.width);
    float ratio_height = (float)((float)display_resolution.height / camera_resolution.height);
    float x_new = (float)((float)x * ratio_width);
    float y_new = (float)((float)y * ratio_height);
    return cv::Point(int(x_new), int(y_new));
}

void parse_args(int argc, char **argv,InitParameters& param)
{
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

/**
 *  This function creates a PCL visualizer
 **/
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

/**
 *  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t*)&colorIn;
    unsigned char* color_uchar = (unsigned char*)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float*> (&color_uint);
}