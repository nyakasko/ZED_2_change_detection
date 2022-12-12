# Change detection algorithm using the ZED 2 stereo camera

If you find this repository useful in your research, please consider citing our [published paper](https://www.mdpi.com/1424-8220/22/17/6342):

Bibtex format :
```
@Article{s22176342,
AUTHOR = {Göncz, Levente and Majdik, András László},
TITLE = {Object-Based Change Detection Algorithm with a Spatial AI Stereo Camera},
JOURNAL = {Sensors},
VOLUME = {22},
YEAR = {2022},
NUMBER = {17},
ARTICLE-NUMBER = {6342},
URL = {https://www.mdpi.com/1424-8220/22/17/6342},
PubMedID = {36080799},
ISSN = {1424-8220},
ABSTRACT = {This paper presents a real-time object-based 3D change detection method that is built around the concept of semantic object maps. The algorithm is able to maintain an object-oriented metric-semantic map of the environment and can detect object-level changes between consecutive patrol routes. The proposed 3D change detection method exploits the capabilities of the novel ZED 2 stereo camera, which integrates stereo vision and artificial intelligence (AI) to enable the development of spatial AI applications. To design the change detection algorithm and set its parameters, an extensive evaluation of the ZED 2 camera was carried out with respect to depth accuracy and consistency, visual tracking and relocalization accuracy and object detection performance. The outcomes of these findings are reported in the paper. Moreover, the utility of the proposed object-based 3D change detection is shown in real-world indoor and outdoor experiments.},
DOI = {10.3390/s22176342}
}
```
APA format:
```
Göncz, L., & Majdik, A. (2022). Object-Based Change Detection Algorithm with a Spatial AI Stereo Camera. Sensors, 22(17), 6342. https://doi.org/10.3390/s22176342
```

This code is an initial version of the change detection algorithm developed using the [ZED 2 stereo camera](https://www.stereolabs.com/zed-2/). The program was developed by Levente Göncz at `SZTAKI (Institute for Computer Science and Control)`. The program is using some of the [tutorial codes](https://github.com/stereolabs/zed-examples) provided by the manufacturer of the ZED 2 stereo camera, Stereolabs.
The data association part proposed by [Sünderhauf et al.](https://ieeexplore.ieee.org/document/8206392) has been used as a written inspiration, the code itself was entirely programmed by Levente Göncz. 


## Getting Started

 - Get the latest [ZED SDK and CUDA](https://www.stereolabs.com/developers/release/)
 - Note, the ZED SDK installer automatically installs the necessary CUDA version. During the development of this program, `ZED SDK for Windows 10` version `3.5.0` and `CUDA` version `11.1` were used. The program was developed and compiled using `Visual Studio 2019`.
 - Important: A computer with at least 4 GB RAM and NVIDIA GPU with `Compute Capabilities > 3` is required, along with a USB 3.0 port. Without the necessary GPU, the ZED SDK won't work at all.
 - Check the [Documentation](https://www.stereolabs.com/docs/)
 - In order to make sure the ZED SDK has been installed properly, please try building and running one of the sample tutorial codes found [here](https://github.com/stereolabs/zed-examples).

## Dependencies

- [OpenCV](https://github.com/stereolabs/zed-opencv) library for displaying and manipulating 2D images
- boost library for XML document generation
- [Point Cloud Library (PCL)](https://github.com/stereolabs/zed-pcl) for point cloud data manipulation and display
- ZED camera specific library (SL, stands for StereoLabs, which is the manufacturer of the camera) for controlling the camera
- CUDA and Open Graphics Library (OpenGL) libraries for displaying 3D point clouds

## Build the program

Follow the instructions found here: [Building a C++ Application on Windows/Linux](https://www.stereolabs.com/docs/app-development/cpp/windows/)

#### Build for Windows

- Create a "build" folder in the source folder
- Open cmake-gui and select the source and build folders
- Generate the Visual Studio `Win64` solution
- Open the resulting solution and change configuration to `Release`
- Build solution
 
## Run the program
- To run the program using a previously recorded footage, an input parameter is required: The absolute path to the .svo file containing the recording.
- Moreover, in the code, the `first_run` macro determines if the provided recording will be used only to create the semantic map or look for an already saved semantic map and detect changes using comparison between multiple runs. (This feature should be updated to work automatically)

More specifically: During the `first run`, the following algorithm that is described in the image below runs.

![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/change_det_1.PNG)

It uses the following data association step to create an object oriented semantic map:

Data association step             |  Object oriented semantic map
:-------------------------:|:-------------------------:
![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/data_association_step.PNG)  |  ![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/built_semantic_map.PNG)

Example of 3D semantic mapping and the segmented 3D point clouds of stored objects in the object database:
3D semantic mapping             |  3D point clouds of stored objects in the object database
:-------------------------:|:-------------------------:
![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/example_of_semantic_mapping.PNG)  |  ![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/segmented_objects.PNG)

The XML structure that is used to store the object of the segmented object database:

![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/xml_structure.PNG)

During the 2nd patrol route, the following algorithm, described in the image below, runs.

![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/change_det_2.PNG)

Example of the change detection algorithm:

![](https://github.com/nyakasko/ZED_2_change_detection/blob/main/documentation/example_of_change_detection.PNG)
