# Change detection algorithm using the ZED 2 stereo camera

This code is an initial version of the change detection algorithm developed using the [ZED 2 stereo camera](https://www.stereolabs.com/zed-2/). The program was developed by Levente Göncz at `SZTAKI (Institute for Computer Science and Control)` in a framework of a 4 months long master thesis project, mandatory according to the [`EIT Digital Masterschool`](https://masterschool.eitdigital.eu/), `Aalto University` and `ELTE (Eötvös Loránd University), Faculty of Informatics`. The program is using some of the [tutorial codes](https://github.com/stereolabs/zed-examples) provided by the manufacturer of the ZED 2 stereo camera, Stereolabs.
The entire code of the algorithm was written by the author of this thesis. The parts proposed by [Sünderhauf et al.](https://ieeexplore.ieee.org/document/8206392) have been used as a written inspiration, the code itself was entirely programmed by Levente Göncz. 

Please find the written thesis containing the results of the change detection algorithm along with various tests on the ZED 2 stereo camera in the `documentation` folder of this repository.

## Getting Started

 - Get the latest [ZED SDK and CUDA](https://www.stereolabs.com/developers/release/)
 - Note, the ZED SDK installer automatically installs the necessary CUDA version. During the development of this program, `ZED SDK for Windows 10` version `3.5.0` and `CUDA` version `11.1` were used. The program was developed and compiled using `Visual Studio 2019`.
 - Check the [Documentation](https://www.stereolabs.com/docs/)
 - In order to make sure the ZED SDK has been installed properly, please try building and running one of the sample tutorial codes found [here](https://github.com/stereolabs/zed-examples).

## Dependencies

- [OpenCV](https://github.com/stereolabs/zed-opencv) library for displaying and manipulating 2D images
- boost library for XML document generation
- [Point Cloud Library (PCL)](https://github.com/stereolabs/zed-pcl) for point cloud data manipulation and display
- ZED camera specific library (SL, stands for StereoLabs, which is the manufacturer of the camera) for controlling the camera
- CUDA and Open Graphics Library (OpenGL) libraries for displaying 3D point clouds

## Build the program

Follow the instructions below: [More](https://www.stereolabs.com/docs/app-development/cpp/windows/)

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
