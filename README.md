# Change detection algorithm using the ZED 2 stereo camera

This code is an initial version of the change detection algorithm developed using the features of the [ZED 2 stereo camera](https://www.stereolabs.com/zed-2/). The program was developed by Levente Göncz at `SZTAKI (Institute for Computer Science and Control)` in a framework of a 4 months long master thesis project, mandatory according to ELTE, Faculty of Informatics. The program is using some of the [tutorial codes](https://github.com/stereolabs/zed-examples) by the manufacturer of the ZED 2 stereo camera, Stereolabs.
The entire code of the algorithm was written by the author of this thesis. The parts proposed by [Sünderhauf et al.](https://ieeexplore.ieee.org/document/8206392) have been used as a written inspiration, the code itself was entirely programmed by the author of this thesis.

## Getting Started

 - Get the latest [ZED SDK and CUDA](https://www.stereolabs.com/developers/release/)
 - Note, the ZED SDK installer automatically installs the necessary CUDA version. During the development of this program, `ZED SDK for Windows 10` version `3.5.0` and `CUDA` version `11.1` were used. The program was developed and compiled using `Visual Studio 2019`.
 - Check the [Documentation](https://www.stereolabs.com/docs/)
 - In order to make sure the ZED SDK has been installed properly: try running one of the sample tutorial codes found [here](https://github.com/stereolabs/zed-examples).

## Dependencies

- OpenCV library for displaying and manipulating 2D images
- boost library for XML document generation
- Point Cloud Library (PCL) for point cloud data manipulation and display
- ZED camera specific library (SL, stands for StereoLabs, which is the manufacturer of the camera) for controlling the camera
- CUDA and Open Graphics Library (OpenGL) libraries for displaying 3D point clouds

## Build the program

Download the sample and follow the instructions below: [More](https://www.stereolabs.com/docs/getting-started/application-development/)

#### Build for Windows

- Create a "build" folder in the source folder
- Open cmake-gui and select the source and build folders
- Generate the Visual Studio `Win64` solution
- Open the resulting solution and change configuration to `Release`
- Build solution
 
## Run the program
- To run the program using a previously recorded footage, an input parameter is required: The absolute path to the .svo file containing the recording.

### Features
 - real time 3D display of the current fused point cloud
 - press 'f' to un/follow the camera movement
 
## Support
If you need assistance go to our Community site at https://community.stereolabs.com/
