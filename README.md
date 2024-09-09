# Interactive_Geometric_Deformation_Engine

A geometric deformation engine that can interact in real-time through serial communication

## Abstract

The deformation of surface mesh models has always been a hot research topic. Unlike FFD technology, Embedded Deformation Graph technology embeds deformation control points on the surface of the model itself, and expresses the overall deformation of the model through sampling. At present, the problem of interactive deformation with the real world is very difficult. Based on the principle of Embedded Deformation Graph technology, this project has built a deformation framework that can interact with the graphics engine in real-time through serial communication on the polyscope.

## Main contributions

1）Based on the principle of Embedded Deformation Graph, an engine framework that can interact with 3D models in real time through serial communication devices (or modified to other) is built.
2）The input of the engine is the number of feature points and the corresponding Cartesian coordinates, and the output is the real-time deformation results of the model

![](https://github.com/Scalpelapex/Images/blob/main/IGDE/RealtimeDeformation.gif)

## Environment setup

Clone the repo: 
> https://github.com/zhenguonie/VFCM-CHPS

Open this project with CMake Or Visual Studio 2019 - CMake project

## Libraries

Eigen

polyscope

yaml-cpp

utils

Notes: We have put them on the ./deps file. You can directly run the main code.

## Case:

We provide some .obj model on the ./data file. 

![](https://github.com/Scalpelapex/Images/blob/main/IGDE/Watering.gif)

## Comments

(1) Config
You need to modify the file "./config/config.yaml" to adjust the input/output path to yourself.

(2) Main Code

1> The file "./DeformationController/Motion_dermation.cpp" is the entrance of the project.

2>And the file "EmbeddedDeformation_tools.cpp" is the core of Embedded Deformation Graph Method.

3>The file "Serial.h" is about the serial communication. You can set your own communication format.
