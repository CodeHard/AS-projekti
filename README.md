# Kinect model scanner and recognizer
In this project, it was meant to build a program that could scan models from Kinect-camera, and then recognize those models. However, these features are very limited at the moment.

This project was build for a project course in Aalto University, School of Electrical Engineering, Department of Automation and Systems Technology.

## Installation instructions
1. [Install PCL 1.7.0 from SVN](http://pointclouds.org/downloads/source.html), after that you should have all required libraries and cmake
2. Build this project with CMake (building is done same way as for PCL)
3. Install Kinect-drivers ([for Windows](http://www.codeproject.com/Articles/148251/How-to-Successfully-Install-Kinect-on-Windows-Open))

## Usage
When you have build the project, you should have two binaries: scanner and recognizer. Scanner can be run with commandline argument, which is then used as an output folder for saved models. Models are saved as PCD-files.

The UI shows all the keys you can use.

The recognizer includes a lot of experimental code. You should study the source code to make it function properly.

## Tests
The project has some tests in the **test**-folder. Some of them might require additional data, that is not distributed through this repository.