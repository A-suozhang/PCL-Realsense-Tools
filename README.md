#Useful Tools with PCL & librealsense
This code provides some useful tools for capturing stream of point cloud(RGB)  of The Intel Realsense depth camera. We used PCL1.8.1 (Point cloud Library & Librealsense), for how to set up the environment, you could refer to tutorials online.
##Code Implemention
We built the project using Visual Studio 2017, via opening the .sln File in VS, you could open the project and build it yourself(Make sure to build with Debug&x64)

##Exeutable Tools
If you only want to use the tools without modifying params, you could find the .exe directly in home folder. Containing the .pcd(The save format of the Point Cloud) and the .exe and .cpp
```
if you wanna use the tool, instead of just  run the .exe
you could drag the .pcd flle on the .exe file to use it

also, if you want to open the .pcd file with a certain way 
by defalut(for example, our visualization tool), you could right-click 
the .pcd file, and set the PCLVisualizer as the default opening way
```
