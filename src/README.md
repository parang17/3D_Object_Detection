# 3D Object Tracking based on LiDAR and Sensor data

The aim of this project computes the time to collision based on the LiDAR and camera information. The method uses feature detector, descriptor, YOLO, and feature matching method.
<img src="images/Diagram.png" width="779" height="414" />

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.


## Performance Evaluation 1
Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.
The following three figures show that the third figure's TTC slightly increases compared to the previous two, first and second results. 

<img src="images/1.png" width="779" height="414" />
<img src="images/2.png" width="779" height="414" />
<img src="images/3.png" width="779" height="414" />

The reason is that collected LiDAR on the third data has more noisy compared to the previous. The following figures visually present two different point cloud. To resolve this one, 
it requires outlier detection and elimination instead of using the average or median values.
<img src="images/2-LiDAR.png" width="779" height="414" />
<img src="images/3-LiDAR.png" width="779" height="414" />

## Performance Evaluation 2
Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

Based on the previous comparison assessment (https://github.com/parang17/Sensor-Fusion-Feature-Tracking), I ranked three different combinations: FAST+BRIEF, FAST+ORB, ORB+BRIEF.
The result show that FAST+BRIEF is the best since standard deviation is smaller than others. The thrid case, one estimated TTC result is inf. Therefore, it cannot compute the mean and standard deviation.     

* All combinations
Table 3. Summary of all the detection/extraction/matching 
| Combination(detect + descriptor)| TTC Mean (sec)      | TTC Standard deviation(sec) | 
| ---                             | ---                 | ---                         |                 
| Shi-Tomasi + SIFT               |      12.0737        |    1.09771                  |
| Shi-Tomasi + ORB                |      12.0182        |    1.42699                  |
| Shi-Tomasi + FREAK              |      12.506         |    2.01588                  |
| Shi-Tomasi + AKAZE              |      N/A            |    N/A                      |
| Shi-Tomasi + BRIEF              |      12.3081        |    1.52982                  |
| HARRIS + SIFT                   |      N/A            |    N/A                      |
| HARRIS + ORB                    |      N/A            |    N/A                      | 
| HARRIS + FREAK                  |      N/A            |    N/A                      | 
| HARRIS + AKAZE                  |      N/A            |    N/A                      |
| HARRIS + BRIEF                  |      N/A            |    N/A                      |
| FAST + SIFT                     |      13.0881        |    4.05403                  |
| FAST + ORB                      |      13.7737        |    4.49047                  |
| FAST + FREAK                    |      12.1466        |    1.5195                   |
| FAST + AKAZE                    |      N/A            |    N/A                      |
| FAST + BRIEF                    |      12.5101        |    3.22864                  | 
| BRISK + SIFT                    |      14.5577        |    3.61103                  |
| BRISK + ORB                     |      16.4222        |    7.42531                  |
| BRISK + FREAK                   |      14.8593        |    5.13075                  |
| BRISK + AKAZE                   |      N/A            |    N/A                      |
| BRISK + BRIEF                   |      13.2849        |    1.95096                  |
| ORB + SIFT                      |      49.6274        |    118.82                   |
| ORB + ORB                       |      15.0126        |    11.2231                  | 
| ORB + FREAK                     |      9.80745        |    8.41832                  |
| ORB + AKAZE                     |      N/A            |    N/A                      | 
| ORB + BRIEF                     |      22.7376        |    30.0728                  |
| AKAZE + SIFT                    |      12.3961        |    2.19857                  | 
| AKAZE + ORB                     |      12.3889        |    1.8007                   |
| AKAZE + FREAK                   |      12.5629        |    2.38933                  |
| AKAZE + AKAZE                   |      N/A            |    N/A                      | 
| AKAZE + BRIEF                   |      12.3445        |    2.22289                  |
| SIFT + SIFT                     |      12.2244        |    2.74215                  |
| SIFT + FREAK                    |      12.8183        |    2.75282                  | 
| SIFT + AKAZE                    |      N/A            |    N/A                      |
| SIFT + BRIEF                    |      12.9706        |    3.81752                  |



* Top three summary 
| Rank | Combination(detect + descriptor)   | Mean   | Standard deviation    |
| ---- | ---                                | ---    |  ---                  |
| 1    | FAST + BRIEF                       |12.5101 | 3.22864               |
| 2    | FAST + ORB                         |13.7737|    4.49047             | 
| 3    | ORB + BRIEF                        | N/A    | N/A                   |     

