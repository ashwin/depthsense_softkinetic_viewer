Sample program provided in DepthSense SDK modified to read RGB-depth images
from a SoftKinetic camera and display them using OpenCV.

Requirements:

First make sure you install the DepthSense SDK.

OpenCV is used for displaying the depth and color (RGB) images.
It can installed easily:

```
$ sudo apt install libopencv-dev
```

To compile:

```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To run, first connect the SoftKinetic camera and then run:

```
$ cd bin
$ ./depthsense_softkinetic_viewer
```


- Ashwin Nanjappa
