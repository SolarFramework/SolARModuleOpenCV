# SolARModuleOpenCV

[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleOpencv?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)
[![Build Status](http://argo.ci.b-com.com/buildStatus/icon?job=SolAR-Modules%2FSolARModuleOpenCV%2Fmaster&style=flat-square&subject=Master)](http://argo.ci.b-com.com/job/SolAR-Modules/job/SolARModuleOpenCV/job/master/)
[![Build Status](http://argo.ci.b-com.com/buildStatus/icon?job=SolAR-Modules%2FSolARModuleOpenCV%2Fdevelop&style=flat-square&subject=Dev)](http://argo.ci.b-com.com/job/SolAR-Modules/job/SolARModuleOpenCV/job/develop/)

**SolARModuleOpenCV** is a module based on [OpenCV](https://opencv.org/), which is a library of programming functions mainly aimed at real-time computer vision. The library is cross-platform and free for use under the open-source [BSD license](http://www.linfo.org/bsdlicense.html).

OpenCV has a modular structure, which means that the package includes several shared or static libraries. The following modules are available:

- **Core functionality (core)** - a compact module defining basic data structures, including the dense multi-dimensional array Mat and basic functions used by all other modules.
- **Image Processing (imgproc)** - an image processing module that includes linear and non-linear image filtering, geometrical image transformations (resize, affine and perspective warping, generic table-based remapping), color space conversion, histograms, and so on.
- **Video Analysis (video)** - a video analysis module that includes motion estimation, background subtraction, and object tracking algorithms.
- **Camera Calibration and 3D Reconstruction (calib3d)** - basic multiple-view geometry algorithms, single and stereo camera calibration, object pose estimation, stereo correspondence algorithms, and elements of 3D reconstruction.
- **2D Features Framework (features2d)** - salient feature detectors, descriptors, and descriptor matchers.
- **Object Detection (objdetect)** - detection of objects and instances of the predefined classes (for example, faces, eyes, mugs, people, cars, and so on).
- **High-level GUI (highgui)** - an easy-to-use interface to simple UI capabilities.
- **Video I/O (videoio)** - an easy-to-use interface to video capturing and video codecs.
- ... some other helper modules, such as FLANN and Google test wrappers, Python bindings, and others.
