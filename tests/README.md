# SolARModuleOpenCV Tests

[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleTools?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

**SolARModuleOpenCV** is a module that implements some generic features used in vision processing based on the OpenCV library.

## Before running the tests

### Required modules

Some tests require other modules such as OpenGL, OpenCV, FBOW and G20. If they are not yet installed on your machine, please run the following command from the test folder:

<pre><code>remaken install packagedependencies.txt</code></pre>

and for debug mode:

<pre><code>remaken install packagedependencies.txt -c debug</code></pre>

For more information about how to install remaken on your machine, visit the [install page](https://solarframework.github.io/install/) on the SolAR website.

### AR device captures

The SolARTest_ModuleOpenCV_DeviceDualMarkerCalibration & SolARTest_ModuleOpenCV_DeviceDataLoader require AR device captures containing both an image sequence and the corresponding poses. You can use two captures available on the solar artifactory:

* <strong>Loop_Desktop_A</strong>: A video sequence captured with a Hololens 1 around a desktop starting and finishing with the fiducial Marker A with a loop trajectory. A fiducial marker B is captured during the trajectory.

Download the video sequences [loopDesktopA.zip](https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip) and extract it into the `./data` folder.
