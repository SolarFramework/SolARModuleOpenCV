/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "xpcf/xpcf.h"

#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/display/IImageViewer.h"
#include "api/display/IMatchesOverlay.h"
#include "api/tracking/IOpticalFlowEstimator.h"
#include "core/Log.h"

#include <iostream>
#include <boost/log/core.hpp>
#include <string>
#include <vector>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc,char** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    try {

        if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_OpticalFlow_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_OpticalFlow_conf.xml")
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");

        SRef<input::devices::ICamera> camera = xpcfComponentManager->resolve<input::devices::ICamera>();
        SRef<features::IKeypointDetector> keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
        SRef<tracking::IOpticalFlowEstimator> opticalFlowLK = xpcfComponentManager->resolve<tracking::IOpticalFlowEstimator>();
        SRef<display::IMatchesOverlay> overlay = xpcfComponentManager->resolve<display::IMatchesOverlay>();
        SRef<display::IImageViewer> viewer = xpcfComponentManager->resolve<display::IImageViewer>();


        // declare data structures
        SRef<Image>                 previousImage;
        SRef<Image>                 currentImage;
        std::vector<Keypoint>       keypointsToTrack;
        std::vector<Point2Df>       pointsToTrack;
        std::vector<Point2Df>       trackedPoints;
        std::vector<Point2Df>       points1, points2;
        std::vector<unsigned char>  opticalFlowStatus;
        std::vector<float>          opticalFlowError;
        SRef<Image>                 viewerImage;

     // Start
        // Start the camera
        if (camera->start() != FrameworkReturnCode::_SUCCESS) // Camera
        {
            LOG_ERROR ("Camera cannot start");
            return -1;
        }

        // Capture first image
        if(camera->getNextImage(previousImage)==SolAR::FrameworkReturnCode::_ERROR_)
        {
            LOG_ERROR("Cannot capture next image");
            return -1;
        }


        while (true)
        {
            // capture a new image
            if(camera->getNextImage(currentImage)==SolAR::FrameworkReturnCode::_ERROR_)
            {
                LOG_ERROR("Cannot capture next image");
                return -1;
            }

            // Detect the keypoints of the first image
            keypointsDetector->detect(previousImage, keypointsToTrack);

            bool trackingOK = true;
            int trackingIteration = 0;
            while (trackingOK)
            {
                if (trackingIteration == 0)
                {
                    // Estimate the optical flow between images. Here, with use the keypoint detected previously
                    opticalFlowLK->estimate(previousImage, currentImage, keypointsToTrack, trackedPoints, opticalFlowStatus, opticalFlowError);
                }
                else
                    // Estimate the optical flow between images. here, with use the point previously tracked
                    opticalFlowLK->estimate(previousImage, currentImage, pointsToTrack, trackedPoints, opticalFlowStatus, opticalFlowError);


                // Convert keypoints in point2Df and remove untracked points
                points1.clear();
                points2.clear();
                for (int i = 0; i < trackedPoints.size(); i++)
                {
                    if (opticalFlowStatus[i] == 1)
                    {
                        if (trackingIteration == 0)
                            points1.push_back(Point2Df(keypointsToTrack[i].getX(), keypointsToTrack[i].getY()));
                        else
                            points1.push_back(Point2Df(pointsToTrack[i].getX(), pointsToTrack[i].getY()));
                        points2.push_back(Point2Df(trackedPoints[i].getX(), trackedPoints[i].getY()));
                    }
                }
                // Draw the matches in a dedicated image
                overlay->draw(currentImage, viewerImage, points1, points2);

                // Display the image with matches in a viewer. If escape key is pressed, exit the loop.
                if (viewer->display(viewerImage) == FrameworkReturnCode::_STOP)
                {
                    LOG_INFO("End of OpticalFlowOpenCVTest");
                    return 0;
                }

                previousImage = currentImage;
                pointsToTrack = points2;
                trackingIteration ++;

                if (points1.size() < 50)
                {
                    trackingOK = false;
                    break;
                }

                // capture the ne image
                if(camera->getNextImage(currentImage)==SolAR::FrameworkReturnCode::_ERROR_)
                {
                    LOG_ERROR("Cannot capture next image");
                    return -1;
                }

            }
        }
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

    return 0;
}





