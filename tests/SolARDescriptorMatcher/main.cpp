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

#include "api/image/IImageLoader.h"
#include "api/features/IKeypointDetector.h"
#include "api/display/IImageViewer.h"
#include "api/display/IMatchesOverlay.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"
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

    if(xpcfComponentManager->load("SolAROpenCVDescriptorMatcher_config.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file SolAROpenCVDescriptorMatcher_config.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");

    SRef<image::IImageLoader> imageLoaderImage1 = xpcfComponentManager->resolve<image::IImageLoader>("image1");
    SRef<image::IImageLoader> imageLoaderImage2 = xpcfComponentManager->resolve<image::IImageLoader>("image2");

    SRef<features::IKeypointDetector> keypointsDetectorAKAZE = xpcfComponentManager->resolve<features::IKeypointDetector>("AKAZEDetector");
    SRef<features::IKeypointDetector> keypointsDetectorORB = xpcfComponentManager->resolve<features::IKeypointDetector>("ORBDetector");
    SRef<features::IKeypointDetector> keypointsDetectorSIFT = xpcfComponentManager->resolve<features::IKeypointDetector>("SIFTDetector");

    SRef<features::IDescriptorsExtractor> extractorAKAZE = xpcfComponentManager->resolve<features::IDescriptorsExtractor>("AKAZEDesc");
    SRef<features::IDescriptorsExtractor> extractorORB = xpcfComponentManager->resolve<features::IDescriptorsExtractor>("ORBDesc");
    SRef<features::IDescriptorsExtractor> extractorSIFT = xpcfComponentManager->resolve<features::IDescriptorsExtractor>("SIFTDesc");

    SRef<features::IDescriptorMatcher> matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
    SRef<features::IDescriptorMatcher> matcherBinary = xpcfComponentManager->resolve<features::IDescriptorMatcher>("BinaryMatcher");

    SRef<display::IMatchesOverlay> overlay = xpcfComponentManager->resolve<display::IMatchesOverlay>();

    SRef<display::IImageViewer> viewerAKAZE = xpcfComponentManager->resolve<display::IImageViewer>("AKAZEViewer");
    SRef<display::IImageViewer> viewerORB = xpcfComponentManager->resolve<display::IImageViewer>("ORBViewer");
    SRef<display::IImageViewer> viewerSIFT = xpcfComponentManager->resolve<display::IImageViewer>("SIFTViewer");

    if (!imageLoaderImage1  || !imageLoaderImage2 || !keypointsDetectorAKAZE || !keypointsDetectorORB || !keypointsDetectorSIFT || !extractorAKAZE || !extractorORB
     || !extractorSIFT || !matcher || !matcherBinary || !overlay || !viewerAKAZE || !viewerORB || !viewerSIFT )
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    SRef<Image>                     image1;
    SRef<Image>                     image2;
    std::vector<Keypoint>           keypoints1;
    std::vector<Keypoint>           keypoints2;
    SRef<DescriptorBuffer>          descriptors1;
    SRef<DescriptorBuffer>          descriptors2;
    std::vector<DescriptorMatch>    matches;
    std::vector<SRef<Point2Df>>     matchedKeypoints1;
    std::vector<SRef<Point2Df>>     matchedKeypoints2;
    SRef<Image>                     viewerImageAKAZE, viewerImageORB, viewerImageSIFT;

 // Start
    // Get the first image (the path of this image is defined in the conf_DetectorMatcher.xml)
    if (imageLoaderImage1->getImage(image1) != FrameworkReturnCode::_SUCCESS)
    {
        LOG_WARNING("First image {} cannot be loaded", imageLoaderImage1->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
        return 0;
    }

    // Get the second image (the path of this image is defined in the conf_DetectorMatcher.xml)
    if (imageLoaderImage2->getImage(image2) != FrameworkReturnCode::_SUCCESS)
    {
        LOG_WARNING("Second image {} cannot be loaded", imageLoaderImage2->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
        return 0;
    }


    // AKAZE
    // --------
    // Detect the keypoints of the first image
    keypointsDetectorAKAZE->detect(image1, keypoints1);

    // Detect the keypoints of the second image
    keypointsDetectorAKAZE->detect(image2, keypoints2);

    // Compute the AKAZE descriptor for each keypoint extracted from the first image
    extractorAKAZE->extract(image1, keypoints1, descriptors1);

    // Compute the AKAZE descriptor for each keypoint extracted from the second image
    extractorAKAZE->extract(image2, keypoints2, descriptors2);

    // Compute the matches between the keypoints of the first image and the keypoints of the second image
    matcherBinary->match(descriptors1, descriptors2, matches);

    // Draw the matches in a dedicated image
    overlay->draw(image1, image2, viewerImageAKAZE, keypoints1, keypoints2, matches);

    // ORB
    // --------
    // Detect the keypoints of the first image
    keypointsDetectorORB->detect(image1, keypoints1);

    // Detect the keypoints of the second image
    keypointsDetectorORB->detect(image2, keypoints2);

    // Compute the ORB descriptor for each keypoint extracted from the first image
    extractorORB->extract(image1, keypoints1, descriptors1);

    // Compute the ORB descriptor for each keypoint extracted from the second image
    extractorORB->extract(image2, keypoints2, descriptors2);

    // Compute the matches between the keypoints of the first image and the keypoints of the second image
    matcherBinary->match(descriptors1, descriptors2, matches);

    // Draw the matches in a dedicated image
    overlay->draw(image1, image2, viewerImageORB, keypoints1, keypoints2, matches);

    // SIFT
    // --------
    // Detect the keypoints of the first image
    keypointsDetectorSIFT->detect(image1, keypoints1);

    // Detect the keypoints of the second image
    keypointsDetectorSIFT->detect(image2, keypoints2);

    // Compute the SIFT descriptor for each keypoint extracted from the first image
    extractorSIFT->extract(image1, keypoints1, descriptors1);

    // Compute the SIFT descriptor for each keypoint extracted from the second image
    extractorSIFT->extract(image2, keypoints2, descriptors2);

    // Compute the matches between the keypoints of the first image and the keypoints of the second image
    matcher->match(descriptors1, descriptors2, matches);

    // Draw the matches in a dedicated image
    overlay->draw(image1, image2, viewerImageSIFT, keypoints1, keypoints2, matches);

    while (true)
    {
        // Display the image with matches in a viewer. If escape key is pressed, exit the loop.
        if (viewerAKAZE->display(viewerImageAKAZE) == FrameworkReturnCode::_STOP || viewerORB->display(viewerImageORB) == FrameworkReturnCode::_STOP || viewerSIFT->display(viewerImageSIFT) == FrameworkReturnCode::_STOP )
        {
            LOG_INFO("End of DescriptorMatcherOpenCVTest");
            break;
        }
    }

    return 0;
}





