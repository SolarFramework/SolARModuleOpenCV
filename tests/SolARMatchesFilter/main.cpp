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

#include <iostream>

#include <boost/log/core.hpp>

using namespace std;

#include "xpcf/xpcf.h"
#include "api/image/IImageLoader.h"
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/display/IImageViewer.h"
#include "api/display/IMatchesOverlay.h"
#include "core/Log.h"


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc, char **argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

//    SolARLog::init();
    LOG_ADD_LOG_TO_CONSOLE();

    LOG_INFO("program is running");


    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_MatchesFilter.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_NaturalImageMarker.xml", argv[1])
        return -1;
    }
    // declare and create components
    LOG_INFO("Start creating components");

    SRef<image::IImageLoader>               image1Loader = xpcfComponentManager->resolve<image::IImageLoader>();
    image1Loader->bindTo<xpcf::IConfigurable>()->configure("conf_MatchesFilter.xml", "image1");
    SRef<image::IImageLoader>               image2Loader = xpcfComponentManager->resolve<image::IImageLoader>();
    image2Loader->bindTo<xpcf::IConfigurable>()->configure("conf_MatchesFilter.xml", "image2");
    SRef<features::IKeypointDetector>       keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
    SRef<features::IDescriptorsExtractor>   extractorAKAZE2 = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
    SRef<features::IDescriptorMatcher>      matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
    SRef<features::IMatchesFilter>          matchesFilterGeometric = xpcfComponentManager->resolve<features::IMatchesFilter>();
    SRef<display::IMatchesOverlay>          overlayMatches = xpcfComponentManager->resolve<display::IMatchesOverlay>();
    SRef<display::IImageViewer>             viewerWithoutFilter = xpcfComponentManager->resolve<display::IImageViewer>();
    viewerWithoutFilter->bindTo<xpcf::IConfigurable>()->configure("conf_MatchesFilter.xml","withoutFilter");
    SRef<display::IImageViewer>             viewerWithFilter = xpcfComponentManager->resolve<display::IImageViewer>();
    viewerWithFilter->bindTo<xpcf::IConfigurable>()->configure("conf_MatchesFilter.xml","withFilter");

    /* we need to check that components are well created*/
    if (!image1Loader || !image2Loader || !keypointsDetector || !extractorAKAZE2 || !matcher ||
        !matchesFilterGeometric || !overlayMatches || !viewerWithoutFilter || !viewerWithFilter)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }
    LOG_INFO("All components have been created");

    // Declare data structures used to exchange information between components

    SRef<Image>                     image1;
    SRef<Image>                     image2;

    std::vector<Keypoint>           keypoints1;
    std::vector<Keypoint>           keypoints2;


    SRef<DescriptorBuffer>          descriptors1;
    SRef<DescriptorBuffer>          descriptors2;
    std::vector<DescriptorMatch>    matches;

    SRef<Image>                             matchesImage;
    SRef<Image>                             filteredMatchesImage;

   // Load the first image
   if (image1Loader->getImage(image1) != FrameworkReturnCode::_SUCCESS)
   {
      LOG_ERROR("Cannot load image with path {}", image1Loader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
      return -1;
   }

   // Load the second image
   if (image2Loader->getImage(image2) != FrameworkReturnCode::_SUCCESS)
   {
      LOG_ERROR("Cannot load image with path {}", image2Loader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
      return -1;
   }

   // Detect the keypoints of the first image
   keypointsDetector->detect(image1, keypoints1);

   // Detect the keypoints of the second image
   keypointsDetector->detect(image2, keypoints2);

   // Compute the AKAZE2 descriptor for each keypoint extracted from the first image
   extractorAKAZE2->extract(image1, keypoints1, descriptors1);

   // Compute the AKAZE2 descriptor for each keypoint extracted from the second image
   extractorAKAZE2->extract(image2, keypoints2, descriptors2);

   // Compute the matches between the keypoints of the first image and the keypoints of the second image
   matcher->match(descriptors1, descriptors2, matches);

   LOG_INFO("number of original matches: {}", matches.size());

   // Draw the original matches in a dedicated image
   overlayMatches->draw(image1, image2, matchesImage, keypoints1, keypoints2, matches);

   // Apply geometric filter
   matchesFilterGeometric->filter(matches,matches,keypoints1, keypoints2);

   LOG_INFO("number of filtered matches: {}", matches.size());

   // Draw the filtered matches in a dedicated image
   overlayMatches->draw(image1, image2, filteredMatchesImage, keypoints1, keypoints2, matches);

   while (true){
       if (viewerWithoutFilter->display(matchesImage) == SolAR::FrameworkReturnCode::_STOP ||
           viewerWithFilter->display(filteredMatchesImage) == SolAR::FrameworkReturnCode::_STOP)
           break;
   }

   return 0;
}



