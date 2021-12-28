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
#include <string>
#include <vector>
#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "api/input/files/ITrackableLoader.h"
#include "datastructure/FiducialMarker.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf  = org::bcom::xpcf;

int main(int argc,char* argv[])
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    try {

        /* instantiate component manager*/
        /* this is needed in dynamic mode */
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_FiducialMarker_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_FiducialMarker_conf.xml")
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");

        SRef<input::files::ITrackableLoader> markerLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();

        SRef<Trackable> trackable;
        if (markerLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("Cannot load marker file with path {}", markerLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
            return 0;
        }

        // Display the world size of the marker in the console
        if (trackable->getType() == TrackableType::FIDUCIAL_MARKER)
        {
            SRef<FiducialMarker> fiducialMarker = xpcf::utils::dynamic_pointer_cast<FiducialMarker>(trackable);

            LOG_INFO("Marker size width: {} ",fiducialMarker->getWidth());
            LOG_INFO("Marker size height: {}",fiducialMarker->getHeight());

            // Display the pattern information in the console, w=white=1 case, b=black=0 case.
            LOG_INFO("Marker pattern: \n{}", fiducialMarker->getPattern().getPatternMatrix());
        }
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }

    return 0;
}



