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

#include "SolARModuleOpencv_traits.h"
#include "api/input/files/IMarker2DSquaredBinary.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
namespace xpcf  = org::bcom::xpcf;

int main(int argc,char* argv[])
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_FiducialMarker.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Marker2DFiducial.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");

    SRef<input::files::IMarker2DSquaredBinary> binaryMarker = xpcfComponentManager->create<SolARMarker2DSquaredBinaryOpencv>()->bindTo<input::files::IMarker2DSquaredBinary>();

    if (binaryMarker->loadMarker() != FrameworkReturnCode::_SUCCESS)
    {
        LOG_ERROR("Cannot load marker file with path {}", binaryMarker->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
        return 0;
    }

    // Display the world size of the marker in the console
    LOG_INFO("Marker size width: {} ",binaryMarker->getWidth());
    LOG_INFO("Marker size height: {}",binaryMarker->getHeight());

    // Display the pattern information in the console, w=white=1 case, b=black=0 case.
    LOG_INFO("Marker pattern: \n{}", binaryMarker->getPattern()->getPatternMatrix());

    return 0;
}



