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
#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

#include "api/input/devices/ICamera.h"
#include "api/segm/ISemanticSegmentation.h"
#include "api/display/IMaskOverlay.h"
#include "api/display/IImageViewer.h"
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

        if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_FCNSegmentation_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_FCNSegmentation_conf.xml")
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");

        auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
        auto segmentation = xpcfComponentManager->resolve<segm::ISemanticSegmentation>();
		auto maskOverlay = xpcfComponentManager->resolve<display::IMaskOverlay>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();

		// open camera
		if (camera->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot open camera");
			return -1;
		}

        // pose estimation
		while (true) {
			SRef<Image> image;
			if (camera->getNextImage(image) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				return -1;
			}		
			SRef<Image> mask;
			if (segmentation->segment(image, mask) == FrameworkReturnCode::_SUCCESS)
				maskOverlay->draw(image, mask);
			if (imageViewer->display(image) == FrameworkReturnCode::_STOP)
				break;
		}
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }

    return 0;
}



