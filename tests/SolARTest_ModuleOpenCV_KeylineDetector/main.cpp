
#include "xpcf/xpcf.h"
#include "core/Log.h"

#include "api/display/I2DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/features/IKeylineDetector.h"
#include "api/image/IImageLoader.h"
#include "api/input/devices/ICamera.h"

#include <boost/log/core.hpp>

namespace xpcf = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

#define WEBCAM 0

/**
 * Declare module.
 */
int main()
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif
	LOG_ADD_LOG_TO_CONSOLE();

	try
	{
		/* instantiate component manager*/
		/* this is needed in dynamic mode */
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

		const char* configFile = "SolARTest_ModuleOpenCV_KeylineDetector_conf.xml";
		if(xpcfComponentManager->load(configFile) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configFile);
			return -1;
		}

		// declare and create components
        LOG_INFO("Start creating components");

#if WEBCAM
		auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
#else
		auto imageLoader = xpcfComponentManager->resolve<image::IImageLoader>();
#endif
		auto keylineDetector = xpcfComponentManager->resolve<features::IKeylineDetector>();
		auto overlay = xpcfComponentManager->resolve<display::I2DOverlay>();
		auto viewer = xpcfComponentManager->resolve<display::IImageViewer>();

        LOG_DEBUG("Components created!");

		SRef<Image> image;
		std::vector<Keyline> keylines;

		int count = 0;
		clock_t start, end;
		start = clock();
#if WEBCAM
		// Init camera
		if (camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start");
			return -1;
		}
		// Main loop, press escape key to exit
		while (true)
		{
			// Read image from camera
			if (camera->getNextImage(image) == FrameworkReturnCode::_ERROR_)
				break;
			count++;
			// Detect keylines in image
			keylineDetector->detect(image, keylines);
			// Draw detected keylines
			overlay->drawLines(keylines, image);
			// Display the image with matches in a viewer. If escape key is pressed, exit the loop.
			if (viewer->display(image) == FrameworkReturnCode::_STOP)
			{
				LOG_INFO("End of SolARKeylineDetector test");
				break;
			}
		}
#else
		if (imageLoader->getImage(image) !=  FrameworkReturnCode::_SUCCESS)
		{
			LOG_WARNING("Image ({}) can't be loaded", imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
			return 0;
		}
		count++;
		// Detect keylines in image
		keylineDetector->detect(image, keylines);
		LOG_INFO("Detected {} lines.", keylines.size());
		// Draw detected keylines
		overlay->drawLines(keylines, image);
        // Display the image with matches in a viewer. If escape key is pressed, exit the loop.
        while (true)
            if (viewer->display(image) == FrameworkReturnCode::_STOP)
                break;
        LOG_INFO("End of SolARKeylineDetector test");
#endif
		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frames per second : %8.2f\n\n", count / duration);
	}
	catch (xpcf::Exception &e)
	{
		LOG_ERROR("{}", e.what());
		return -1;
	}
	return 0;
}
