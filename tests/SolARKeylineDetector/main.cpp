
#include <xpcf/xpcf.h>
#include <boost/log/core.hpp>
#include <core/Log.h>

#include "SolARModuleOpencv_traits.h"

#include "api/image/IImageLoader.h"
#include "api/input/devices/ICamera.h"
#include "api/features/IKeylineDetector.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IImageViewer.h"

namespace xpcf=org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

#define WEBCAM

/**
 * Declare module.
 */
int main(int argc, char *argv[])
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

		if(xpcfComponentManager->load("SolARKeylineDetector_config.xml") != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file SolARKeylineDetector_config.xml");
			return -1;
		}

		// declare and create components
        LOG_INFO("Start creating components");

		SRef<input::devices::ICamera> camera = xpcfComponentManager->resolve<input::devices::ICamera>();
		SRef<image::IImageLoader> imageLoader = xpcfComponentManager->resolve<image::IImageLoader>();
		SRef<features::IKeylineDetector> keylineDetector = xpcfComponentManager->resolve<features::IKeylineDetector>();
		SRef<display::I2DOverlay> overlay = xpcfComponentManager->resolve<display::I2DOverlay>();
		SRef<display::IImageViewer> viewer = xpcfComponentManager->resolve<display::IImageViewer>();

        LOG_DEBUG("Components created!");

		SRef<Image> image;
		std::vector<Keyline> keylines;

#ifdef WEBCAM
		if (camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start");
			return -1;
		}

		int count = 0;
		clock_t start, end;
		start = clock();

		while (true)
		{
			if (camera->getNextImage(image) == FrameworkReturnCode::_ERROR_)
				break;
			count++;

			keylineDetector->detect(image, keylines);
			overlay->drawLines(keylines, image);

			if (viewer->display(image) == FrameworkReturnCode::_STOP)
			{
				LOG_INFO("End of SolARKeylineDetector test");
				break;
			}
		}
		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frames per second : %8.2f\n\n", count / duration);
#else
		if (imageLoader->getImage(image) !=  FrameworkReturnCode::_SUCCESS)
		{
			LOG_WARNING("Image can't be loaded", imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
			return 0;
		}

        keylineDetector->detect(image, keylines);
        LOG_DEBUG("Detected {} keylines", keylines.size());
        overlay->drawLines(keylines, image);
        LOG_DEBUG("Lines drawn");

        while (true)
        {
                // Display the image with matches in a viewer. If escape key is pressed, exit the loop.
                if (viewer->display(image) == FrameworkReturnCode::_STOP)
                {
                        LOG_INFO("End of SolARKeylineDetector test");
                        break;
                }
        }
#endif
	}
	catch (xpcf::Exception &e)
	{
		LOG_ERROR("{}", e.what());
		return -1;
	}
	return 0;
}
