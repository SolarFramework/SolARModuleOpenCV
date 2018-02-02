#include <iostream>
#include <string>
#include "api/image/IImageLoader.h"
#include "IComponentManager.h"

#define BOOST_TEST_MODULE SolARModuleOpenCVUnitTests
#include <boost/test/unit_test.hpp>


using namespace std;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

BOOST_AUTO_TEST_CASE(TestLoadImage)
{
    FrameworkReturnCode result;
    boost::uuids::string_generator gen;

    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();;


    xpcfComponentManager->load() ;
    BOOST_TEST(xpcfComponentManager->isLoaded(),"SOLAR ERROR: SolARModuleOpencv xml registry file should be opened");

	int res=0;
    
    // imageLoader introspection
    sptrnms::shared_ptr<image::IImageLoader> imageLoader;
    res=xpcfComponentManager->createComponent(gen("E42D6526-9EB1-4F8A-BB68-53E06F09609C"), gen(image::IImageLoader::UUID), imageLoader);

    BOOST_CHECK( imageLoader != NULL);
    SRef<Image> inputImage;
    result=imageLoader->loadImage("test.jpg", inputImage);
    // BOOST_TEST_MESSAGE ("result is:" << static_cast<int>(result));  // to be displayed with log_level=message

    BOOST_TEST( static_cast<int>(result)==static_cast<int>(FrameworkReturnCode::_SUCCESS),"SOLAR ERROR: Load Image should return _SUCCESS");

}

BOOST_AUTO_TEST_CASE(TestLoadImageInexistante)
{
    FrameworkReturnCode result;
    boost::uuids::string_generator gen;

    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();;


    xpcfComponentManager->load() ;
    BOOST_TEST(xpcfComponentManager->isLoaded(),"SOLAR ERROR: SolARModuleOpencv xml registry file should be opened");

    int res=0;
    
    // imageLoader introspection
    sptrnms::shared_ptr<image::IImageLoader> imageLoader;
    res=xpcfComponentManager->createComponent(gen("E42D6526-9EB1-4F8A-BB68-53E06F09609C"), gen(image::IImageLoader::UUID), imageLoader);

    BOOST_TEST(( imageLoader != NULL),"SOLAR ERROR: createComponent should not return null pointer for imageLoader");

    SRef<Image> inputImage;
    result=imageLoader->loadImage("test2.jpg", inputImage);
    BOOST_TEST( static_cast<int>(result)==static_cast<int>(FrameworkReturnCode::_ERROR_LOAD_IMAGE),"SOLAR ERROR: Load Image should return _ERROR_LOAD_IMAGE");

}
