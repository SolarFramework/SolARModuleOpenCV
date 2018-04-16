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


/* à ré écrire*/
/*
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

}*/

/* à ré écrire*/
/*
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

}*/


/******************************************************/
/* ce test case est un test case d'exemple à suivre****/
/****** Cas Nominal                               *****/
/******************************************************/
BOOST_AUTO_TEST_CASE(TestCasNominalComposant)
{
    // CMC: prévoir des tests de fonctionnement cas nominal
    //faire appel à xpcf
     //BOOST_TEST(xpcfComponentManager->isLoaded(),"SOLAR ERROR: SolARModuleOpencv xml registry file should be opened");

    //faire appel à la création du composant
     //BOOST_TEST(création d'un composant);

    //vérifier que le composant est conforme
     //BOOST_TEST(verifier que le composant est conforme, non nul);

    // FOR ( chaque méthode du composant ):
    //vérifier un comportement normal du composant : tester ses méthodes
     //BOOST_TEST(verifier que le composant est conforme, non nul);


}
/******************************************************/
/* ce test case est un test case d'exemple à suivre****/
/****** Cas aux limites pour le même composant    *****/
/******************************************************/
BOOST_AUTO_TEST_CASE(TestCasLimiteComposant)
{
    // CMC: prévoir des tests de fonctionnement cas aux limites

    //faire appel à xpcf

    //faire appel à la création du composant
     //BOOST_TEST(création d'un composant);

    //vérifier que le composant est conforme
     //BOOST_TEST(verifier que le composant est conforme, non nul);

    //FOR(  chaque méthode du composant ) :
    //vérifier un comportement anormal du composant exemple: charger une image inexistante

            //FOR (chaque cas d'erreur)
              //BOOST_TEST(verifier que la méthode renvoie la bonne erreur);

}


BOOST_AUTO_TEST_CASE(SolAR2DOVerlay)
{

}
BOOST_AUTO_TEST_CASE(SolAR3DOverlayOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARCameraCalibrationOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARCameraOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARContoursExtractorOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARContoursFilterBinaryMarkerOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARDescriptorMatcherHammingBruteForceOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARDescriptorMatcherKNNOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARDescriptorMatcherRadiusOpencv)
{

}
BOOST_AUTO_TEST_CASE(SolARDescriptorsExtractorAKAZE2Opencv)
{

}

BOOST_AUTO_TEST_CASE(SolARDescriptorsExtractorAKAZEOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARDescriptorsExtractorORBOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARDescriptorsExtractorSBPatternOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARHomographyFinderOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARHomographyEstimationOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARImageConvertorOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARImageFilterOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARImageLoaderOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARImageViewerOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARKeypointDetectorOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARMarker2DNaturalImageOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARMarker2DSquaredBinaryOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARModuleManagerOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolAROpencvAPI)
{

}

BOOST_AUTO_TEST_CASE(SolAROpenCVHelper)
{

}

BOOST_AUTO_TEST_CASE(SolARPerspectiveControllerOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARPoseEstimationOpencv)
{

}

BOOST_AUTO_TEST_CASE(SolARSideBySideOverlayOpencv)
{

}

