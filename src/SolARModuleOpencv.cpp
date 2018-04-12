/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include <iostream>

#include "ContainerFactory.h"

#include "SolAR2DOverlayOpencv.h"
#include "SolAR3DOverlayOpencv.h"
#include "SolARBasicMatchesFilterOpencv.h"
#include "SolARCameraCalibrationOpencv.h"
#include "SolARCameraOpencv.h"
#include "SolARContoursExtractorOpencv.h"
#include "SolARContoursFilterBinaryMarkerOpencv.h"
#include "SolARDescriptorMatcherHammingBruteForceOpencv.h"
#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARDescriptorMatcherRadiusOpencv.h"
#include "SolARDescriptorsExtractorAKAZEOpencv.h"
#include "SolARDescriptorsExtractorAKAZE2Opencv.h"
#include "SolARDescriptorsExtractorORBOpencv.h"
#include "SolARDescriptorsExtractorSBPatternOpencv.h"
#include "SolARFundamentalMatrixDecompositionValidationOpencv.h"
#include "SolARFundamentalMatrixEstimationOpencv.h"
#include "SolARGeometricMatchesFilterOpencv.h"
#include "SolARHomographyEstimationOpencv.h"
#include "SolARImageConvertorOpencv.h"
#include "SolARImageFilterOpencv.h"
#include "SolARImageLoaderOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARKeypointDetectorOpencv.h"
#include "SolARMarker2DNaturalImageOpencv.h"
#include "SolARMarker2DSquaredBinaryOpencv.h"
#include "SolARPerspectiveControllerOpencv.h"
#include "SolARPoseEstimationOpencv.h"
#include "SolARSideBySideOverlayOpencv.h"
#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolARSVDTriangulationOpencv.h"

namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_CONTAINER("15e1990b-86b2-445c-8194-0cbe80ede970", "SolARModuleOpenCV")

extern "C" XPCF_EXPORT_API void XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{

    boost::uuids::uuid uuidOf_XPCF_CID_SolAR2DOverlayOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolAR3DOverlayOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolAR3DOverlayOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARBasicMatchesFilterOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARBasicMatchesFilterOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARCameraCalibrationOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARCameraOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARCameraOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARContoursExtractorOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARContoursFilterBinaryMarkerOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARDescriptorMatcherHammingBruteForceMatcherOpencv  = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARDescriptorMatcherKNNOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARDescriptorMatcherRadiusOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARDescriptorExtractorAKAZE2Opencv  = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARDescriptorExtractorAKAZEOpencv  = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARDescriptorsExtractorORBOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorORBOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARDescriptorsExtractorSBPatternOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARFundamentalMatrixDecompositionValidationOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARFundamentalMatrixDecompositionValidationOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARFundamentalMatrixEstimationOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARFundamentalMatrixEstimationOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARGeometricMatchesFilterOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARHomographyEstimationOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARHomographyEstimationOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARImageConvertorOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARImageConvertorOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARImageFilterOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARImageFilterOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARImageLoaderOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARImageLoaderOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARImageViewerOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARImageViewerOpencv::UUID );  
    boost::uuids::uuid uuidOf_XPCF_CID_SolARKeypointDetectorOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARMarker2DNaturalImageOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARMarker2DNaturalImageOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARMarker2DSquaredBinaryOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARMarker2DSquaredBinaryOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARPerspectiveControllerOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARPerspectiveControllerOpencv::UUID );   
    boost::uuids::uuid uuidOf_XPCF_CID_SolARPoseEstimationOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARPoseEstimationOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARSideBySideOverlayOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARSideBySideOverlayOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARSVDFundamentalMatrixDecomposerOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv::UUID );
    boost::uuids::uuid uuidOf_XPCF_CID_SolARSVDTriangulationOpencv = xpcf::toUUID(SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv::UUID );


    if (componentUUID==uuidOf_XPCF_CID_SolAR2DOverlayOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolAR3DOverlayOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolAR3DOverlayOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARBasicMatchesFilterOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARBasicMatchesFilterOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARCameraCalibrationOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARCameraOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARCameraOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARContoursExtractorOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARContoursFilterBinaryMarkerOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARDescriptorMatcherHammingBruteForceMatcherOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARDescriptorMatcherKNNOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARDescriptorMatcherRadiusOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARDescriptorExtractorAKAZE2Opencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARDescriptorExtractorAKAZEOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARDescriptorsExtractorORBOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorORBOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARDescriptorsExtractorSBPatternOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARFundamentalMatrixDecompositionValidationOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARFundamentalMatrixDecompositionValidationOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARFundamentalMatrixEstimationOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARFundamentalMatrixEstimationOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARGeometricMatchesFilterOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARHomographyEstimationOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARHomographyEstimationOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARImageConvertorOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARImageConvertorOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARImageFilterOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARImageFilterOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARImageLoaderOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARImageLoaderOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARImageViewerOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARImageViewerOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARKeypointDetectorOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARMarker2DNaturalImageOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARMarker2DNaturalImageOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARMarker2DSquaredBinaryOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARMarker2DSquaredBinaryOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARPerspectiveControllerOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARPerspectiveControllerOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARPoseEstimationOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARPoseEstimationOpencv>(interfaceRef);
    }  
    else if (componentUUID==uuidOf_XPCF_CID_SolARSideBySideOverlayOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARSideBySideOverlayOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARSVDFundamentalMatrixDecomposerOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv>(interfaceRef);
    }
    else if (componentUUID==uuidOf_XPCF_CID_SolARSVDTriangulationOpencv)
    {
        xpcf::ComponentFactory::createComponent<SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv>(interfaceRef);
    }
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv::UUID,"Component SolAR2DOverlayOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolAR3DOverlayOpencv::UUID,"Component SolAR3DOverlayOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARBasicMatchesFilterOpencv::UUID,"Component SolARBasicMatchesFilterOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv::UUID,"Component SolARGeometricMatchesFilterOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv::UUID,"Component SolARCameraCalibrationOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARCameraOpencv::UUID,"Component SolARCameraOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv::UUID,"Component SolARContoursExtractorOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv::UUID,"Component SolARContoursFilterBinaryMarkerOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv::UUID,"Component SolARDescriptorMatcherHammingBruteForceOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv::UUID,"Component SolARDescriptorMatcherKNNOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv::UUID,"Component SolARDescriptorMatcherRadiusOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv::UUID,"Component SolARDescriptorsExtractorAKAZE2Opencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv::UUID,"Component SolARDescriptorsExtractorAKAZEOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorORBOpencv::UUID,"Component SolARDescriptorsExtractorORBOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv::UUID,"Component SolARDescriptorsExtractorSBPatternOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARFundamentalMatrixDecompositionValidationOpencv::UUID,"Component SolARFundamentalMatrixDecompositionValidationOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARFundamentalMatrixEstimationOpencv::UUID,"Component SolARFundamentalMatrixEstimationOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv::UUID,"Component SolARGeometricMatchesFilterOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARHomographyEstimationOpencv::UUID,"Component SolARHomographyEstimationOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageConvertorOpencv::UUID,"Component SolARImageConvertorOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageFilterOpencv::UUID,"Component SolARImageFilterOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageLoaderOpencv::UUID,"Component SolARImageLoaderOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageViewerOpencv::UUID,"Component SolARImageViewerOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv::UUID,"Component SolARKeypointDetectorOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARMarker2DNaturalImageOpencv::UUID,"Component SolARMarker2DNaturalImageOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARMarker2DSquaredBinaryOpencv::UUID,"Component SolARMarker2DSquaredBinaryOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARPerspectiveControllerOpencv::UUID,"Component SolARPerspectiveControllerOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARPoseEstimationOpencv::UUID,"Component SolARPoseEstimationOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARSideBySideOverlayOpencv::UUID,"Component SolARSideBySideOverlayOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv::UUID,"Component SolARSVDFundamentalMatrixDecomposerOpencv")
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv::UUID,"Component SolARSVDTriangulationOpencv")
XPCF_END_COMPONENTS_DECLARATION
