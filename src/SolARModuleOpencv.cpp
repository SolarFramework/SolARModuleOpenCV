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

#include "xpcf/module/ModuleFactory.h"
#include "SolARModuleOpencv_traits.h"

#include "SolAR2DOverlayOpencv.h"
#include "SolAR3DOverlayBoxOpencv.h"
#include "SolARCameraCalibrationOpencv.h"
#include "SolARCameraOpencv.h"
#include "SolARContoursExtractorOpencv.h"
#include "SolARContoursFilterBinaryMarkerOpencv.h"
#include "SolARDescriptorMatcherHammingBruteForceOpencv.h"
#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARDescriptorMatcherRadiusOpencv.h"
#include "SolARDescriptorMatcherGeometricOpencv.h"
#include "SolARDescriptorMatcherRegionOpencv.h"
#include "SolARDescriptorsExtractorSIFTOpencv.h"
#include "SolARDescriptorsExtractorAKAZEOpencv.h"
#include "SolARDescriptorsExtractorAKAZE2Opencv.h"
#include "SolARDescriptorsExtractorORBOpencv.h"
#include "SolARDescriptorsExtractorSBPatternOpencv.h"
#include "SolARDescriptorsExtractorFromImageOpencv.h"
#include "SolARFiducialMarkerLoaderOpencv.h"
#include "SolARQRCodeLoaderOpencv.h"
#include "SolARQRCodePoseEstimatorOpencv.h"
#include "SolARQRCodesDetectorOpencv.h"
#include "SolARMultiQRCodesPoseEstimatorOpencv.h"
#include "SolARFundamentalMatrixEstimationOpencv.h"
#include "SolARGeometricMatchesFilterOpencv.h"
#include "SolARHomographyEstimationOpencv.h"
#include "SolARHomographyMatrixDecomposerOpencv.h"
#include "SolARImageConvertorOpencv.h"
#include "SolARImageConvertorUnity.h"
#include "SolARImageFilterBinaryOpencv.h"
#include "SolARImageFilterAdaptiveBinaryOpencv.h"
#include "SolARImageFilterBlurOpencv.h"
#include "SolARImageFilterDilateOpencv.h"
#include "SolARImageFilterErodeOpencv.h"
#include "SolARImageFilterWallisOpencv.h"
#include "SolARImageLoaderOpencv.h"
#include "SolARImageMarkerLoaderOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARKeypointDetectorOpencv.h"
#include "SolARKeypointDetectorRegionOpencv.h"
#include "SolAROpticalFlowPyrLKOpencv.h"
#include "SolARPerspectiveControllerOpencv.h"
#include "SolARProjectOpencv.h"
#include "SolARUnprojectPlanarPointsOpencv.h"
#include "SolARPoseEstimationPlanarPointsOpencv.h"
#include "SolARPoseEstimationPnpOpencv.h"
#include "SolARPoseEstimationSACPnpOpencv.h"
#include "SolARPoseFinderFrom2D2DOpencv.h"
#include "SolARMatchesOverlayOpencv.h"
#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolARSVDTriangulationOpencv.h"
#include "SolAR2D3DcorrespondencesFinderOpencv.h"
#include "SolARUndistortPointsOpencv.h"
#include "SolARVideoAsCameraOpencv.h"
#include "SolARImagesAsCameraOpencv.h"
#include "SolARDeviceDataLoader.h"
#include "SolARMapFusionOpencv.h"
#include "SolARCornerRefinementOpencv.h"
#include "SolARStereoCalibrationOpencv.h"
#include "SolARStereoDescriptorMatcherOpencv.h"
#include "SolARStereo2DPointsRectificationOpencv.h"
#include "SolARStereoImageRectificationOpencv.h"
#include "SolARYOLACTSegmentationOpencv.h"
#include "SolARMaskOverlayOpencv.h"
#include "SolARFCNSegmentationOpencv.h"
#include "SolARGenericSemanticSegmentationOpencv.h"

namespace xpcf=org::bcom::xpcf;

#ifdef WITHCUDA
XPCF_DECLARE_MODULE("fde83ee6-3e22-4531-9e1c-54cb0a3bf3fa", "SolARModuleOpenCVCuda", "SolARModuleOpenCVCuda module description");
#else
XPCF_DECLARE_MODULE("15e1990b-86b2-445c-8194-0cbe80ede970", "SolARModuleOpenCV", "SolARModuleOpenCV module description");
#endif // WITHCUDA

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv>(componentUUID,interfaceRef);
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolAR3DOverlayBoxOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARCameraOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherGeometricOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorMatcherRegionOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSIFTOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorORBOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorFromImageOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARFiducialMarkerLoaderOpencv>(componentUUID,interfaceRef);
    }
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARQRCodeLoaderOpencv>(componentUUID, interfaceRef);
	}
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARQRCodePoseEstimatorOpencv>(componentUUID, interfaceRef);
	}
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARQRCodesDetectorOpencv>(componentUUID, interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARMultiQRCodesPoseEstimatorOpencv>(componentUUID, interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARFundamentalMatrixEstimationOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARHomographyEstimationOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARHomographyMatrixDecomposerOpencv>(componentUUID,interfaceRef);
    }    
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageConvertorOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageConvertorUnity>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageFilterBinaryOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageFilterAdaptiveBinaryOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageFilterBlurOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageFilterDilateOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageFilterErodeOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageFilterWallisOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageLoaderOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageMarkerLoaderOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImageViewerOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARKeypointDetectorRegionOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolAROpticalFlowPyrLKOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARPerspectiveControllerOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARProjectOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARUnprojectPlanarPointsOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARPoseEstimationPlanarPointsOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARPoseEstimationPnpOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARPoseEstimationSACPnpOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARPoseFinderFrom2D2DOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARMatchesOverlayOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolAR2D3DCorrespondencesFinderOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARVideoAsCameraOpencv>(componentUUID,interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARImagesAsCameraOpencv>(componentUUID,interfaceRef);
    }
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARDeviceDataLoader>(componentUUID, interfaceRef);
	}
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARMapFusionOpencv>(componentUUID, interfaceRef);
	}
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARUndistortPointsOpencv>(componentUUID, interfaceRef);
	}
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARCornerRefinementOpencv>(componentUUID, interfaceRef);
	}
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARStereoCalibrationOpencv>(componentUUID, interfaceRef);
	}
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
		errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARStereoDescriptorMatcherOpencv>(componentUUID, interfaceRef);
	}
	if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
	{
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARStereo2DPointsRectificationOpencv>(componentUUID, interfaceRef);
	}
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARStereoImageRectificationOpencv>(componentUUID, interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARYOLACTSegmentationOpencv>(componentUUID, interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARMaskOverlayOpencv>(componentUUID, interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARFCNSegmentationOpencv>(componentUUID, interfaceRef);
    }
    if (errCode != xpcf::XPCFErrorCode::_SUCCESS)
    {
        errCode = xpcf::tryCreateComponent<SolAR::MODULES::OPENCV::SolARGenericSemanticSegmentationOpencv>(componentUUID, interfaceRef);
    }
    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolAR3DOverlayBoxOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARCameraOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherGeometricOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRegionOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSIFTOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorORBOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorFromImageOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARFiducialMarkerLoaderOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARQRCodeLoaderOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARQRCodePoseEstimatorOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARQRCodesDetectorOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARMultiQRCodesPoseEstimatorOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARFundamentalMatrixEstimationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARHomographyEstimationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARHomographyMatrixDecomposerOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageConvertorOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageConvertorUnity)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageFilterBinaryOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageFilterAdaptiveBinaryOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageFilterBlurOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageFilterDilateOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageFilterErodeOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageFilterWallisOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageLoaderOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageMarkerLoaderOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImageViewerOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARKeypointDetectorRegionOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolAROpticalFlowPyrLKOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARPerspectiveControllerOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARProjectOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARUnprojectPlanarPointsOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARPoseEstimationPlanarPointsOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARPoseEstimationPnpOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARPoseEstimationSACPnpOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARPoseFinderFrom2D2DOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARMatchesOverlayOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolAR2D3DCorrespondencesFinderOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARVideoAsCameraOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARImagesAsCameraOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARDeviceDataLoader)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARMapFusionOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARUndistortPointsOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARCornerRefinementOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARStereoCalibrationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARStereoDescriptorMatcherOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARStereo2DPointsRectificationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARStereoImageRectificationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARYOLACTSegmentationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARMaskOverlayOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARFCNSegmentationOpencv)
XPCF_ADD_COMPONENT(SolAR::MODULES::OPENCV::SolARGenericSemanticSegmentationOpencv)
XPCF_END_COMPONENTS_DECLARATION
