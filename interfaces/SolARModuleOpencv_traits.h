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

#ifndef SOLARMODULEOPENCV_TRAITS_H
#define SOLARMODULEOPENCV_TRAITS_H

#include "xpcf/core/traits.h"

namespace SolAR {
namespace MODULES {
/**
 * @namespace SolAR::MODULES::OPENCV
 * @brief <B>Provides a set of computer vision components based on OpenCV library: https://opencv.org/</B>
 * <TT>UUID: 15e1990b-86b2-445c-8194-0cbe80ede970</TT>
 *
 */
namespace OPENCV {
class SolAR2DOverlayOpencv;
class SolAR3DOverlayBoxOpencv;
class SolARCameraCalibrationOpencv;
class SolARCameraOpencv;
class SolARContoursExtractorOpencv;
class SolARContoursFilterBinaryMarkerOpencv;
class SolARDescriptorMatcherHammingBruteForceOpencv;
class SolARDescriptorMatcherKNNOpencv;
class SolARDescriptorMatcherRadiusOpencv;
class SolARDescriptorsExtractorAKAZE2Opencv;
class SolARDescriptorsExtractorAKAZEOpencv;
class SolARDescriptorsExtractorORBOpencv;
class SolARDescriptorsExtractorSBPatternOpencv;
class SolARFundamentalMatrixEstimationOpencv;
class SolARGeometricMatchesFilterOpencv;
class SolARHomographyEstimationOpencv;
class SolARHomographyMatrixDecomposerOpencv;
class SolARImageConvertorOpencv;
class SolARImageConvertorUnity;
class SolARImageFilterBinaryOpencv;
class SolARImageFilterAdaptiveBinaryOpencv;
class SolARImageFilterBlurOpencv;
class SolARImageFilterDilateOpencv;
class SolARImageFilterErodeOpencv;
class SolARImageLoaderOpencv;
class SolARImageViewerOpencv;
class SolARImagesAsCameraOpencv;
class SolARKeypointDetectorOpencv;
class SolARKeypointDetectorRegionOpencv;
class SolARMarker2DNaturalImageOpencv;
class SolARMarker2DSquaredBinaryOpencv;
class SolARMatchesOverlayOpencv;
class SolAROpticalFlowPyrLKOpencv;
class SolARPerspectiveControllerOpencv;
class SolARProjectOpencv;
class SolARUnprojectPlanarPointsOpencv;
class SolARPoseEstimationPlanarPointsOpencv;
class SolARPoseEstimationPnpOpencv;
class SolARPoseEstimationSACPnpOpencv;
class SolARPoseFinderFrom2D2DOpencv;
class SolARSVDFundamentalMatrixDecomposerOpencv;
class SolARSVDTriangulationOpencv;
class SolAR2D3DCorrespondencesFinderOpencv;
class SolARUndistortPointsOpencv;
class SolARVideoAsCameraOpencv;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolAR2D3DCorrespondencesFinderOpencv,
                             "cedd8c47-e7b0-47bf-abb1-7fb54d198117",
                             "SolAR2D3DCorrespondencesFinderOpencv",
                             "Finds the 3D correspondents of 2D keypoints.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv,
                             "cc51d685-9797-4ffd-a9dd-cec4f367fa6a",
                             "SolAR2DOverlayOpencv",
                             "Draws 2D features (circles, lines, etc.) on an image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolAR3DOverlayBoxOpencv,
                             "2db01f59-9793-4cd5-8e13-b25d0ed5735b",
                             "SolAR3DOverlayBoxOpencv",
                             "Draws a 3D box on an image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv,
                             "702a7f53-e5ec-45d2-887d-daa99a34a33c",
                             "SolARCameraCalibrationOpencv",
                             "Calibrates a camera based on a chessboard.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARCameraOpencv,
                             "5B7396F4-A804-4F3C-A0EB-FB1D56042BB4",
                             "SolARCameraOpencv",
                             "Grabs current image captured by a RGB camera.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARContoursExtractorOpencv,
                             "6acf8de2-cc63-11e7-abc4-cec278b6b50a",
                             "SolARContoursExtractorOpencv",
                             "Extracts the contours of a given image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv,
                             "4309dcc6-cc73-11e7-abc4-cec278b6b50a",
                             "SolARContoursFilterBinaryMarkerOpencv",
                             "Filters contours to select only the contours of squared binary markers.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv,
                             "d67ce1ba-04a5-43bc-a0f8-e0c3653b32c9",
                             "SolARDescriptorMatcherHammingBruteForceOpencv",
                             "Matches descriptors based on a Hamming distance and select the best matches of each descriptor.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv,
                             "7823dac8-1597-41cf-bdef-59aa22f3d40a",
                             "SolARDescriptorMatcherKNNOpencv",
                             "Matches descriptors and selects k best matches for each descriptor.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv,
                             "549f7873-96e4-4eae-b4a0-ae8d80664ce5",
                             "SolARDescriptorMatcherRadiusOpencv",
                             "Matches descriptors and selects all matches not farther than a specified distance.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv,
                             "21238c00-26dd-11e8-b467-0ed5f89f718b",
                             "SolARDescriptorsExtractorAKAZE2Opencv",
                             "Extracts the AKAZE descriptors for a set of keypoints (optimized version).")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv,
                             "c8cc68db-9abd-4dab-9204-2fe4e9d010cd",
                             "SolARDescriptorsExtractorAKAZEOpencv",
                             "Extracts the AKAZE descriptors for a set of keypoints.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorORBOpencv,
                             "0ca8f7a6-d0a7-11e7-8fab-cec278b6b50a",
                             "SolARDescriptorsExtractorORBOpencv",
                             "Extracts the ORB descriptors for a set of keypoints.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv,
                             "d25625ba-ce3a-11e7-abc4-cec278b6b50a",
                             "SolARDescriptorsExtractorSBPatternOpencv",
                             "Extracts the descriptor corresponding to a squared binary marker pattern.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARFundamentalMatrixEstimationOpencv,
                             "79b29b50-cf4d-441e-b5de-1de829b91c41",
                             "SolARFundamentalMatrixEstimationOpencv",
                             "Estimates the fundamental matrix from two set of keypoints that match together.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv,
                             "3731691e-2c4c-4d37-a2ce-06d1918f8d41",
                             "SolARGeometricMatchesFilterOpencv",
                             "Filters a set of matches based on geometric constraints.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARHomographyEstimationOpencv,
                             "fb9dac20-2a44-44b2-aa42-2871eec31427",
                             "SolARHomographyEstimationOpencv",
                             "Estimates the homography between two images from their matching keypoints.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARHomographyMatrixDecomposerOpencv,
                             "b5fab395-2184-4123-b0d5-4af74d0a2d79",
                             "SolARHomographyMatrixDecomposerOpencv",
                             "Decomposes a homography matrix to extract four possible 3D poses.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageConvertorOpencv,
                             "fd7fb607-144f-418c-bcf2-f7cf71532c22",
                             "SolARImageConvertorOpencv",
                             "Converts an image according to a given expected layout.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageConvertorUnity,
                             "65282fb3-6651-4e73-b532-5a64ade0ead0",
                             "SolARImageConvertorUnity",
                             "Converts an image to be compliant with Unity image format and layout.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageFilterBinaryOpencv,
                             "e5fd7e9a-fcae-4f86-bfc7-ea8584c298b2",
                             "SolARImageFilterBinaryOpencv",
                             "Filters an image to a binary image based on a unique threshold.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageFilterAdaptiveBinaryOpencv,
                             "901e7a07-5013-4907-be41-0259fca3726c",
                             "SolARImageFilterAdaptiveBinaryOpencv",
                             "Filters a greyscale image to a binary image based on an adaptive threshold.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageFilterBlurOpencv,
                             "deb083aa-69fb-409a-af94-151d476de922",
                             "SolARImageFilterBlurOpencv",
                             "Blurs an image using the normalized box filter.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageFilterDilateOpencv,
                             "7ac9d1b8-afda-4c99-b8df-92e71015a3be",
                             "SolARImageFilterDilateOpencv",
                             "Dilates the white regions of a binary image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageFilterErodeOpencv,
                             "58b09819-64bc-4a80-b6a2-9fe7b179f3fc",
                             "SolARImageFilterErodeOpencv",
                             "Erodes the white regions of a binary image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageLoaderOpencv,
                             "e42d6526-9eb1-4f8a-bb68-53e06f09609c",
                             "SolARImageLoaderOpencv",
                             "Loads an image from a file.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImagesAsCameraOpencv,
                             "b8a8b963-ba55-4ea4-b045-d9e7e8f6db02",
                             "SolARImagesAsCameraOpencv",
                             "Loads an image sequence stored in a dedicated folder.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARImageViewerOpencv,
                             "19ea4e13-7085-4e3f-92ca-93f200ffb01b",
                             "SolARImageViewerOpencv",
                             "Displays an image in a dedicated window.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv,
                             "e81c7e4e-7da6-476a-8eba-078b43071272",
                             "SolARKeypointDetectorOpencv",
                             "Detects keypoints in an image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARKeypointDetectorRegionOpencv,
                             "22c2ca9f-e43b-4a88-8337-4a166a789971",
                             "SolARKeypointDetectorRegionOpencv",
                             "Detects keypoints in an given region of an image.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARMarker2DNaturalImageOpencv,
                             "efcdb590-c570-11e7-abc4-cec278b6b50a",
                             "SolARMarker2DNaturalImageOpencv",
                             "Loads a 2D natural image marker from a file.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARMarker2DSquaredBinaryOpencv,
                             "5d2b8da9-528e-4e5e-96c1-f883edcf3b1c",
                             "SolARMarker2DSquaredBinaryOpencv",
                             "Loads a 2D squared binary marker from a file.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARMatchesOverlayOpencv,
                             "e95302be-3fe1-44e0-97bf-a98380464af9",
                             "SolARMatchesOverlayOpencv",
                             "Displays matching keypoints between two images.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolAROpticalFlowPyrLKOpencv,
                             "b513e9ff-d2e7-4dcf-9a29-4ed95c512158",
                             "SolAROpticalFlowPyrLKOpencv",
                             "Estimates the optical flow beteen two images based on a pyramidal Lucas Kanade approach.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARPerspectiveControllerOpencv,
                             "9c960f2a-cd6e-11e7-abc4-cec278b6b50a",
                             "SolARPerspectiveControllerOpencv",
                             "Extracts an unwrapped image from a specific region defined with 4 points.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARPoseEstimationPlanarPointsOpencv,
                             "9fbadf80-251f-4160-94f8-a64dc3d40a2f",
                             "SolARPoseEstimationPlanarPointsEPFL",
                             "Finds the camera pose of 2D-3D planar points correspondances based on opencv homography.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARPoseEstimationPnpOpencv,
                             "0753ade1-7932-4e29-a71c-66155e309a53",
                             "SolARPoseEstimationPnpOpencv",
                             "Finds the camera pose of 2D-3D points correspondaces based on opencv pnp algorithm.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARPoseEstimationSACPnpOpencv,
                             "4d369049-809c-4e99-9994-5e8167bab808",
                             "SolARPoseEstimationSACPnpOpencv",
                             "Finds the camera pose of 2D-3D points correspondaces based on opencv pnp algorithm using Ransac method.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARPoseFinderFrom2D2DOpencv,
                             "52babb5e-9d33-11e8-98d0-529269fb1459",
                             "SolARPoseFinderFrom2D2DOpencv",
                             "Finds the camera pose based on a 2D-2D points correspondances between two images.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARProjectOpencv,
                             "741fc298-0149-4322-a7a9-ccb971e857ba",
                             "SolARProjectOpencv",
                             "Projects a set of 3D points on a 2D image plane.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv,
                             "31188e79-6bd5-43df-9633-6d6c5d7afb5c",
                             "SolARSVDFundamentalMatrixDecomposerOpencv",
                             "Decomposes Fundamental matrix on a set of camera poses based on opencv SVD solver.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv,
                             "85274ecd-2914-4f12-96de-37c6040633a4",
                             "SolARSVDTriangulationOpencv",
                             "Triangulates set of corresponding 2D-2D points correspondances with known respective camera poses based on opencv SVD.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARUndistortPointsOpencv,
                             "d926e249-8b7f-46e0-8cbd-f981ceb8f921",
                             "SolARUndistortPoints",
                             "Undistorts a set of points according to the distortion matrix of a camera.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARUnprojectPlanarPointsOpencv,
                             "9938354d-6476-437e-8325-97e82666a46e",
                             "SolARUnprojectPlanarPointsOpencv",
                             "Recovers 3D points defined in world coordinate system from a set of 2D points defined in the image coordinate system.")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::OPENCV::SolARVideoAsCameraOpencv,
                             "fa4a780a-9720-11e8-9eb6-529269fb1459",
                             "SolARVideoAsCameraOpencv",
                             "Grabs the images from a video file.")

#endif // SOLARMODULEOPENCV_TRAITS_H
