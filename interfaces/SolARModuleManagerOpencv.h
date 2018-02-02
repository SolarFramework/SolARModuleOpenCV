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

#ifndef SOLARMODULEMANAGEROPENCV_H
#define SOLARMODULEMANAGEROPENCV_H

#include "IComponentManager.h"

#include "api/display/I2DOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/ISideBySideOverlay.h"

#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/features/IKeypointsReIndexer.h"

#include "api/geom/I2DTransform.h"
#include "api/geom/I3DTransform.h"
#include "api/geom/IImage2WorldMapper.h"

#include "api/input/devices/ICamera.h"
#include "api/input/devices/ICameraCalibration.h"
#include "api/input/files/IMarker2DNaturalImage.h"
#include "api/input/files/IMarker2DSquaredBinary.h"

#include "api/solver/pose/IHomographyEstimation.h"
#include "api/solver/pose/IHomographyValidation.h"

#include "api/image/IImageConvertor.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageLoader.h"
#include "api/image/IPerspectiveController.h"


#include "api/solver/pose/IPoseEstimation.h"

#include "SolAROpencvAPI.h"

using namespace std;

namespace xpcf  = org::bcom::xpcf;
namespace SolAR {
namespace MODULES {
namespace OPENCV {
namespace UUID{

// declaration of components UUIDs
const string TRANSFORM2D="a95de484-6d84-4a02-a506-d1ce442e0062";
const string OVERLAY3D="2db01f59-9793-4cd5-8e13-b25d0ed5735b";
const string OVERLAY2D="cc51d685-9797-4ffd-a9dd-cec4f367fa6a";
const string OVERLAYSBS="e95302be-3fe1-44e0-97bf-a98380464af9";
const string TRANSFORM3D="01b51dd7-f651-4fab-bb95-a9eb38ddf15c";
const string CAMERA_CALIBRATION="702a7f53-e5ec-45d2-887d-daa99a34a33c";
const string CAMERA="5B7396F4-A804-4F3C-A0EB-FB1D56042BB4";
const string CONTOURS_EXTRACTOR="6acf8de2-cc63-11e7-abc4-cec278b6b50a";
const string CONTOURS_FILTER_BINARY_MARKER="4309dcc6-cc73-11e7-abc4-cec278b6b50a";
const string DESCRIPTOR_MATCHER_KNN="7823dac8-1597-41cf-bdef-59aa22f3d40a";
const string DESCRIPTOR_MATCHER_RADIUS="904e64f6-d502-11e7-9296-cec278b6b50a";
const string DESCRIPTORS_EXTRACTOR_ORB="0ca8f7a6-d0a7-11e7-8fab-cec278b6b50a";
const string DESCRIPTORS_EXTRACTOR_SBPATTERN="d25625ba-ce3a-11e7-abc4-cec278b6b50a";
const string DESCRIPTORS_EXTRACTOR_SIFT="3787eaa6-d0a0-11e7-8fab-cec278b6b50a";
const string DESCRIPTORS_EXTRACTOR_SURF128="fe14a310-d0a2-11e7-8fab-cec278b6b50a";
const string DESCRIPTORS_EXTRACTOR_SURF64="1a437804-d0a3-11e7-8fab-cec278b6b50a";
const string HOMOGRAPHY_ESTIMATION="fb9dac20-2a44-44b2-aa42-2871eec31427";
const string HOMOGRAPHY_VALIDATION="dcc94624-dd32-11e7-9296-cec278b6b50a";
const string IMAGE2WORLD_MAPPER="d7fee286-5931-4954-b3d1-bb302259c7ef";
const string IMAGE_CONVERTOR="fd7fb607-144f-418c-bcf2-f7cf71532c22";
const string IMAGE_FILTER="fa356d0c-0a53-4722-a7f3-bb92b934d8db";
const string IMAGE_LOADER="E42D6526-9EB1-4F8A-BB68-53E06F09609C";
const string IMAGE_VIEWER="19EA4E13-7085-4E3F-92CA-93F200FFB01B";
const string KEYPOINT_DETECTOR="e81c7e4e-7da6-476a-8eba-078b43071272";
const string KEYPOINTS_REINDEXER="ea0991f8-df18-11e7-80c1-9a214cf093ae";
const string MARKER2D_NATURAL_IMAGE="efcdb590-c570-11e7-abc4-cec278b6b50a";
const string MARKER2D_SQUARED_BINARY="5d2b8da9-528e-4e5e-96c1-f883edcf3b1c";
const string PERSPECTIVE_CONTROLLER="9c960f2a-cd6e-11e7-abc4-cec278b6b50a";
const string POSE_ESTIMATION="0753ade1-7932-4e29-a71c-66155e309a53";
const string SBPATTERN_REINDEXER="46b7dd26-d558-11e7-9296-cec278b6b50a";
}  // End namespace UUID

using namespace SolAR;

// class SolARComponentManagerOpencv declaration
class SOLAROPENCV_EXPORT_API SolARModuleManagerOpencv
{
public:
    SolARModuleManagerOpencv();
    SolARModuleManagerOpencv(const char *iniFile);
    ~SolARModuleManagerOpencv() = default;

    template <class T>
    int createComponent(string uuid, SRef<T> &compRef);

    template <class T>
    SRef<T> createComponent(string uuid);

    inline bool isLoaded() const {return loaded;}

protected:
    SRef<xpcf::IComponentManager> m_xpcfComponentManager;

private:
    bool loaded;
};

template <class T>
SRef<T> SolARModuleManagerOpencv::createComponent(string uuid)
{
    SRef<T> output;
    if (createComponent(uuid, output))
        return output;
    else
        return nullptr;
}

template <class T>
int SolARModuleManagerOpencv::createComponent(string uuid, SRef<T> &compRef)
{
    boost::uuids::string_generator gen;
    int res;

    if (uuid == UUID::TRANSFORM2D) // Transform 2D component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::geom::I2DTransform::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Transform 2D component creation has failed");
        return res;
    }

    else if (uuid == UUID::OVERLAY2D) // 2D overlay component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::display::I2DOverlay::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Overlay 2D component creation has failed");
        return res;
    }

    else if (uuid == UUID::OVERLAYSBS) // SideBySide overlay component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::display::ISideBySideOverlay::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Overlay Side By Side component creation has failed");
        return res;
    }

    else if (uuid == UUID::OVERLAY3D) // 3D overlay component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::display::I3DOverlay::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Overlay 3D component creation has failed");
        return res;
    }

    else if (uuid == UUID::TRANSFORM3D) // 3D overlay component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::geom::I3DTransform::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Transform 3D component creation has failed");
        return res;
    }

    else if (uuid == UUID::CAMERA_CALIBRATION) // Camera Calibration component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::input::devices::ICameraCalibration::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Camera calibration component creation has failed");
        return res;
    }

    else if (uuid == UUID::CAMERA) // camera
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::input::devices::ICamera::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Camera component creation has failed");
        return res;
    }

    else if (uuid == UUID::CONTOURS_EXTRACTOR) // Contour Extractor Component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IContoursExtractor::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Contours Extractor component creation has failed");
        return res;
    }

    else if (uuid == UUID::CONTOURS_FILTER_BINARY_MARKER) // Contour Filter for Binary Marker Component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IContoursFilter::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Contours Filter for Binary Marker component creation has failed");
        return res;
    }

    else if (uuid == UUID::DESCRIPTOR_MATCHER_KNN || uuid == UUID::DESCRIPTOR_MATCHER_RADIUS) // descriptor matcher components
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IDescriptorMatcher::UUID), compRef);
        if (res == -1)
             LOG_ERROR("KNN descriptor matcher component creation has failed");
        return res;
    }

    else if (uuid == UUID::DESCRIPTORS_EXTRACTOR_ORB || uuid == UUID::DESCRIPTORS_EXTRACTOR_SIFT || uuid == UUID::DESCRIPTORS_EXTRACTOR_SURF128 || uuid == UUID::DESCRIPTORS_EXTRACTOR_SURF64) // keypoint descriptors extractors component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IDescriptorsExtractor::UUID), compRef);
        if (res == -1)
             LOG_ERROR("ORB descriptors extractor component creation has failed");
        return res;
    }

    else if (uuid == UUID::DESCRIPTORS_EXTRACTOR_SBPATTERN) // Squared Binary Pattern descriptor extractor Component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IDescriptorsExtractorSBPattern::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Squared binary pattern descriptors extractor component creation has failed");
        return res;
    }

    else if (uuid == UUID::HOMOGRAPHY_ESTIMATION) // homography estimation
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::solver::pose::IHomographyEstimation::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Homography estimation component creation has failed");
        return res;
    }

    else if (uuid == UUID::HOMOGRAPHY_VALIDATION) // homography validation
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::solver::pose::IHomographyValidation::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Homography validation component creation has failed");
        return res;
    }

    else if (uuid == UUID::IMAGE2WORLD_MAPPER) // Image to world mapper component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::geom::IImage2WorldMapper::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Image to world mapper component creation has failed");
        return res;
    }

    else if (uuid == UUID::IMAGE_CONVERTOR) // image convertor component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::image::IImageConvertor::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Image convertor component creation has failed");
        return res;
    }

    else if (uuid == UUID::IMAGE_FILTER) // image filter component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::image::IImageFilter::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Image filter component creation has failed");
        return res;
    }

    else if (uuid == UUID::IMAGE_LOADER) // image loader component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::image::IImageLoader::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Image loader component creation has failed");
        return res;
    }

    else if (uuid == UUID::IMAGE_VIEWER) // image Viewer component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::display::IImageViewer::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Image viewer component creation has failed");
        return res;
    }

    else if (uuid == UUID::KEYPOINT_DETECTOR) // keypoint detector component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IKeypointDetector::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Keypoints detector component creation has failed");
        return res;
    }

    else if (uuid == UUID::KEYPOINTS_REINDEXER) // Keypoints reindexer component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::IKeypointsReIndexer::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Keypoints reindexer component creation has failed");
        return res;
    }

    else if (uuid == UUID::MARKER2D_NATURAL_IMAGE) // natural image marker component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::input::files::IMarker2DNaturalImage::UUID), compRef);
        if (res == -1)
             LOG_ERROR("2D natural image marker component creation has failed");
        return res;
    }

    else if (uuid == UUID::MARKER2D_SQUARED_BINARY) // Squared Binary Marker component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::input::files::IMarker2DSquaredBinary::UUID), compRef);
        if (res == -1)
             LOG_ERROR("2D squared binary marker component creation has failed");
        return res;
    }

    else if (uuid == UUID::PERSPECTIVE_CONTROLLER) // Perspective controller component
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::image::IPerspectiveController::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Perspective controller component creation has failed");
        return res;
    }

    else if (uuid == UUID::POSE_ESTIMATION) // posefinder
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::solver::pose::IPoseEstimation::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Pose estimation component creation has failed");
        return res;
    }

    else if (uuid == UUID::SBPATTERN_REINDEXER) // posefinder
    {
        res=m_xpcfComponentManager->createComponent(gen(uuid), gen(api::features::ISBPatternReIndexer::UUID), compRef);
        if (res == -1)
             LOG_ERROR("Squared binary pattern reindexer component creation has failed");
        return res;
    }

    return -1;
}
}  // End namespace OPENCV
}  // End namespace MODULES
}  // End namespace SolAR

#endif // SOLARMODULEMANAGEROPENCV_H
