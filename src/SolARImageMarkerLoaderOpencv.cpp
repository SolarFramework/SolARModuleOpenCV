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

#include <boost/filesystem.hpp>
#include <SolARImageMarkerLoaderOpencv.h>
#include <datastructure/ImageMarker.h>
#include <SolAROpenCVHelper.h>
#include <core/Log.h>
#include <opencv2/opencv.hpp>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageMarkerLoaderOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARImageMarkerLoaderOpencv::SolARImageMarkerLoaderOpencv():ConfigurableBase(xpcf::toUUID<SolARImageMarkerLoaderOpencv>())
    {        
        declareInterface<api::input::files::ITrackableLoader>(this);
        LOG_DEBUG("SolARMarker2DSquaredBinaryOpencv constructor")
        declareProperty("filePath", m_filePath);
    }

    FrameworkReturnCode SolARImageMarkerLoaderOpencv::loadTrackable(SRef<datastructure::Trackable>& trackable)
    {
        std::string imagePath;
        Sizef markerSize;
        SRef<Image> image;

        cv::FileStorage fs(m_filePath, cv::FileStorage::READ);
        fs["MarkerWidth"] >> markerSize.width;
        fs["MarkerHeight"] >> markerSize.height;
        fs["ImagePath"] >> imagePath;
        fs.release();

        if (imagePath.empty())
        {
            LOG_ERROR("Marker file does not define an image path under markup ImagePath")
            return FrameworkReturnCode::_ERROR_;
        }

        // If the path is relative, append it to the directory of the marker file
        boost::filesystem::path imageFullPath(imagePath);
        if (imageFullPath.is_relative())
        {
            imageFullPath = m_filePath;
            boost::filesystem::path parentPath = imageFullPath.parent_path();
            imageFullPath.remove_filename();
            imageFullPath/=imagePath;
        }

        LOG_DEBUG("{}",imageFullPath)

        cv::Mat ocvImage = cv::imread(imageFullPath.string(), cv::IMREAD_COLOR);

        if(! ocvImage.data )  // Check for invalid input
        {
            LOG_ERROR("Error: Could not open or find the 2D natural image marker {}", m_filePath)
            return FrameworkReturnCode::_ERROR_LOAD_IMAGE;
        }
        else
        {
           SolAROpenCVHelper::convertToSolar(ocvImage, image);
        }

        trackable = xpcf::utils::make_shared<ImageMarker>(m_filePath, markerSize, image);

        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
