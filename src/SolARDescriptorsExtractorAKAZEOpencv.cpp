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

#include "SolARDescriptorsExtractorAKAZEOpencv.h"
#include "SolARImageConvertorOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv)

namespace xpcf = org::bcom::xpcf;
using namespace cv;

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

SolARDescriptorsExtractorAKAZEOpencv::SolARDescriptorsExtractorAKAZEOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorsExtractorAKAZEOpencv>())
{
    declareInterface<api::features::IDescriptorsExtractor>(this);
    LOG_DEBUG(" SolARDescriptorsExtractorAKAZEOpencv constructor")
    // m_extractor must have a default implementation : initialize default extractor type
    m_extractor=AKAZE::create();
    declareProperty("threshold", m_threshold);
}


SolARDescriptorsExtractorAKAZEOpencv::~SolARDescriptorsExtractorAKAZEOpencv()
{
    LOG_DEBUG(" SolARDescriptorsExtractorAKAZEOpencv destructor")
}

xpcf::XPCFErrorCode SolARDescriptorsExtractorAKAZEOpencv::onConfigured()
{
    LOG_DEBUG(" SolARDescriptorsExtractorAKAZEOpencv onConfigured");
    if (m_extractor->empty())
        return xpcf::_FAIL;
    m_extractor->setThreshold(m_threshold);
    return xpcf::_SUCCESS;
}

void SolARDescriptorsExtractorAKAZEOpencv::extract(const SRef<Image> image, const std::vector<Keypoint> & keypoints, SRef<DescriptorBuffer> & descriptors)
{
    //transform all SolAR data to openCv data

    SRef<Image> convertedImage = image;

    if (image->getImageLayout() != Image::ImageLayout::LAYOUT_GREY) {
        // input Image not in grey levels : convert it !
        SolARImageConvertorOpencv convertor;
        convertedImage = xpcf::utils::make_shared<Image>(Image::ImageLayout::LAYOUT_GREY,Image::PixelOrder::INTERLEAVED,image->getDataType());
        convertor.convert(image,convertedImage);
    }

    cv::Mat opencvImage;
    SolAROpenCVHelper::mapToOpenCV(convertedImage,opencvImage);

    cv::Mat out_mat_descps;

    std::vector<cv::KeyPoint> transform_to_data;
    transform_to_data.reserve(keypoints.size());

    for(const auto & keypoint : keypoints)
    {
        transform_to_data.emplace_back(
                        //instantiate keypoint
                        keypoint.x(),
                        keypoint.y(),
                        keypoint.getSize(),
                        keypoint.getAngle(),
                        keypoint.getResponse(),
                        keypoint.getOctave(),
                        keypoint.getClassId()
                    );
    }

   m_extractor->compute(opencvImage, transform_to_data, out_mat_descps);

   descriptors.reset( new DescriptorBuffer(out_mat_descps.data,DescriptorBuffer::AKAZE, DescriptorBuffer::TYPE_8U, 61, out_mat_descps.rows)) ;

}

}
}
}  // end of namespace SolAR
