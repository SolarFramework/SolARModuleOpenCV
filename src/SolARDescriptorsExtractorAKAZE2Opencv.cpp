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

#include "SolARDescriptorsExtractorAKAZE2Opencv.h"
#include "SolARImageConvertorOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

//#include <boost/thread/thread.hpp>
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv)

namespace xpcf = org::bcom::xpcf;
using namespace cv;

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {


SolARDescriptorsExtractorAKAZE2Opencv::SolARDescriptorsExtractorAKAZE2Opencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorsExtractorAKAZE2Opencv>())
{
    declareInterface<api::features::IDescriptorsExtractor>(this);
    LOG_DEBUG(" SolARDescriptorsExtractorAKAZE2Opencv constructor")
    // m_extractor must have a default implementation : initialize default extractor type
    m_extractor=AKAZE2::create();
    declareProperty("threshold", m_threshold);
}

SolARDescriptorsExtractorAKAZE2Opencv::~SolARDescriptorsExtractorAKAZE2Opencv()
{
    LOG_DEBUG(" SolARDescriptorsExtractorAKAZE2Opencv destructor")
}

xpcf::XPCFErrorCode SolARDescriptorsExtractorAKAZE2Opencv::onConfigured()
{
    LOG_DEBUG(" SolARDescriptorsExtractorAKAZE2Opencv onConfigured");
    if (m_extractor.empty())
        return xpcf::XPCFErrorCode::_FAIL;
    m_extractor->setThreshold(m_threshold);
    return xpcf::XPCFErrorCode::_SUCCESS;
}

void SolARDescriptorsExtractorAKAZE2Opencv::extract(const SRef<Image> image, const std::vector<Keypoint> & keypoints, SRef<DescriptorBuffer> & descriptors)
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

    for(unsigned int k =0; k < keypoints.size(); ++k)
    {
        transform_to_data.push_back(
                    //instantiate keypoint
                     cv::KeyPoint(keypoints[k].getX(),
                                  keypoints[k].getY(),
                                  keypoints[k].getSize(),
                                  keypoints[k].getAngle(),
                                  keypoints[k].getResponse(),
                                  keypoints[k].getOctave(),
                                  keypoints[k].getClassId())
                    );
    }

   m_extractor->compute(opencvImage, transform_to_data, out_mat_descps);

   descriptors.reset( new DescriptorBuffer(out_mat_descps.data, DescriptorType::AKAZE, DescriptorDataType::TYPE_8U, 61, out_mat_descps.rows)) ;

}

}
}
}  // end of namespace SolAR
