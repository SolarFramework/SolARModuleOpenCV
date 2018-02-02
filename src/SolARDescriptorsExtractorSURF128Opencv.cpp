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

#include "SolARDescriptorsExtractorSURF128Opencv.h"
#include "SolARImageConvertorOpencv.h"
#include "SolAROpenCVHelper.h"

/*
#include <utility>
#include <iostream>
#include <thread>
#include <array>
*/
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorSURF128Opencv);

namespace xpcf = org::bcom::xpcf;

using namespace cv;
using namespace cv::xfeatures2d;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDescriptorsExtractorSURF128Opencv::SolARDescriptorsExtractorSURF128Opencv()
{
    setUUID(SolARDescriptorsExtractorSURF128Opencv::UUID);
    addInterface<api::features::IDescriptorsExtractor>(this,api::features::IDescriptorsExtractor::UUID, "interface api::features::DescriptorsExtractor");
    LOG_DEBUG(" SolARDescriptorsExtractorSURF128Opencv constructor")
    // m_extractor must have a default implementation : initialize default extractor type
    m_extractor=SURF::create(100,4,3,true);
}


SolARDescriptorsExtractorSURF128Opencv::~SolARDescriptorsExtractorSURF128Opencv()
{
    LOG_DEBUG(" SolARDescriptorExtractorSURF128Opencv destructor")
}

void SolARDescriptorsExtractorSURF128Opencv::extract(const SRef<Image> image, const std::vector<SRef<Keypoint> > &keypoints, SRef<DescriptorBuffer>& descriptors){


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
                     cv::KeyPoint(keypoints[k]->getX(),
                                  keypoints[k]->getY(),
                                  keypoints[k]->getSize(),
                                  keypoints[k]->getAngle(),
                                  keypoints[k]->getResponse(),
                                  keypoints[k]->getOctave(),
                                  keypoints[k]->getClassId())
                    );
    }

   m_extractor->compute(opencvImage, transform_to_data, out_mat_descps);

  // m_ex
  // enum DESCRIPTOR::TYPE desc_type = descriptors->getDescriptorType();

    descriptors.reset( new DescriptorBuffer(out_mat_descps.data,DescriptorBuffer::SURF_128, DescriptorBuffer::TYPE_32F, 128, out_mat_descps.rows)) ;

}

}
}
}  // end of namespace SolAR
