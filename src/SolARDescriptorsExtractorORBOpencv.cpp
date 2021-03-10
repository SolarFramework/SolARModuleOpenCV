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

#include "SolARDescriptorsExtractorORBOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorORBOpencv)

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

SolARDescriptorsExtractorORBOpencv::SolARDescriptorsExtractorORBOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorsExtractorORBOpencv>())
{
    declareInterface<api::features::IDescriptorsExtractor>(this);

    declareProperty("nbFeatures", m_nbFeatures);
    declareProperty("scaleFactor", m_scaleFactor);
    declareProperty("nbLevels", m_nbLevels);
    declareProperty("edgeThreshold", m_edgeThreshold);
    declareProperty("firstLevel", m_firstLevel);
    declareProperty("WTAK", m_WTAK);
    declareProperty("scoreType", m_scoreType);
    declareProperty("patchSize", m_patchSize);
    declareProperty("fastThreshold", m_fastThreshold);

    LOG_DEBUG(" SolARDescriptorsExtractorORBOpencv constructor")
}

xpcf::XPCFErrorCode SolARDescriptorsExtractorORBOpencv::onConfigured()
{
    cv::ORB::ScoreType scoreType;
    if (m_scoreType == "Harris")
        scoreType = cv::ORB::ScoreType::HARRIS_SCORE;
    else if (m_scoreType == "Fast")
        scoreType = cv::ORB::ScoreType::FAST_SCORE;
    else
    {
        scoreType = cv::ORB::ScoreType::HARRIS_SCORE;
        LOG_WARNING("Score Type \"{}\" is not allowed for ORB descriptor. It should be whether \"Harris\" or \"Fast\". Set to Harris by default", m_scoreType);
    }

    m_extractor=cv::ORB::create(m_nbFeatures, m_scaleFactor, m_nbLevels, m_edgeThreshold, m_firstLevel, m_WTAK, scoreType, m_patchSize, m_fastThreshold);
    return xpcf::XPCFErrorCode::_SUCCESS;
}

SolARDescriptorsExtractorORBOpencv::~SolARDescriptorsExtractorORBOpencv()
{
    LOG_DEBUG(" SolARDescriptorsExtractorORBOpencv destructor")
}

void SolARDescriptorsExtractorORBOpencv::extract(const SRef<Image> image, const std::vector<Keypoint > &keypoints, SRef<DescriptorBuffer>& descriptors)
{
    //transform all SolAR data to openCv data

    SRef<Image> convertedImage = image;

    if (image->getImageLayout() != Image::ImageLayout::LAYOUT_GREY) {
        // input Image not in grey levels : convert it !
        m_convertor.convert(image, convertedImage, Image::ImageLayout::LAYOUT_GREY);
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

    descriptors.reset( new DescriptorBuffer(out_mat_descps.data, DescriptorType::ORB, DescriptorDataType::TYPE_8U, 32, out_mat_descps.rows)) ;
    
}

}
}
}  // end of namespace SolAR
