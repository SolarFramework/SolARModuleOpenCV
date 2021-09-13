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


#include "SolARDescriptorsExtractorFromImageOpencv.h"
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorsExtractorFromImageOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDescriptorsExtractorFromImageOpencv::SolARDescriptorsExtractorFromImageOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorsExtractorFromImageOpencv>())
{
    addInterface<api::features::IDescriptorsExtractorFromImage>(this);
    declareInjectable<api::features::IKeypointDetector>(m_detector);
    declareInjectable<api::features::IDescriptorsExtractor>(m_extractor);
    LOG_DEBUG(" SolARDescriptorsExtractorFromImageOpencv constructor");
}

SolARDescriptorsExtractorFromImageOpencv::~SolARDescriptorsExtractorFromImageOpencv(){
    LOG_DEBUG(" SolARDescriptorsExtractorFromImageOpencv deconstructor");
}

xpcf::XPCFErrorCode SolARDescriptorsExtractorFromImageOpencv::onConfigured()
{
    return xpcf::XPCFErrorCode::_SUCCESS;
}

std::string SolARDescriptorsExtractorFromImageOpencv::getTypeString()
{
    return m_extractor->getTypeString();
}

FrameworkReturnCode SolARDescriptorsExtractorFromImageOpencv::extract(const SRef<datastructure::Image> image,
                                                                      std::vector<datastructure::Keypoint> & keypoints,
                                                                      SRef<SolAR::datastructure::DescriptorBuffer> & descriptors )
{
    m_detector->detect(image, keypoints);
    if (keypoints.size() == 0){
        LOG_WARNING("No keypoints detected");
        return FrameworkReturnCode::_ERROR_;
    }
    m_extractor->extract(image, keypoints, descriptors);
    return FrameworkReturnCode::_SUCCESS;
}


}
}
}
