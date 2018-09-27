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

#include "SolARKeypointDetectorOpencv.h"
#include "SolAROpenCVHelper.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv)

namespace xpcf = org::bcom::xpcf;

using namespace cv;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

static std::map<std::string,KeypointDetectorType> stringToType = {{"AKAZE",KeypointDetectorType::AKAZE},
                                                                  {"AKAZE2",KeypointDetectorType::AKAZE2},
                                                                  {"ORB",KeypointDetectorType::ORB},
                                                                  {"BRISK",KeypointDetectorType::BRISK}};

static std::map<KeypointDetectorType,std::string> typeToString = {{KeypointDetectorType::AKAZE, "AKAZE"},
                                                                  {KeypointDetectorType::AKAZE2,"AKAZE2"},
                                                                  {KeypointDetectorType::ORB,"ORB"},
                                                                  {KeypointDetectorType::BRISK,"BRISK"}};

SolARKeypointDetectorOpencv::SolARKeypointDetectorOpencv():ConfigurableBase(xpcf::toUUID<SolARKeypointDetectorOpencv>())
{
    addInterface<api::features::IKeypointDetector>(this);

    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapFloat("imageRatio", m_imageRatio);
    params->wrapInteger("nbDescriptors", m_nbDescriptors);
	params->wrapFloat("threshold", m_threshold);
    params->wrapString("type", m_type);
    LOG_DEBUG("SolARKeypointDetectorOpencv constructor");
}

SolARKeypointDetectorOpencv::~SolARKeypointDetectorOpencv()
{
    LOG_DEBUG("SolARKeypointDetectorOpencv destructor");
}

xpcf::XPCFErrorCode SolARKeypointDetectorOpencv::onConfigured()
{
    LOG_DEBUG(" SolARKeypointDetectorOpencv onConfigured");
    setType(stringToType.at(m_type));
    return xpcf::_SUCCESS;
}

void SolARKeypointDetectorOpencv::setType(KeypointDetectorType type)
{

    /*
     * 	SURF,
        ORB,
        SIFT,
        DAISY,
        LATCH,
        AKAZE,
        AKAZEUP,
        BRISK,
        BRIEF,
        */
    m_type=typeToString.at(type);
    switch (type) {
	case (KeypointDetectorType::AKAZE):
		LOG_DEBUG("KeypointDetectorImp::setType(AKAZE)");
		if (m_threshold > 0)
			m_detector = AKAZE::create(5, 0, 3, m_threshold);
		else
			m_detector = AKAZE::create();
		break;
	case (KeypointDetectorType::AKAZE2):
		LOG_DEBUG("KeypointDetectorImp::setType(AKAZE2)");
		if (m_threshold > 0)
			m_detector = AKAZE2::create(5, 0, 3, m_threshold);
		else
			m_detector = AKAZE2::create();
		break;
	case (KeypointDetectorType::ORB):
        LOG_DEBUG("KeypointDetectorImp::setType(ORB)");
		if (m_nbDescriptors > 0)
			m_detector=ORB::create(m_nbDescriptors);
		else
			m_detector = ORB::create();
        break;
    case (KeypointDetectorType::BRISK):
        LOG_DEBUG("KeypointDetectorImp::setType(BRISK)");
		if (m_threshold > 0)
			m_detector = BRISK::create((int)m_threshold);
		else
			m_detector=BRISK::create();
        break;


    default :
        LOG_DEBUG("KeypointDetectorImp::setType(AKAZE)");
        m_detector=AKAZE::create();
        break;
    }
}

KeypointDetectorType SolARKeypointDetectorOpencv::getType()
{
    return stringToType.at(m_type);
}

void SolARKeypointDetectorOpencv::detect(const SRef<Image> &image, std::vector<SRef<Keypoint>> &keypoints)
{
    std::vector<cv::KeyPoint> kpts;

    // the input image is down-scaled to accelerate the keypoints extraction

    float ratioInv=1.f/m_imageRatio;

    keypoints.clear();

    // instantiation of an opencv image from an input IImage
    cv::Mat opencvImage = SolAROpenCVHelper::mapToOpenCV(image);

    cv::Mat img_1;
    cvtColor( opencvImage, img_1, CV_BGR2GRAY );
    cv::resize(img_1, img_1, Size(img_1.cols*m_imageRatio,img_1.rows*m_imageRatio), 0, 0);

    try
    {
        if(!m_detector){
            LOG_DEBUG(" detector is initialized with default value : {}", this->m_type)
            setType(stringToType.at(this->m_type));
        }
        m_detector->detect(img_1, kpts, Mat());
    }
    catch (Exception& e)
    {
        LOG_ERROR("Feature : {}", m_detector->getDefaultName())
        LOG_ERROR("{}",e.msg)
        return;
    }

    if (m_nbDescriptors >= 0)
        kptsFilter.retainBest(kpts,m_nbDescriptors);

    for(std::vector<cv::KeyPoint>::iterator itr=kpts.begin();itr!=kpts.end();++itr){
       SRef<Keypoint> kpa = xpcf::utils::make_shared<Keypoint>();

        kpa->init((*itr).pt.x*ratioInv,(*itr).pt.y*ratioInv,(*itr).size,(*itr).angle,(*itr).response,(*itr).octave,(*itr).class_id) ;
        keypoints.push_back(kpa);
    }
}

}
}
}  // end of namespace SolAR
