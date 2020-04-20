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
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv)

namespace xpcf = org::bcom::xpcf;

using namespace cv;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

static std::map<std::string,IKeypointDetector::KeypointDetectorType> stringToType = {{"AKAZE",IKeypointDetector::KeypointDetectorType::AKAZE},
                                                                  {"AKAZE2",IKeypointDetector::KeypointDetectorType::AKAZE2},
                                                                  {"ORB",IKeypointDetector::KeypointDetectorType::ORB},
                                                                  {"BRISK",IKeypointDetector::KeypointDetectorType::BRISK},
                                                                  {"FEATURE_TO_TRACK", IKeypointDetector::KeypointDetectorType::FEATURE_TO_TRACK}};

static std::map<IKeypointDetector::KeypointDetectorType,std::string> typeToString = {{IKeypointDetector::KeypointDetectorType::AKAZE, "AKAZE"},
                                                                  {IKeypointDetector::KeypointDetectorType::AKAZE2,"AKAZE2"},
                                                                  {IKeypointDetector::KeypointDetectorType::ORB,"ORB"},
                                                                  {IKeypointDetector::KeypointDetectorType::BRISK,"BRISK"},
                                                                  {IKeypointDetector::KeypointDetectorType::FEATURE_TO_TRACK,"FEATURE_TO_TRACK"}};

SolARKeypointDetectorOpencv::SolARKeypointDetectorOpencv():ConfigurableBase(xpcf::toUUID<SolARKeypointDetectorOpencv>())
{
    declareInterface<api::features::IKeypointDetector>(this);

    declareProperty("imageRatio", m_imageRatio);
    declareProperty("nbDescriptors", m_nbDescriptors);
    declareProperty("threshold", m_threshold);
    declareProperty("type", m_type);
    LOG_DEBUG("SolARKeypointDetectorOpencv constructor");
}

SolARKeypointDetectorOpencv::~SolARKeypointDetectorOpencv()
{
    LOG_DEBUG("SolARKeypointDetectorOpencv destructor");
}

xpcf::XPCFErrorCode SolARKeypointDetectorOpencv::onConfigured()
{
    LOG_DEBUG(" SolARKeypointDetectorOpencv onConfigured");
    if (stringToType.find(m_type) != stringToType.end())
    {
        setType(stringToType.at(m_type));
         return xpcf::_SUCCESS;
    }
    else
    {
        LOG_WARNING("Keypoint detector of type {} defined in your configuration file does not exist", m_type);
        return xpcf::_ERROR_NOT_IMPLEMENTED;
    }
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
        FEATURE_TO_TRACK
        */
    m_type=typeToString.at(type);
    switch (type) {
	case (KeypointDetectorType::AKAZE):
		LOG_DEBUG("KeypointDetectorImp::setType(AKAZE)");
		if (m_threshold > 0)
            m_detector = AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, m_threshold);
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

IKeypointDetector::KeypointDetectorType SolARKeypointDetectorOpencv::getType()
{
    return stringToType.at(m_type);
}

void SolARKeypointDetectorOpencv::detect(const SRef<Image> image, std::vector<Keypoint> & keypoints)
{
    std::vector<cv::KeyPoint> kpts;

    // the input image is down-scaled to accelerate the keypoints extraction

    float ratioInv=1.f/m_imageRatio;

    keypoints.clear();

    // instantiation of an opencv image from an input IImage
    cv::Mat opencvImage = SolAROpenCVHelper::mapToOpenCV(image);

    cv::Mat img_1;
    cvtColor( opencvImage, img_1, COLOR_BGR2GRAY );
    cv::resize(img_1, img_1, Size(img_1.cols*m_imageRatio,img_1.rows*m_imageRatio), 0, 0);

    try
    {
        if (m_type == "FEATURE_TO_TRACK") {
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(img_1, corners, m_nbDescriptors, 0.008, 3, cv::Mat(), 3);
            cornerSubPix(img_1, corners, cv::Size(7, 7), Size(-1, -1), cv::TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03));
            for (auto it : corners)
                kpts.push_back(cv::KeyPoint(it, 0.f));
        }
        else {
            if(!m_detector){
                LOG_DEBUG(" detector is initialized with default value : {}", this->m_type)
                setType(stringToType.at(this->m_type));
            }
            m_detector->detect(img_1, kpts, Mat());
            if (m_nbDescriptors >= 0)
                kptsFilter.retainBest(kpts,m_nbDescriptors);
        }
    }
    catch (Exception& e)
    {
        LOG_ERROR("Feature : {}", m_detector->getDefaultName())
        LOG_ERROR("{}",e.msg)
        return;
    }

    for(auto keypoint : kpts){
       Keypoint kpa;
       kpa.init(keypoint.pt.x*ratioInv, keypoint.pt.y*ratioInv, keypoint.size, keypoint.angle, keypoint.response, keypoint.octave, keypoint.class_id) ;
       keypoints.push_back(kpa);
    }
}

}
}
}  // end of namespace SolAR
