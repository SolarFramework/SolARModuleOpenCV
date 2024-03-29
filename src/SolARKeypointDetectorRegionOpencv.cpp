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

#include "SolARKeypointDetectorRegionOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARKeypointDetectorRegionOpencv)

namespace xpcf = org::bcom::xpcf;

using namespace cv;
#if ((CV_VERSION_MAJOR < 4 ) || (CV_VERSION_MINOR < 4 ))
    using namespace cv::xfeatures2d;
#endif

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

static std::map<std::string,IKeypointDetector::KeypointDetectorType> stringToType = {{"SIFT",IKeypointDetector::KeypointDetectorType::SIFT},
                                                                  {"AKAZE",IKeypointDetector::KeypointDetectorType::AKAZE},
                                                                  {"AKAZE2",IKeypointDetector::KeypointDetectorType::AKAZE2},
                                                                  {"ORB",IKeypointDetector::KeypointDetectorType::ORB},
                                                                  {"BRISK",IKeypointDetector::KeypointDetectorType::BRISK},
                                                                  {"FEATURE_TO_TRACK", IKeypointDetector::KeypointDetectorType::FEATURE_TO_TRACK}};

static std::map<IKeypointDetector::KeypointDetectorType,std::string> typeToString = {{IKeypointDetector::KeypointDetectorType::SIFT, "SIFT"},
                                                                  {IKeypointDetector::KeypointDetectorType::AKAZE, "AKAZE"},
                                                                  {IKeypointDetector::KeypointDetectorType::AKAZE2,"AKAZE2"},
                                                                  {IKeypointDetector::KeypointDetectorType::ORB,"ORB"},
                                                                  {IKeypointDetector::KeypointDetectorType::BRISK,"BRISK"},
                                                                  {IKeypointDetector::KeypointDetectorType::FEATURE_TO_TRACK,"FEATURE_TO_TRACK"}};

SolARKeypointDetectorRegionOpencv::SolARKeypointDetectorRegionOpencv():ConfigurableBase(xpcf::toUUID<SolARKeypointDetectorRegionOpencv>())
{
    declareInterface<api::features::IKeypointDetectorRegion>(this);

    declareProperty("imageRatio", m_imageRatio);
    declareProperty("nbDescriptors", m_nbDescriptors);
    declareProperty("threshold", m_threshold);
    declareProperty("type", m_type);
    LOG_DEBUG("SolARKeypointDetectorRegionOpencv constructor");
}

SolARKeypointDetectorRegionOpencv::~SolARKeypointDetectorRegionOpencv()
{
    LOG_DEBUG("SolARKeypointDetectorRegionOpencv destructor");
}

xpcf::XPCFErrorCode SolARKeypointDetectorRegionOpencv::onConfigured()
{
    LOG_DEBUG(" SolARKeypointDetectorRegionOpencv onConfigured");
    if (stringToType.find(m_type) != stringToType.end())
    {
        setType(stringToType.at(m_type));
        return xpcf::XPCFErrorCode::_SUCCESS;
    }
    else
    {
        LOG_WARNING("Keypoint detector of type {} defined in your configuration file does not exist", m_type);
        return xpcf::_ERROR_NOT_IMPLEMENTED;
    }
}

void SolARKeypointDetectorRegionOpencv::setType(IKeypointDetector::KeypointDetectorType type)
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
    case (IKeypointDetector::KeypointDetectorType::SIFT):
       LOG_DEBUG("KeypointDetectorImp::setType(SIFT)");
       if (m_threshold > 0)
            m_detector = SIFT::create(m_nbDescriptors, 3, 0.04, m_threshold);
       else
            m_detector = SIFT::create(m_nbDescriptors);
       break;
    case (IKeypointDetector::KeypointDetectorType::AKAZE):
		LOG_DEBUG("KeypointDetectorImp::setType(AKAZE)");
		if (m_threshold > 0)
            m_detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3, m_threshold);
		else
			m_detector = AKAZE::create();
		break;
    case (IKeypointDetector::KeypointDetectorType::AKAZE2):
		LOG_DEBUG("KeypointDetectorImp::setType(AKAZE2)");
		if (m_threshold > 0)
			m_detector = AKAZE2::create(5, 0, 3, m_threshold);
		else
			m_detector = AKAZE2::create();
		break;
    case (IKeypointDetector::KeypointDetectorType::ORB):
        LOG_DEBUG("KeypointDetectorImp::setType(ORB)");
		if (m_nbDescriptors > 0)
			m_detector=ORB::create(m_nbDescriptors);
		else
			m_detector = ORB::create();
        break;
    case (IKeypointDetector::KeypointDetectorType::BRISK):
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

IKeypointDetector::KeypointDetectorType SolARKeypointDetectorRegionOpencv::getType()
{
    return stringToType.at(m_type);
}

void goodFeaturesToTrackDetection(cv::Mat &img, int &nbDescriptors, std::vector<cv::KeyPoint> &kpts) {
	std::vector<cv::Point2f> corners;
	cv::goodFeaturesToTrack(img, corners, nbDescriptors, 0.008, 3, cv::Mat(), 3);
	cornerSubPix(img, corners, cv::Size(7, 7), Size(-1, -1), cv::TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03));
	for (auto it : corners) {
		kpts.push_back(cv::KeyPoint(it, 0.f));
	}
}

void SolARKeypointDetectorRegionOpencv::detect(const SRef<Image> image, const std::vector<Point2Df> & contours, std::vector<Keypoint> & keypoints)
{
    std::vector<cv::KeyPoint> kpts;

    // the input image is down-scaled to accelerate the keypoints extraction

    float ratioInv=1.f/m_imageRatio;

    keypoints.clear();

    // instantiation of an opencv image from an input IImage
    cv::Mat opencvImage = SolAROpenCVHelper::mapToOpenCV(image);

	cv::Mat img_1;
	if (opencvImage.channels() != 1)
		cvtColor(opencvImage, img_1, COLOR_BGR2GRAY);
	else
		img_1 = opencvImage;
    cv::resize(img_1, img_1, Size(img_1.cols*m_imageRatio,img_1.rows*m_imageRatio), 0, 0);
	
    try
    {
		if (m_type == "FEATURE_TO_TRACK") {
			goodFeaturesToTrackDetection(img_1, m_nbDescriptors, kpts);
		}
		else {
			if (!m_detector) {
				LOG_DEBUG(" detector is initialized with default value : {}", this->m_type)
					setType(stringToType.at(this->m_type));
			}
			m_detector->detect(img_1, kpts, Mat());
			if (m_nbDescriptors >= 0)
				kptsFilter.retainBest(kpts, m_nbDescriptors);
		}
    }
    catch (Exception& e)
    {
        LOG_ERROR("Feature : {}", m_detector->getDefaultName())
        LOG_ERROR("{}",e.msg)
        return;
    }    

	auto getAngle = [](cv::Point2f &A, cv::Point2f &B, cv::Point2f &C) {
		float c = cv::norm(A - B);
		float a = cv::norm(A - C);
		float b = cv::norm(B - C);
		float tmp = (a*a + b * b - c * c) / (2.f * a * b);
		return acosf(tmp);
	};

    auto checkInside = [getAngle](const std::vector<Point2Df>& contours, Point2f &ptToCheck) {
		float sumAngles(0.f);
        for (unsigned int i = 0; i < contours.size(); ++i) {
            cv::Point2f pt1(contours[i].getX(), contours[i].getY());
            cv::Point2f pt2(contours[(i + 1) % contours.size()].getX(), contours[(i + 1) % contours.size()].getY());
			sumAngles += getAngle(pt1, pt2, ptToCheck);
		}
		
		if (std::fabs(sumAngles - 2 * CV_PI) < 1e-4)
			return true;
		else
			return false;
	};

    int kpID = 0;

    for(const auto &keypoint: kpts){
        Point2f ptToCheck = keypoint.pt * ratioInv;
        if (checkInside(contours, ptToCheck)) {
            Keypoint kpa;
			float px = keypoint.pt.x*ratioInv;
			float py = keypoint.pt.y*ratioInv;
			cv::Vec3b bgr{ 0, 0, 0 };
			if (opencvImage.channels() == 3)
				bgr = opencvImage.at<cv::Vec3b>((int)py, (int)px);
			kpa.init(kpID++, px, py, bgr[2], bgr[1], bgr[0], keypoint.size, keypoint.angle, keypoint.response, keypoint.octave, keypoint.class_id);
			keypoints.push_back(kpa);
		}
    }
}

}
}
}  // end of namespace SolAR
