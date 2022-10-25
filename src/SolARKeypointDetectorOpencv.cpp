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

SolARKeypointDetectorOpencv::SolARKeypointDetectorOpencv():ConfigurableBase(xpcf::toUUID<SolARKeypointDetectorOpencv>())
{
    declareInterface<api::features::IKeypointDetector>(this);

    declareProperty("imageRatio", m_imageRatio);
    declareProperty("nbDescriptors", m_nbDescriptors);
    declareProperty("threshold", m_threshold);
    declareProperty("nbOctaves", m_nbOctaves);
    declareProperty("nbGridWidth", m_nbGridWidth);
    declareProperty("nbGridHeight", m_nbGridHeight);
    declareProperty("borderRatio", m_borderRatio);
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
         return xpcf::XPCFErrorCode::_SUCCESS;
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
	case (KeypointDetectorType::SIFT):
		LOG_DEBUG("KeypointDetectorImp::setType(SIFT)");
		if (m_threshold > 0)
			m_detector = SIFT::create(0, m_nbOctaves, m_threshold, 10.0);
        else
            m_detector = SIFT::create(4 * m_nbDescriptors);
        break;
	case (KeypointDetectorType::AKAZE):
		LOG_DEBUG("KeypointDetectorImp::setType(AKAZE)");
		if (m_threshold > 0)
            m_detector = AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, m_threshold, m_nbOctaves);
		else
			m_detector = AKAZE::create();
		break;
	case (KeypointDetectorType::AKAZE2):
		LOG_DEBUG("KeypointDetectorImp::setType(AKAZE2)");
		if (m_threshold > 0)
            m_detector = AKAZE2::create(5, 0, 3, m_threshold, m_nbOctaves);
		else
			m_detector = AKAZE2::create();
		break;
	case (KeypointDetectorType::ORB):
        LOG_DEBUG("KeypointDetectorImp::setType(ORB)");
		if (m_nbDescriptors > 0)
            m_detector=ORB::create(100* m_nbDescriptors, 1.2f);
		else
			m_detector = ORB::create();
        break;
    case (KeypointDetectorType::BRISK):
        LOG_DEBUG("KeypointDetectorImp::setType(BRISK)");
		if (m_threshold > 0)
			m_detector = BRISK::create((int)m_threshold, m_nbOctaves);
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
	if (opencvImage.channels() != 1)
		cvtColor(opencvImage, img_1, COLOR_BGR2GRAY);
	else
		img_1 = opencvImage;
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
			// fix scale due to downscaling
			for (auto& keypoint : kpts) {
				keypoint.pt.x = keypoint.pt.x * ratioInv;
				keypoint.pt.y = keypoint.pt.y * ratioInv;
			}
			// group keypoints according to cells of the grid
			std::map<int, std::vector<cv::KeyPoint>> gridKps;
			int borderWidth = static_cast<int>(m_borderRatio * image->getWidth());
			int borderHeight = static_cast<int>(m_borderRatio * image->getHeight());
			int widthToExtract = image->getWidth() - 2 * borderWidth;
			int heightToExtract = image->getHeight() - 2 * borderHeight;
			for (const auto &it : kpts) {
				int id_width = static_cast<int>(std::floor((it.pt.x - borderWidth) * m_nbGridWidth / widthToExtract));
				int id_height = static_cast<int>(std::floor((it.pt.y - borderHeight) * m_nbGridHeight / heightToExtract));
				if ((id_width < 0) || (id_width >= m_nbGridWidth) || (id_height < 0) || (id_height >= m_nbGridHeight))
					continue;
				gridKps[id_height * m_nbGridWidth + id_width].push_back(it);
			}
			// get best feature per cell
			kpts.clear();
			int nbCells = static_cast<int>(gridKps.size());
			if (nbCells > 0) {
				int countCell(0);				
				for (auto& it: gridKps) {
					int nbKpPerCell = static_cast<int>((m_nbDescriptors - kpts.size()) / (nbCells - countCell));
					m_kptsFilter.retainBest(it.second, nbKpPerCell);
					kpts.insert(kpts.end(), it.second.begin(), it.second.end());
					countCell++;
				}
			}
        }
    }
    catch (Exception& e)
    {
        LOG_ERROR("Feature : {}", m_detector->getDefaultName())
        LOG_ERROR("{}",e.msg)
        return;
    }

    int kpID=0;
    for(const auto& keypoint : kpts){
       Keypoint kpa;
	   float px = keypoint.pt.x;
	   float py = keypoint.pt.y;
	   cv::Vec3b bgr{ 0, 0, 0 };
	   if (opencvImage.channels() == 3)
           bgr = image->getPixel<cv::Vec3b>((int)py, (int)px);
       else {
           uint8_t value = image->getPixel<uint8_t>((int)py, (int)px);
           bgr = cv::Vec3b(value, value, value);
       }
       kpa.init(kpID++, px, py, bgr[2], bgr[1], bgr[0], keypoint.size, keypoint.angle, keypoint.response, keypoint.octave, keypoint.class_id) ;
       keypoints.push_back(kpa);	   
    }
}

}
}
}  // end of namespace SolAR
