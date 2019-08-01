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

#include "SolAROpticalFlowPyrLKOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolAROpticalFlowPyrLKOpencv)

namespace SolAR {
using namespace datastructure;
using namespace api::tracking;
namespace MODULES {
namespace OPENCV {

SolAROpticalFlowPyrLKOpencv::SolAROpticalFlowPyrLKOpencv():ConfigurableBase(xpcf::toUUID<SolAROpticalFlowPyrLKOpencv>())
{
    declareInterface<IOpticalFlowEstimator>(this);
    declareProperty("searchWinWidth", m_searchWinWidth);
    declareProperty("searchWinHeight", m_searchWinHeight);
    declareProperty("maxLevel", m_maxLevel);
    declareProperty("minEigenThreshold", m_minEigenThreshold);
    declareProperty("maxSearchIterations", m_maxSearchIterations);
    declareProperty("searchWindowAccuracy",m_searchWindowAccuracy);

    LOG_DEBUG(" SolAROpticalFlowPyrLKOpencv constructor")
}

SolAROpticalFlowPyrLKOpencv::~SolAROpticalFlowPyrLKOpencv()
{
    LOG_DEBUG(" SolAROpticalFlowPyrLKOpencv destructor")
}

template <class T> std::vector<cv::Point2f> fillcvpoints(const std::vector<T> & points)
{

}

template <>
inline std::vector<cv::Point2f> fillcvpoints(const std::vector<Point2Df> & points)
{

    std::vector<cv::Point2f> cv_points;
    cv_points.reserve(points.size());
for (const auto& point : points)
        cv_points.push_back(cv::Point2f(point.getX(), point.getY()));
    return cv_points;
}

template <>
inline std::vector<cv::Point2f> fillcvpoints(const std::vector<Keypoint> & points)
{
    std::vector<cv::Point2f> cv_points;
    cv_points.reserve(points.size());
for (const auto& point : points)
        cv_points.push_back(cv::Point2f(point.getX(), point.getY()));
    return cv_points;
}

FrameworkReturnCode SolAROpticalFlowPyrLKOpencv::estimate(
                const SRef<Image> previousImage,
                const SRef<Image> currentImage,
                const std::vector<Keypoint> & pointsToTrack,
                std::vector<Point2Df> & trackedPoints,
                std::vector<unsigned char> & status,
                std::vector<float> & error)
{
    std::vector<cv::Point2f> cv_points = fillcvpoints (pointsToTrack);
    return estimate (previousImage, currentImage, cv_points, trackedPoints, status, error);
}

FrameworkReturnCode SolAROpticalFlowPyrLKOpencv::estimate(
                const SRef<Image> previousImage,
                const SRef<Image> currentImage,
                const std::vector<Point2Df> & pointsToTrack,
                std::vector<Point2Df> & trackedPoints,
                std::vector<unsigned char> & status,
                std::vector<float> & error)
{
    std::vector<cv::Point2f> cv_points = fillcvpoints (pointsToTrack);
    return estimate (previousImage, currentImage, cv_points, trackedPoints, status, error);
}

FrameworkReturnCode SolAROpticalFlowPyrLKOpencv::estimate(const SRef<Image>& previousImage,
                                                          const SRef<Image>& currentImage,
                                                          const std::vector<cv::Point2f> & pointsToTrack,
                                                          std::vector<Point2Df> & trackedPoints,
                                                          std::vector<unsigned char> & status,
                                                          std::vector<float> & error)
{
    cv::Mat previousFrame, currentFrame;
    if (previousImage->getImageLayout() == Image::ImageLayout::LAYOUT_GREY)
        previousFrame = SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(previousImage);
    else {
        cv::Mat tempPreviousFrame = SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(previousImage);
         cv::cvtColor(tempPreviousFrame, previousFrame, cv::COLOR_BGR2GRAY);
    }

    if (currentImage->getImageLayout() == Image::ImageLayout::LAYOUT_GREY)
        currentFrame = SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(currentImage);
    else {
        cv::Mat tempCurrentFrame = SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(currentImage);
         cv::cvtColor(tempCurrentFrame, currentFrame, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::Point2f> cv_trackedPoints;

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, m_maxSearchIterations, m_searchWindowAccuracy);
    int flags = m_minEigenThreshold <=0 ? 0 : cv::OPTFLOW_LK_GET_MIN_EIGENVALS;
    cv::calcOpticalFlowPyrLK(previousFrame, currentFrame, pointsToTrack, cv_trackedPoints, status, error, cv::Size(m_searchWinWidth, m_searchWinHeight), m_maxLevel, termcrit, flags, m_minEigenThreshold);

    trackedPoints.clear();
    for (auto & cv_trackedPoint : cv_trackedPoints)
        trackedPoints.push_back(Point2Df(cv_trackedPoint.x, cv_trackedPoint.y));

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
