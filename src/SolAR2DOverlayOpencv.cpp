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

#include "SolAR2DOverlayOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"



#include <map>
#include <random>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolAR2DOverlayOpencv::SolAR2DOverlayOpencv():ConfigurableBase(xpcf::toUUID<SolAR2DOverlayOpencv>())
{
   addInterface<api::display::I2DOverlay>(this);
   SRef<xpcf::IPropertyMap> params = getPropertyRootNode();

   params->wrapUnsignedInteger("thickness", m_thickness);
   params->wrapUnsignedInteger("radius", m_radius);
   params->wrapUnsignedIntegerVector("color", m_color);
   params->wrapUnsignedInteger("randomColor", m_randomColor);
   LOG_DEBUG(" SolAR2DOverlayOpencv constructor");

}


void SolAR2DOverlayOpencv::drawCircle(const SRef<Point2Df> point, SRef<Image> displayImage)
{
    // check that center of cirlcle is inside the image
    if (point->getX()<0 || point->getY() < 0 || point->getX() > displayImage->getWidth() || point->getY() > displayImage->getHeight())
        return;

    // image where circle will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);
    if (!m_randomColor)
        cv::circle(displayedImage,cv::Point2f(point->getX(), point->getY()) ,m_radius,cv::Scalar(m_color[0],m_color[1], m_color[2]),m_thickness);
    else
        cv::circle(displayedImage,cv::Point2f(point->getX(), point->getY()) ,m_radius,cv::Scalar(128,128,128),m_thickness);

}

void SolAR2DOverlayOpencv::drawCircles(const std::vector<SRef<Point2Df>>& points, SRef<Image> displayImage)
{
    // image where circle will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);

    if (!m_randomColor)
    {
        for (std::vector<SRef<Point2Df>>::const_iterator itr = points.begin(); itr != points.end(); itr++)
        {
            // check that center of cirlcle is inside the image
            if ((*itr)->x()>= 0 && (*itr)->y() >= 0 && (*itr)->x() <= displayImage->getWidth() && (*itr)->y() < displayImage->getHeight())
                cv::circle(displayedImage,cv::Point2f((*itr)->x(), (*itr)->y()) ,m_radius,cv::Scalar(m_color[0],m_color[1], m_color[2]),m_thickness);
        }
    }
    else
    {
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased

        for (std::vector<SRef<Point2Df>>::const_iterator itr = points.begin(); itr != points.end(); itr++)
        {
            // check that center of cirlcle is inside the image
            if ((*itr)->x()>= 0 && (*itr)->y() >= 0 && (*itr)->x() <= displayImage->getWidth() && (*itr)->y() < displayImage->getHeight())
                cv::circle(displayedImage,cv::Point2f((*itr)->x(), (*itr)->y()) ,m_radius,cv::Scalar(uni(rng), uni(rng), uni(rng)),m_thickness);
        }
    }
}

void SolAR2DOverlayOpencv::drawCircles(const std::vector<SRef<Keypoint>>& keypoints, SRef<Image> displayImage)
{
    // image where circle will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);

    if (!m_randomColor)
    {
        for (std::vector<SRef<Keypoint>>::const_iterator itr = keypoints.begin(); itr != keypoints.end(); itr++)
        {
            // check that center of cirlcle is inside the image
            if ((*itr)->x()>= 0 && (*itr)->y() >= 0 && (*itr)->x() <= displayImage->getWidth() && (*itr)->y() < displayImage->getHeight())
                cv::circle(displayedImage, cv::Point2f((*itr)->x(), (*itr)->y()), m_radius, cv::Scalar(m_color[0],m_color[1], m_color[2]), m_thickness);
        }
    }
    else // use random colors
    {
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased

        for (std::vector<SRef<Keypoint>>::const_iterator itr = keypoints.begin(); itr != keypoints.end(); itr++)
        {
            // check that center of cirlcle is inside the image
            if ((*itr)->x()>= 0 && (*itr)->y() >= 0 && (*itr)->x() <= displayImage->getWidth() && (*itr)->y() < displayImage->getHeight())
                cv::circle(displayedImage, cv::Point2f((*itr)->x(), (*itr)->y()), m_radius, cv::Scalar(uni(rng), uni(rng), uni(rng)), m_thickness);
        }
    }

}

void SolAR2DOverlayOpencv::drawContours (const std::vector <SRef<Contour2Df>> & contours, SRef<Image> displayImage)
{
    // image where contours will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);

    std::vector<unsigned int> color = m_color;
    if (m_randomColor)
        color = {128, 128, 128};

    for (std::vector<SRef<Contour2Df>>::const_iterator itr = contours.begin(); itr != contours.end(); itr++)
    {
        const SRef<const Contour2Df> contour = *itr;
        for (int i = 0; i < contour->size(); i++)
        {
            if (i != contour->size() - 1)
            {
                cv::Point2f pt_a((*contour)[i][0], (*contour)[i][1]);
                cv::Point2f pt_b((*contour)[i+1][0],(*contour)[i+1][1]);

                SolAROpenCVHelper::drawCVLine(displayedImage, pt_a, pt_b, cv::Scalar(color[0],color[1], color[2]), m_thickness);
            }
            else
            {
                //the contours loops to the first point
                cv::Point2f pt_a((*contour)[i][0], (*contour)[i][1]);
                cv::Point2f pt_b((*contour)[0][0],(*contour)[0][1]);

                SolAROpenCVHelper::drawCVLine(displayedImage, pt_a, pt_b, cv::Scalar(color[0],color[1], color[2]), m_thickness);
            }
        }
    }
}

void SolAR2DOverlayOpencv::drawSBPattern (const SRef<SquaredBinaryPattern> pattern, SRef<Image> displayImage)
{
    // image where contours will be displayed
    cv::Mat displayImageCV = SolAROpenCVHelper::mapToOpenCV(displayImage);
    displayImageCV.setTo(0);

    int patternSize = pattern->getSize();
    SquaredBinaryPatternMatrix matrix = pattern->getPatternMatrix();

    float cellSizeX = (float)displayImage->getWidth()/(patternSize+2);
    float cellSizeY = (float)displayImage->getHeight()/(patternSize+2);

    for (int i = 0; i < patternSize+2; i++)
    {
        for (int j = 0; j < patternSize+2; j++)
        {
            if (i>0 && j>0 && i <patternSize+1 && j < patternSize+1 && (bool)matrix(j-1,i-1))
            {
                cv::Rect roi = cv::Rect(cv::Point(i*cellSizeX, j*cellSizeY), cv::Point((i+1) * cellSizeX, (j+1) * cellSizeY));
                displayImageCV(roi).setTo(255);
            }
        }
    }
}
}
}
}
