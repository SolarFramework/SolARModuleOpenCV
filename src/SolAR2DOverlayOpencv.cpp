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
#include "core/Log.h"
#include "opencv2/core.hpp"
#include "opencv2/video/video.hpp"
#include <random>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolAR2DOverlayOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolAR2DOverlayOpencv::SolAR2DOverlayOpencv():ConfigurableBase(xpcf::toUUID<SolAR2DOverlayOpencv>())
{
   declareInterface<api::display::I2DOverlay>(this);
   declareProperty("thickness", m_thickness);
   declareProperty("radius", m_radius);
   declarePropertySequence("color", m_color);
   declareProperty("randomColor", m_randomColor);
   declareProperty("font", m_font);

   LOG_DEBUG(" SolAR2DOverlayOpencv constructor");

}


void SolAR2DOverlayOpencv::drawCircle(const Point2Df & point, SRef<Image> displayImage)
{
    // Check that center of circle is inside the image
    if (point.getX()<0 || point.getY() < 0 || point.getX() > displayImage->getWidth() || point.getY() > displayImage->getHeight())
        return;

    // Image where circle will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);
    if (!m_randomColor)
        cv::circle(displayedImage,cv::Point2f(point.getX(), point.getY()) ,m_radius,cv::Scalar(m_color[0],m_color[1], m_color[2]),m_thickness);
    else
        cv::circle(displayedImage,cv::Point2f(point.getX(), point.getY()) ,m_radius,cv::Scalar(128,128,128),m_thickness);
}

void SolAR2DOverlayOpencv::drawCircles(const std::vector<Point2Df>& points, SRef<Image> displayImage)
{
    // image where circle will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);

    if (!m_randomColor)
    {
        for (auto point : points)
        {
            // check that center of circle is inside the image
            if (point.x()>= 0 && point.y() >= 0 && point.x() <= displayImage->getWidth() && point.y() < displayImage->getHeight())
                cv::circle(displayedImage,cv::Point2f(point.x(), point.y()) ,m_radius,cv::Scalar(m_color[0],m_color[1], m_color[2]),m_thickness);
        }
    }
    else
    {
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased

        for (auto point : points)
        {
            // check that center of circle is inside the image
            if (point.x()>= 0 && point.y() >= 0 && point.x() <= displayImage->getWidth() && point.y() < displayImage->getHeight())
                cv::circle(displayedImage,cv::Point2f(point.x(), point.y()) ,m_radius,cv::Scalar(uni(rng), uni(rng), uni(rng)),m_thickness);
        }
    }
}

void SolAR2DOverlayOpencv::drawCircles(const std::vector<Keypoint>& keypoints, SRef<Image> displayImage)
{
    // image where circle will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);

    if (!m_randomColor)
    {
        for (auto keypoint : keypoints)
        {
            // check that center of cirlcle is inside the image
            if (keypoint.x()>= 0 && keypoint.y() >= 0 && keypoint.x() <= displayImage->getWidth() && keypoint.y() < displayImage->getHeight())
                cv::circle(displayedImage, cv::Point2f(keypoint.x(), keypoint.y()), m_radius, cv::Scalar(m_color[0],m_color[1], m_color[2]), m_thickness);
        }
    }
    else // use random colors
    {
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased

        for (auto keypoint : keypoints)
        {
            // check that center of circle is inside the image
            if (keypoint.x()>= 0 && keypoint.y() >= 0 && keypoint.x() <= displayImage->getWidth() && keypoint.y() < displayImage->getHeight())
                cv::circle(displayedImage, cv::Point2f(keypoint.x(), keypoint.y()), m_radius, cv::Scalar(uni(rng), uni(rng), uni(rng)), m_thickness);
        }
    }
}

void SolAR2DOverlayOpencv::drawLines(const std::vector<Keyline>& keylines, SRef<Image> displayImage)
{
    // Image where lines will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);
    cv::Scalar color;
	cv::Point2f start, end;

	if (!m_randomColor)
	{
		color = cv::Scalar(m_color[0], m_color[1], m_color[2]);
		for (int i = 0; i < keylines.size(); i++)
		{
			start = cv::Point2f(keylines[i].getStartPointX(), keylines[i].getStartPointY());
			end = cv::Point2f(keylines[i].getEndPointX(), keylines[i].getEndPointY());
			SolAROpenCVHelper::drawCVLine(displayedImage, start, end, color, m_thickness);
		}
	}
	else
	{
		std::random_device rd;     // only used once to initialise (seed) engine
		std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
		std::uniform_int_distribution<int> uni(0, 255); // guaranteed unbiased

		for (int i = 0; i < keylines.size(); i++)
		{
			start = cv::Point2f(keylines[i].getStartPointX(), keylines[i].getStartPointY());
			end = cv::Point2f(keylines[i].getEndPointX(), keylines[i].getEndPointY());
			color = cv::Scalar(uni(rng), uni(rng), uni(rng));
			SolAROpenCVHelper::drawCVLine(displayedImage, start, end, color, m_thickness);
		}
	}
}

void SolAR2DOverlayOpencv::drawLines(const std::vector<Edge2Df>& ln2d, SRef<Image> displayImage)
{
	// Image where lines will be displayed
	cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);
	cv::Scalar color;
	cv::Point2f start, end;

	if (!m_randomColor)
	{
		color = cv::Scalar(m_color[0], m_color[1], m_color[2]);
		for (int i = 0; i < ln2d.size(); i++)
		{
			start = cv::Point2f(ln2d[i].p1.getX(), ln2d[i].p1.getY());
			end = cv::Point2f(ln2d[i].p2.getX(), ln2d[i].p2.getY());
			SolAROpenCVHelper::drawCVLine(displayedImage, start, end, color, m_thickness);
		}
	}
	else
	{
		std::random_device rd;     // only used once to initialise (seed) engine
		std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
		std::uniform_int_distribution<int> uni(0, 255); // guaranteed unbiased

		for (int i = 0; i < ln2d.size(); i++)
		{
			start = cv::Point2f(ln2d[i].p1.getX(), ln2d[i].p1.getY());
			end = cv::Point2f(ln2d[i].p2.getX(), ln2d[i].p2.getY());
			color = cv::Scalar(uni(rng), uni(rng), uni(rng));
			SolAROpenCVHelper::drawCVLine(displayedImage, start, end, color, m_thickness);
		}
	}
}

void SolAR2DOverlayOpencv::drawContour (const Contour2Df & contour, SRef<Image> displayImage)
{
    // image where contours will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);
    cv::Scalar color;

    if (!m_randomColor)
        color = cv::Scalar(m_color[0], m_color[1], m_color[2]);
    else
    {
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased

        color = cv::Scalar(uni(rng), uni(rng), uni(rng));
    }

    for (int i = 0; i < contour.size(); i++)
    {
        if (i != contour.size() - 1)
        {
            cv::Point2f pt_a(contour[i].getX(), contour[i].getY());
            cv::Point2f pt_b(contour[i+1].getX(),contour[i+1].getY());

            SolAROpenCVHelper::drawCVLine(displayedImage, pt_a, pt_b, cv::Scalar(color[0],color[1], color[2]), m_thickness);
        }
        else
        {
            //the contours loops to the first point
            cv::Point2f pt_a(contour[i].getX(), contour[i].getY());
            cv::Point2f pt_b(contour[0].getX(), contour[0].getY());

            SolAROpenCVHelper::drawCVLine(displayedImage, pt_a, pt_b, cv::Scalar(color[0],color[1], color[2]), m_thickness);
        }
    }
}

void SolAR2DOverlayOpencv::drawContours (const std::vector<Contour2Df> & contours, SRef<Image> displayImage)
{
    // image where contours will be displayed
    cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);
    cv::Scalar color;

    for (auto contour : contours)
    {
        if (!m_randomColor)
            color = cv::Scalar(m_color[0], m_color[1], m_color[2]);
        else
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased

            color = cv::Scalar(uni(rng), uni(rng), uni(rng));
        }

        for (int i = 0; i < contour.size(); i++)
        {
            if (i != contour.size() - 1)
            {
                cv::Point2f pt_a(contour[i].getX(), contour[i].getY());
                cv::Point2f pt_b(contour[i+1].getX(),contour[i+1].getY());

                SolAROpenCVHelper::drawCVLine(displayedImage, pt_a, pt_b, cv::Scalar(color[0],color[1], color[2]), m_thickness);
            }
            else
            {
                //the contours loops to the first point
                cv::Point2f pt_a(contour[i].getX(), contour[i].getY());
                cv::Point2f pt_b(contour[0].getX(), contour[0].getY());

                SolAROpenCVHelper::drawCVLine(displayedImage, pt_a, pt_b, cv::Scalar(color[0],color[1], color[2]), m_thickness);
            }
        }
    }
}

void SolAR2DOverlayOpencv::drawSBPattern (const SquaredBinaryPattern & pattern, SRef<Image> displayImage)
{
    // image where contours will be displayed
    cv::Mat displayImageCV = SolAROpenCVHelper::mapToOpenCV(displayImage);
    displayImageCV.setTo(0);

    int patternSize = pattern.getSize();
    SquaredBinaryPatternMatrix matrix = pattern.getPatternMatrix();

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

void SolAR2DOverlayOpencv::putText(const std::string & text, Point2Df origin, double fontScale, std::vector<int> color, SRef<Image> displayImage)
{
	cv::Mat displayedImage = SolAROpenCVHelper::mapToOpenCV(displayImage);
	cv::Scalar chosenColor = cv::Scalar(color[0], color[1], color[2]);
	int font;

	if (m_font == -1)
		font = cv::FONT_HERSHEY_SIMPLEX;
	else
		font = m_font;

	cv::putText(displayedImage,
				text,
				cv::Point(origin.getX(), origin.getY()),
				font,
				fontScale,
				chosenColor,
				m_thickness);
}
}
}
}
