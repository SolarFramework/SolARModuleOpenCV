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

#include "SolARSideBySideOverlayOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/video/video.hpp"
#include <random>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARSideBySideOverlayOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARSideBySideOverlayOpencv::SolARSideBySideOverlayOpencv():ConfigurableBase(xpcf::toUUID<SolARSideBySideOverlayOpencv>())
{
    addInterface<api::display::ISideBySideOverlay>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    m_color.resize(3);

    params->wrapUnsignedInteger("thickness", m_thickness);
    params->wrapUnsignedIntegerVector("color", m_color);
    params->wrapUnsignedInteger("randomColor", m_randomColor);
    params->wrapInteger("maxMatches", m_maxMatches);
    LOG_DEBUG(" SolARSideBySideOverlayOpencv constructor");

}


void SolARSideBySideOverlayOpencv::drawMatchesLines(const SRef<Image> image1, const SRef<Image> image2, SRef<Image> & outImage, const std::vector <SRef<Point2Df>> & points_image1, const std::vector <SRef<Point2Df>> & points_image2)
{
    if (outImage == nullptr)
    {
        outImage = xpcf::utils::make_shared<Image>(image1->getWidth()+image2->getWidth(), std::max(image1->getHeight(), image2->getHeight()), image1->getImageLayout(), image1->getPixelOrder(), image1->getDataType());
    }
    else if ((outImage->getWidth() != image1->getWidth()+image2->getWidth()) || (outImage->getHeight() != std::max(image1->getHeight(), image2->getHeight())))
    {
        outImage->setSize(image1->getWidth()+image2->getWidth(), std::max(image1->getHeight(), image2->getHeight()));
    }

    cv::Mat img1, img2, outImg;
    Point2Df point1, point2;
    int img1_width=image1->getWidth();

    img1=SolAROpenCVHelper::mapToOpenCV(image1);
    img2=SolAROpenCVHelper::mapToOpenCV(image2);
    outImg=SolAROpenCVHelper::mapToOpenCV(outImage);

    outImg.setTo(0);

    img1.copyTo(outImg(cv::Rect(0, 0, img1_width, image1->getHeight())));
    img2.copyTo(outImg(cv::Rect(img1_width, 0, image2->getWidth(), image2->getHeight())));

    int nbPoints = std::min(points_image1.size(), points_image2.size());
    if (m_maxMatches >= 0)
        nbPoints = std::min((int)m_maxMatches, nbPoints);

    if (!m_randomColor)
    {
        for (int i = 0;i<nbPoints;++i){
            point1 = *(points_image1.at(i));
            point2 = *(points_image2.at(i));
            cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(m_color[0],m_color[1],m_color[2]),m_thickness);
        }
    }
    else
    {
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
        for (int i = 0;i<nbPoints;++i){
            point1 = *(points_image1.at(i));
            point2 = *(points_image2.at(i));
            cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
        }
    }
}

}
}
}
