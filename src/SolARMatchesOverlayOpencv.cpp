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

#include "SolARMatchesOverlayOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/video/video.hpp"
#include <random>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMatchesOverlayOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARMatchesOverlayOpencv::SolARMatchesOverlayOpencv():ConfigurableBase(xpcf::toUUID<SolARMatchesOverlayOpencv>())
{
    declareInterface<api::display::IMatchesOverlay>(this);
    m_color.resize(3);

    declareProperty("thickness", m_thickness);
    declarePropertySequence("color", m_color);
    declareProperty("mode", m_mode); // COLOR, RANDOM, FADING
    declareProperty("maxMatches", m_maxMatches);

    LOG_DEBUG(" SolARMatchesOverlayOpencv constructor");
}

void SolARMatchesOverlayOpencv::draw(const SRef<Image> image1, const SRef<Image> image2, SRef<Image> & outImage, const std::vector <Point2Df> & points_image1, const std::vector <Point2Df> & points_image2, const std::vector<DescriptorMatch> & matches)
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

    int nbPoints;

    if (matches.empty())
    {
        nbPoints = std::min(points_image1.size(), points_image2.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(i);
                point2 = points_image2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(i);
                point2 = points_image2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
            }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(i);
                point2 = points_image2.at(i);
                float distance = ((point1/image1->getWidth())-(point2/image2->getWidth())).norm();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");
    }
    else
    {
        nbPoints = static_cast<int>(matches.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(matches[i].getIndexInDescriptorA());
                point2 = points_image2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(matches[i].getIndexInDescriptorA());
                point2 = points_image2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
             }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(matches[i].getIndexInDescriptorA());
                point2 = points_image2.at(matches[i].getIndexInDescriptorB());
                float distance = ((point1/image1->getWidth())-(point2/image2->getWidth())).norm();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");
    }
}

void SolARMatchesOverlayOpencv::draw(const SRef<Image> image1, const SRef<Image> image2, SRef<Image> & outImage, const std::vector <Keypoint> & points_image1, const std::vector<Keypoint> & points_image2, const std::vector<DescriptorMatch> & matches)
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

    int nbPoints;

    if (matches.empty())
    {
        nbPoints = std::min(points_image1.size(), points_image2.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(i);
                point2 = points_image2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(i);
                point2 = points_image2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
            }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(i);
                point2 = points_image2.at(i);
                float distance = ((point1/image1->getWidth())-(point2/image2->getWidth())).norm();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");
    }
    else
    {
        nbPoints = static_cast<int>(matches.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(matches[i].getIndexInDescriptorA());
                point2 = points_image2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(matches[i].getIndexInDescriptorA());
                point2 = points_image2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
             }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points_image1.at(matches[i].getIndexInDescriptorA());
                point2 = points_image2.at(matches[i].getIndexInDescriptorB());
                float distance = ((point1/image1->getWidth())-(point2/image2->getWidth())).norm();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX()+img1_width,point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");
    }
}

void SolARMatchesOverlayOpencv::draw(const SRef<Image> image, SRef<Image> & outImage, const std::vector <Point2Df> & points1, const std::vector <Point2Df> & points2, const std::vector<DescriptorMatch> & matches)
{
    if (outImage == nullptr)
    {
        outImage = xpcf::utils::make_shared<Image>(image->getWidth(), image->getHeight(), image->getImageLayout(), image->getPixelOrder(), image->getDataType());
    }
    else if (outImage->getWidth() != image->getWidth() || outImage->getHeight() != image->getHeight())
    {
        outImage->setSize(image->getWidth(), image->getHeight());
    }

    cv::Mat img, outImg;
    Point2Df point1, point2;

    img=SolAROpenCVHelper::mapToOpenCV(image);
    outImg=SolAROpenCVHelper::mapToOpenCV(outImage);

    outImg.setTo(0);

    img.copyTo(outImg(cv::Rect(0, 0, image->getWidth(), image->getHeight())));

    int nbPoints;

    if (matches.empty())
    {
        nbPoints = std::min(points1.size(), points2.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(i);
                point2 = points2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(i);
                point2 = points2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
            }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(i);
                point2 = points2.at(i);
                float distance = (point1-point2).norm()/image->getWidth();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");

    }
    else
    {
        nbPoints = static_cast<int>(matches.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(matches[i].getIndexInDescriptorA());
                point2 = points2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(matches[i].getIndexInDescriptorA());
                point2 = points2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
             }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(matches[i].getIndexInDescriptorA());
                point2 = points2.at(matches[i].getIndexInDescriptorB());
                float distance = (point1-point2).norm()/image->getWidth();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");
    }
}

void SolARMatchesOverlayOpencv::draw(const SRef<Image> image, SRef<Image> & outImage, const std::vector <Keypoint> & points1, const std::vector <Keypoint> & points2, const std::vector<DescriptorMatch> & matches)
{
    if (outImage == nullptr)
    {
        outImage = xpcf::utils::make_shared<Image>(image->getWidth(), image->getHeight(), image->getImageLayout(), image->getPixelOrder(), image->getDataType());
    }
    else if (outImage->getWidth() != image->getWidth() || outImage->getHeight() != image->getHeight())
    {
        outImage->setSize(image->getWidth(), image->getHeight());
    }

    cv::Mat img, outImg;
    Point2Df point1, point2;

    img=SolAROpenCVHelper::mapToOpenCV(image);
    outImg=SolAROpenCVHelper::mapToOpenCV(outImage);

    outImg.setTo(0);

    img.copyTo(outImg(cv::Rect(0, 0, image->getWidth(), image->getHeight())));

    int nbPoints;

    if (matches.empty())
    {
        nbPoints = std::min(points1.size(), points2.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(i);
                point2 = points2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(i);
                point2 = points2.at(i);
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
            }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(i);
                point2 = points2.at(i);
                float distance = (point1-point2).norm()/image->getWidth();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");

    }
    else
    {
        nbPoints = static_cast<int>(matches.size());
        if (m_maxMatches >= 0)
            nbPoints = std::min((int)m_maxMatches, nbPoints);

        if (m_mode.compare("COLOR") == 0)
        {
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(matches[i].getIndexInDescriptorA());
                point2 = points2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(m_color[2],m_color[1],m_color[0]),m_thickness);
            }
        }
        else if (m_mode.compare("RANDOM") == 0)
        {
            std::random_device rd;     // only used once to initialise (seed) engine
            std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
            std::uniform_int_distribution<int> uni(0,255); // guaranteed unbiased
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(matches[i].getIndexInDescriptorA());
                point2 = points2.at(matches[i].getIndexInDescriptorB());
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(uni(rng),uni(rng),uni(rng)),m_thickness);
             }
        }
        else if (m_mode.compare("FADING") == 0)
        {
            float halfMinDistanceRatioGreen = m_minDistanceRatioGreen/2.0f;
            for (int i = 0;i<nbPoints;++i){
                point1 = points1.at(matches[i].getIndexInDescriptorA());
                point2 = points2.at(matches[i].getIndexInDescriptorB());
                float distance = (point1-point2).norm()/image->getWidth();
                std::vector<unsigned int> color = {0,0,0};

                if (distance >= m_minDistanceRatioGreen)
                    color = {0, 255, 0};
                else
                {
                    if (distance < halfMinDistanceRatioGreen)
                    {
                        color[0] = 255;
                        color[1] = (unsigned int)(255*distance/halfMinDistanceRatioGreen);
                    }
                    else
                    {
                        color[0] = (unsigned int)(255*(m_minDistanceRatioGreen - distance)/halfMinDistanceRatioGreen);
                        color[1] = 255;
                    }
                }
                cv::line(outImg,cv::Point2f(point1.getX(), point1.getY()),cv::Point2f(point2.getX(),point2.getY()),cv::Scalar(color[2],color[1],color[0]),m_thickness);
            }
        }
        else
            LOG_WARNING ("For SolARMatchesOverlayOpenCV, mode should be either COLOR, RANDOM or FADING");
    }
}

}
}
}
