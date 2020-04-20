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

#include "SolAROpenCVHelper.h"
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "datastructure/DescriptorBuffer.h"

using namespace org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

static std::map<DescriptorDataType,uint32_t> solarDescriptor2cvType =
{{ DescriptorDataType::TYPE_8U,CV_8U},{DescriptorDataType::TYPE_32F,CV_32F}};



static std::map<std::tuple<uint32_t,std::size_t,uint32_t>,int> solar2cvTypeConvertMap = {{std::make_tuple(8,1,3),CV_8UC3},{std::make_tuple(8,1,1),CV_8UC1}};

static std::map<int,std::pair<Image::ImageLayout,Image::DataType>> cv2solarTypeConvertMap = {{CV_8UC3,{Image::ImageLayout::LAYOUT_BGR,Image::DataType::TYPE_8U}},
                                                                                                      {CV_8UC1,{Image::ImageLayout::LAYOUT_GREY,Image::DataType::TYPE_8U}}};

uint32_t SolAROpenCVHelper::deduceOpenDescriptorCVType(DescriptorDataType querytype){
    return solarDescriptor2cvType.at(querytype);
}


int SolAROpenCVHelper::deduceOpenCVType(SRef<Image> img)
{
    // TODO : handle safe mode if missing map entry
    // is it ok when destLayout != img->ImageLayout ?
    return solar2cvTypeConvertMap.at(std::forward_as_tuple(img->getNbBitsPerComponent(),1,img->getNbChannels()));
}

void SolAROpenCVHelper::mapToOpenCV (SRef<Image> imgSrc, cv::Mat& imgDest)
{
    cv::Mat imgCV(imgSrc->getHeight(),imgSrc->getWidth(),deduceOpenCVType(imgSrc), imgSrc->data()); 
    imgDest = imgCV;
}


cv::Mat SolAROpenCVHelper::mapToOpenCV (SRef<Image> imgSrc)
{
    cv::Mat imgCV(imgSrc->getHeight(),imgSrc->getWidth(),deduceOpenCVType(imgSrc), imgSrc->data());
    return imgCV;
}

FrameworkReturnCode SolAROpenCVHelper::convertToSolar (cv::Mat&  imgSrc, SRef<Image>& imgDest)
{
    if (cv2solarTypeConvertMap.find(imgSrc.type()) == cv2solarTypeConvertMap.end() || imgSrc.empty()) {
        return FrameworkReturnCode::_ERROR_LOAD_IMAGE;
    }
    std::pair<Image::ImageLayout,Image::DataType> type = cv2solarTypeConvertMap.at(imgSrc.type());
    imgDest = utils::make_shared<Image>(imgSrc.ptr(), imgSrc.cols, imgSrc.rows, type.first, Image::PixelOrder::INTERLEAVED, type.second);

    return FrameworkReturnCode::_SUCCESS;
}

std::vector<cv::Point2i> SolAROpenCVHelper::convertToOpenCV (const Contour2Di &contour)
{
    std::vector<cv::Point2i> output;
    for (int i = 0; i < contour.size(); i++)
    {
        output.push_back(cv::Point2i(contour[i]->getX(), contour[i]->getY()));
    }
    return output;
}

std::vector<cv::Point2f> SolAROpenCVHelper::convertToOpenCV (const Contour2Df &contour)
{
    std::vector<cv::Point2f> output;
    for (int i = 0; i < contour.size(); i++)
    {
        output.push_back(cv::Point2f(contour[i].getX(), contour[i].getY()));
    }
    return output;
}

// Compute the intersection between a edge and a rectangle
bool Liang_Barsky (cv::Point2f& p1, cv::Point2f& p2, Rectanglei& rect, cv::Point2f& p1_out, cv::Point2f& p2_out)
{
    int p[4];
    int q[4];

    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;

    int xmin = rect.startX;
    int xmax = xmin + rect.size.width;
    int ymin = rect.startY;
    int ymax = ymin + rect.size.height;

    int dx = x2 - x1;
    int dy = y2 - y1;

    p[0] = -dx;
    p[1] = dx;
    p[2] = -dy;
    p[3] = dy;

    q[0] = x1 - xmin;
    q[1] = xmax - x1;
    q[2] = y1 - ymin;
    q[3] = ymax - y1;

    for (int i = 0 ; i < 4; i++)
    {
        if (p[i] == 0)
        {
            // line is parallel to one of the clipping boundary.
            if (i < 2)
            {
                // Line is horizontal
                if (y1 < ymin)
                {
                    y1 = ymin;
                }
                if(y2>ymax)
                {
                    y2=ymax;
                }
                p1_out.x = x1;
                p1_out.y = y1;
                p2_out.x = x2;
                p2_out.y = y2;
                return true;
            }

            if (i>1)
            {
                // Line is vertical
                if(x1<xmin)
                {
                    x1=xmin;
                }

                if(x2>xmax)
                {
                    x2=xmax;
                }

                p1_out.x = x1;
                p1_out.y = y1;
                p2_out.x = x2;
                p2_out.y = y2;
                return true;
            }
        }
    }

    float t1 = 0.0f;
    float t2 = 1.0f;
    float temp;

    for(int i=0;i<4;i++)
    {
        temp=(float)q[i]/(float)p[i];

        if(p[i]<0)
        {
            if(t1<=temp)
                t1=temp;
        }
        else
        {
            if(t2>temp)
                t2=temp;
        }
    }

    if (t1 < t2)
    {
        p1_out.x = x1 + t1 * p[1];
        p2_out.x = x1 + t2 * p[1];
        p1_out.y = y1 + t1 * p[3];
        p2_out.y = y1 + t2 * p[3];
        return true;
    }
    return false;
}

void SolAROpenCVHelper::drawCVLine (cv::Mat& inputImage, cv::Point2f& p1, cv::Point2f& p2, cv::Scalar color, int thickness)
{
    Rectanglei rect = {0, 0, Sizei{(uint32_t)inputImage.cols, (uint32_t)inputImage.rows}};
    float x1, x2, y1, y2;

    x1 = p1.x;
    y1 = p1.y;
    x2 = p2.x;
    y2 = p2.y;

    if (x1>=0 && x1 < inputImage.cols && y1>=0 && y1 < inputImage.rows &&
        x2>=0 && x2 < inputImage.cols && y2>=0 && y2 < inputImage.rows)
        cv::line(inputImage, p1, p2, color, thickness, cv::LINE_AA);
    else
    {
        cv::Point2f p1_result;
        cv::Point2f p2_result;
        if (Liang_Barsky(p1, p2, rect, p1_result, p2_result))
            cv::line(inputImage, p1_result, p2_result, color, thickness, cv::LINE_AA);
    }
}

}
}
}

