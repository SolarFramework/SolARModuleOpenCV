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

#include "SolARImageFilterWallisOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/photo.hpp"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;

template<> SolAR::MODULES::OPENCV::SolARImageFilterWallisOpencv * xpcf::ComponentFactory::createInstance<SolAR::MODULES::OPENCV::SolARImageFilterWallisOpencv>();


namespace SolAR {
namespace MODULES {
namespace OPENCV {


SolARImageFilterWallisOpencv::SolARImageFilterWallisOpencv():xpcf::ConfigurableBase(xpcf::toMap<SolARImageFilterWallisOpencv>())
{
    declareInterface<SolAR::api::image::IImageFilter>(this);


    declareProperty<uint32_t>("desiredMean",m_desiredMean);
    declareProperty<uint32_t>("desiredStdDev",m_desiredStdDev);
    declareProperty<float>("A",m_A);
    declareProperty<float>("B",m_B);
    declareProperty<uint32_t>("windowWidth",m_windowWidth);
    declareProperty<std::string>("denoisingMethod",m_denoisingMethod);

}


SolARImageFilterWallisOpencv::~SolARImageFilterWallisOpencv()
{

}

void SolARImageFilterWallisOpencv::unloadComponent ()
{
    // provide component cleanup strategy
    // default strategy is to delete self, uncomment following line in this case :
    // delete this;
    return;
}

xpcf::XPCFErrorCode SolARImageFilterWallisOpencv::onConfigured()
{
    // Add custom onConfigured code
    return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARImageFilterWallisOpencv::filter(const SRef<SolAR::datastructure::Image> input, SRef<SolAR::datastructure::Image> & output)
{
    SRef<datastructure::Image> imageGrey;
    if (input->getImageLayout() != datastructure::Image::ImageLayout::LAYOUT_GREY)
        m_convertor.convert(input, imageGrey, datastructure::Image::ImageLayout::LAYOUT_GREY);
    else
        imageGrey = input;

    cv::Mat cvImgGrey8U, cvImgFiltered, cvImgSmooth8U, cvImgSmooth64F, cvImgUniform64F, cvImgSum64F, cvImgCount64F, cvImgLocalMean64F, cvImgTemp64F, cvImgTemp2_64F, cvImgS_64F, cvKernel64F;
    SolAROpenCVHelper::mapToOpenCV(imageGrey,cvImgGrey8U);


    uint32_t width = m_windowWidth;
    if (m_windowWidth%2 == 0)
        width++;

    if (m_denoisingMethod == "GaussianBlur")
    {
        cv::GaussianBlur(cvImgGrey8U, cvImgSmooth8U, cv::Size(width, width),1,1);
    }
    else if (m_denoisingMethod == "NonLocalMeans")
    {
        cv::fastNlMeansDenoising(cvImgGrey8U, cvImgSmooth8U, 3, 7, width);
    }
    else if (m_denoisingMethod == "None")
    {
        cvImgSmooth8U = cvImgGrey8U;
    }
    else
    {
        LOG_WARNING("Denoising method named {} for Wallis Filter is unknown. Choose between None, GaussianBlur, or NonLocalMeans", m_denoisingMethod);
        cvImgSmooth8U = cvImgGrey8U;
    }

    cvImgSmooth8U.convertTo(cvImgSmooth64F, CV_64FC1);

    cvImgUniform64F = cv::Mat::ones(cvImgSmooth64F.rows, cvImgSmooth64F.cols, CV_64FC1);
    cvKernel64F = cv::Mat::ones(width, width, CV_64FC1);


    // Output image put on 16 bits to handle the overflows of 8bits pixels
    cv::filter2D(cvImgSmooth64F, cvImgSum64F, CV_64FC1, cvKernel64F, cv::Point(-1,-1), 0, cv::BORDER_CONSTANT);
    cv::filter2D(cvImgUniform64F, cvImgCount64F, CV_64FC1, cvKernel64F, cv::Point(-1,-1), 0, cv::BORDER_CONSTANT);

    cv::divide(cvImgSum64F, cvImgCount64F, cvImgLocalMean64F);

    cvImgTemp64F =  cvImgSmooth64F - cvImgLocalMean64F;

    cv::filter2D(cvImgTemp64F.mul(cvImgTemp64F), cvImgTemp2_64F, -1, cvKernel64F, cv::Point(-1,-1), 0, cv::BORDER_CONSTANT);
    //cvImgTemp2_64F /= cvImgCount64F;

    cv::sqrt(cvImgTemp2_64F/cvImgCount64F, cvImgS_64F);

    cvImgFiltered = ((cvImgSmooth64F - cvImgLocalMean64F)*m_desiredStdDev/**m_A*/)/(cvImgS_64F+m_A)
               + m_B * m_desiredMean
               + (1-m_B) * cvImgLocalMean64F;

    // Clip values to avoid overflow
    for (int i = 0; i < cvImgFiltered.rows; i++)
        for (int j = 0; j < cvImgFiltered.cols; j++)
        {
            if (cvImgFiltered.at<double>(i,j) < 0)
                cvImgFiltered.at<double>(i,j) = 0;
            if (cvImgFiltered.at<double>(i,j) > 255)
                cvImgFiltered.at<double>(i,j) = 255;
        }
    cvImgFiltered.convertTo(cvImgFiltered, CV_8UC1);

    SolAROpenCVHelper::convertToSolar(cvImgFiltered, output);

    return FrameworkReturnCode::_SUCCESS;

}


} // namespace TOOLS
} // namespace MODULES
} // namespace SolAR
