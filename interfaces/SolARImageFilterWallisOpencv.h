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

#ifndef INTERFACESSolARImageFilterWallisOpencv_H
#define INTERFACESSolARImageFilterWallisOpencv_H
#include <xpcf/component/ConfigurableBase.h>
#include "interfaces/SolARImageConvertorOpencv.h"
#include "api/image/IImageFilter.h"


namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageFilterWallisOpencv
 * @brief <B>Apply a Wallis Filter to an image.</B>
 * <TT>UUID: 77113af0-4af2-4c45-92d4-fd1ea77b56cc</TT>
 *
 * Wallis filter is a local adaptive median filter, i.e., it adjusts pixel brightness values in local areas. In contrast, a global filters use the same contrast values throughout an entire image, and therefore can not enhance details in both high- and low-level-of-brightness areas simultaneously.
 * The Wallis Formula implemented here is the following one:
 * \f[
 *     I_{j}^{(w)} (x,y) =  \frac{desiredMean \: I_{j}^{(G)} (x,y)}{s+A} + B \: desiredStdDev + (1-B)m
 * \f]
 * where \f$I_{j}^{(w)}\f$ is pre-processed output image, \f$s\f$ is the local mean, and \f$m\f$ is the local standard deviation. This local mean and local deviation are computed based on a window of size \f$windowWidth\f$.
 *
 * More information are available in:
 * M. Gaiani, F. Remondino, F. I. Apollonio, and A. Ballabeni, “An advanced pre-processing pipeline to improve automated photogrammetric reconstructions of architectural scenes,” Remote sensing, vol. 8, no. 3, p. 178, 2016.
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ desiredMean ,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ unsigned int, [0..MAX UINT], 127 }}
 * @SolARComponentProperty{ desiredStdDev,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ unsigned int, [0..MAX INT], 60 }}
 * @SolARComponentProperty{ A,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 2.1 }}
 * @SolARComponentProperty{ B,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.7 }}
 * @SolARComponentProperty{ windowWidth ,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ unsigned int, [0..MAX UINT], 14 }}
 * @SolARComponentProperty{ denoisingMethod,
 *                          The denoising method applied before the wallis filter. Accepted values: None\, GaussianBlur\, or NonLocalMeans,
 *                          @SolARComponentPropertyDescString{ "None" }}
 * @SolARComponentPropertiesEnd
 */

class SolARImageFilterWallisOpencv : public org::bcom::xpcf::ConfigurableBase, virtual public SolAR::api::image::IImageFilter
{
public:
    SolARImageFilterWallisOpencv();
    ~SolARImageFilterWallisOpencv() override;
    void unloadComponent () override final;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    /// @brief This method filters an image source to an image destination.
    /// @param[in] input: input image to filter
    /// @param[out] output: output image filtred
    /// @return FrameworkReturnCode::_SUCCESS_ id filtering succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode filter(const SRef<SolAR::datastructure::Image> input, SRef<SolAR::datastructure::Image> & output) override;

private:
    uint32_t m_desiredMean = 127;
    uint32_t m_desiredStdDev = 60;
    float m_A = 2.1f;
    float m_B = 0.7f;
    uint32_t m_windowWidth = 14;
    std::string m_denoisingMethod = "None";
    SolARImageConvertorOpencv m_convertor;
};


} // namespace OPENCV
} // namespace MODULES
} // namespace SolAR

#endif // INTERFACESSolARImageFilterWallisOpencv_H
