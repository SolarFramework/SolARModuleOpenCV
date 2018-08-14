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

#ifndef SolARSVDFundamentalMatrixDecomposerOpencv_H
#define SolARSVDFundamentalMatrixDecomposerOpencv_H
#include <vector>

#include "api/solver/pose/I2Dto3DTransformDecomposer.h"
#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <vector>
#include "opencv2/core.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {
/**
 * @class SolARSVDFundamentalMatrixDecomposerOpencv
 * @brief Decomposes Fundamental matrix on a set of camera poses based on opencv SVD solving.
 */
class SOLAROPENCV_EXPORT_API SolARSVDFundamentalMatrixDecomposerOpencv : public org::bcom::xpcf::ComponentBase,
    public api::solver::pose::I2DTO3DTransformDecomposer{
public:
    ///@brief SolARSVDFundamentalMatrixDecomposerOpencv constructor.
    SolARSVDFundamentalMatrixDecomposerOpencv();
    ///@brief SolARSVDFundamentalMatrixDecomposerOpencv destructor.
    ~SolARSVDFundamentalMatrixDecomposerOpencv();

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams)  override;

    /// @brief Decomposes Fundamental matrix four possible camera poses based on opencv svd solving.
    /// @param[in] The Fundamental matrix.
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    /// @param[out] Decomposed camera poses in the world coordinate system.
    bool decompose(const Transform2Df& F,
                   std::vector<Transform3Df>& decomposedPoses) override;

    void unloadComponent () override final;


private:
    cv::Mat_<double> m_camMatrix;
    cv::Mat_<double> m_camDistorsion;

    bool decomposeInternal(cv::Mat_<double>& E,
                           cv::Mat_<double>& R1,
                           cv::Mat_<double>& R2,
                           cv::Mat_<double>& t1,
                           cv::Mat_<double>& t2);
    /// @brief Decomposes Esstential matrix on SVD representation.
    /// @param[in] The Essential matrix.
    /// @param[out] Matrix U of the essntial matrix.
    /// @param[out] Matrix V of the essntial matrix.
    /// @param[out] Matrix W of the essntial matrix.
    void takeSVDOfE(cv::Mat_<double>& E,
                    cv::Mat& svd_u,
                    cv::Mat& svd_vt,
                    cv::Mat& svd_w);


    void fillposes(const cv::Mat_<double>& R1,
                   const cv::Mat_<double>& R2,
                   const cv::Mat_<double>& t1,
                   const cv::Mat_<double>& t2,
                   std::vector<Transform3Df>&decomposedPoses);

};

}
}
}

#endif // SolARHomographyEstimationOpencv_H
