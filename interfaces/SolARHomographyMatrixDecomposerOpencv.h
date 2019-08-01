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

#ifndef SolARHomographyMatrixMatrixDecomposerOpencv_H
#define SolARHomographyMatrixMatrixDecomposerOpencv_H
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
 * @class SolARHomographyMatrixDecomposerOpencv
 * @brief <B>Decomposes a homography matrix to extract four possible 3D poses.</B>
 * <TT>UUID: b5fab395-2184-4123-b0d5-4af74d0a2d79</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARHomographyMatrixDecomposerOpencv : public org::bcom::xpcf::ComponentBase,
   public api::solver::pose::I2Dto3DTransformDecomposer {

public:
    SolARHomographyMatrixDecomposerOpencv();

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams)  override;

    /// @brief decompose a transform 2d to a transform 3d (4  possible poses {R1,t1},{R1,t2}, {R2,t1}, {R2,t2}).
    /// @param[in] Transform 2D (fundamental matrxi, homgraphy..).
    /// @param[out] Set (04 possibles cases) of the decomposed camera poses in the world coordinate system expressed as Transform3D.
    bool decompose(const Transform2Df & F,
                   std::vector<Transform3Df> & decomposedPoses) override;

    void unloadComponent () final;

private:
    cv::Mat_<double> m_camMatrix;
    cv::Mat_<double> m_camDistorsion;

};
}
}
}
#endif // SOLARHOMOGRAPHYMATRIXDECOMPOSITIONOPENCV_H



