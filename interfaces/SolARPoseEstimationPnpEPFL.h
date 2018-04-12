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

#ifndef SOLARPOSEESTIMATIONPNPEPFL_H
#define SOLARPOSEESTIMATIONPNPEPFL_H

#include <vector>
#include "opencv2/core.hpp"

#include "api/solver/pose/I3DTransformFinder.h"

#include "SolAROpencvAPI.h"
#include "ComponentBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARPoseEstimationPnpEPFL : public org::bcom::xpcf::ComponentBase,
    public api::solver::pose::I3DTransformFinder
{
public:
    SolARPoseEstimationPnpEPFL();

    void set_internal_parameters(const double uc, const double vc,const double fu, const double fv);
    void set_maximum_number_of_correspondences(const int n);
    void reset_correspondences(void);
    void add_correspondence(const double X, const double Y, const double Z,const double u, const double v);
    double compute_pose(double R[3][3], double T[3]);
    void relative_error(double & rot_err, double & transl_err,const double Rtrue[3][3], const double ttrue[3],const double Rest[3][3], const double test[3]);
    void print_pose(const double R[3][3], const double t[3]);
    double reprojection_error(const double R[3][3], const double t[3]);

    FrameworkReturnCode estimate( const std::vector<SRef<Point2Df>> & imagePoints,
                                  const std::vector<SRef<Point3Df>> & worldPoints,
                                  Pose & pose) override;

    void unloadComponent () override final;

    void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams)  override;

    XPCF_DECLARE_UUID("a38edf79-f0dc-45ca-92fc-2b336fceedf9");

private:
    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
    void choose_control_points(void);
    void compute_barycentric_coordinates(void);
    void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
    void compute_ccs(const double * betas, const double * ut);
    void compute_pcs(void);
    void solve_for_sign(void);
    void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
    void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
    void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
    void qr_solve(CvMat * A, CvMat * b, CvMat * X);

    double dot(const double * v1, const double * v2);
    double dist2(const double * p1, const double * p2);

    void compute_rho(double * rho);
    void compute_L_6x10(const double * ut, double * l_6x10);

    void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
    void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,double cb[4], CvMat * A, CvMat * b);
    double compute_R_and_t(const double * ut, const double * betas,double R[3][3], double t[3]);
    void estimate_R_and_t(double R[3][3], double t[3]);
    void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],double R_src[3][3], double t_src[3]);
    void mat_to_quat(const double R[3][3], double q[4]);


    double uc, vc, fu, fv;
    double * pws, *us, *alphas, *pcs;
    int maximum_number_of_correspondences;
    int number_of_correspondences;

    double cws[4][3], ccs[4][3];
    double cws_determinant;
};

}
}
}

#endif // SOLARPOSEESTIMATIONOPENCV_H
