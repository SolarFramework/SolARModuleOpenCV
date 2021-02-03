#ifndef SOLARGEOMETRICMATCHESFILTEROPENCV_H
#define SOLARGEOMETRICMATCHESFILTEROPENCV_H

#include "api/features/IMatchesFilter.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARGeometricMatchesFilterOpencv
 * @brief <B>Filters a set of matches based on geometric constraints.</B>
 * <TT>UUID: 3731691e-2c4c-4d37-a2ce-06d1918f8d41</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ confidence,
 *                          The desirable level of confidence (propability) that the estimated matrix is correct.,
 *                          @SolARComponentPropertyDescNum{ float, [0..1], 0.99f }}
 * @SolARComponentProperty{ outlierDistanceRatio,
 *                          Here we are using a RANSAC method to keep only inliers matches.<br>
 *                            This attribute is the ratio between the maximum distance in pixels between source points and the maximum distance in pixels to the epipolar line for which point is considered as a outlier.<br>
 *                            The higher is this ratio\, the more you will keep inliers to estimate your 2D transform\, but the less this estimation will be correct.<br>
 *                            By default\, this value is set to the one proposed by [Snavely07 4.1],
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.006f }}
 * @SolARComponentProperty{ epilinesDistance,
 *                          threshold to valid matches based on distance to epilines,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX_FLOAT], 10.f }}
 * @SolARComponentPropertiesEnd
 */

class SOLAROPENCV_EXPORT_API SolARGeometricMatchesFilterOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IMatchesFilter {

public:
    ///@brief SolARGeometricMatchesFilterOpencv constructor.
        SolARGeometricMatchesFilterOpencv();
    ///@brief SolARGeometricMatchesFilterOpencv destructor.
       ~SolARGeometricMatchesFilterOpencv();
        /// @brief filter matches based fundamental matrix assumptions. This filter removes all outliers matches which give high reprojection error.
        /// @param[in] Original matches found between two descriptors "desc_1" and "desc_2".
        /// @param[out] Filtred matches based on geometric relations such as epipolar constraint.
        /// @param[in] Original keypoints associated to desc_1.
        /// @param[in] Original keypoints associated to desc_2.
        void filter(const std::vector<datastructure::DescriptorMatch> & inputMatches,
                    std::vector<datastructure::DescriptorMatch> & outputMatches,
                    const std::vector<datastructure::Keypoint> & inputKeyPointsA,
                    const std::vector<datastructure::Keypoint> & inputKeyPointsB) override;

		/// @brief filter matches based fundamental matrix calculated from camera matrices
		/// @param[in] Original matches found between two descriptors "desc_1" and "desc_2".
		/// @param[out] Filtred matches based on geometric relations such as epipolar constraint.
		/// @param[in] Original keypoints associated to desc_1.
		/// @param[in] Original keypoints associated to desc_2.
		/// @param[in] camera pose 1.
		/// @param[in] camera pose 2.
		/// @param[in] camera's intrinsic parameters.
		virtual void filter(const std::vector<datastructure::DescriptorMatch> & inputMatches,
							std::vector<datastructure::DescriptorMatch> & outputMatches,
							const std::vector<datastructure::Keypoint> & inputKeyPoints1,
							const std::vector<datastructure::Keypoint> & inputKeyPoints2,
							const datastructure::Transform3Df &pose1,
							const datastructure::Transform3Df &pose2,
							const datastructure::CamCalibration &intrinsicParams) override;

        void unloadComponent () override final;

     private:

        /// @brief The desirable level of confidence (propability) that the estimated matrix is correct.
        float m_confidence = 0.99f;

        ///  @brief threshold to define which point are ouliers
        ///  Here we are using a RANSAC method to keep only inliers matches.
        ///  This attribute is the ratio between the maximum distance in pixels between source points and the maximum distance in pixels to the epipolar line for which point is considered as a outlier.
        ///  The higher is this ratio, the more you will keep inliers to estimate your 2D transform, but the less this estimation will be correct.
        ///  By default, this value is set to the one proposed by [Snavely07 4.1]
        float m_outlierDistanceRatio = 0.006f;

		///  @brief threshold to valid matches based on distance to epilines
		float m_epilinesDistance = 10.f;
};

}
}
}

#endif // SOLARGEOMETRICMATCHESFILTEROPENCV_H
