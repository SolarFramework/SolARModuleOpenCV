#ifndef SOLAR2D3DCORRESPONDENCESFINDER_H
#define SOLAR2D3DCORRESPONDENCESFINDER_H
#include <vector>
#include "opencv2/core.hpp"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "SolAROpencvAPI.h"

#include "xpcf/component/ComponentBase.h"

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
        /**
         * @class SolAR2D3DCorrespondencesFinderOpencv
         * @brief <B>Finds the 3D correspondents of 2D keypoints.</B>
         * <TT>UUID: cedd8c47-e7b0-47bf-abb1-7fb54d198117</TT>
         *
         */
            class SOLAROPENCV_EXPORT_API SolAR2D3DCorrespondencesFinderOpencv : public org::bcom::xpcf::ComponentBase,
                public api::solver::pose::I2D3DCorrespondencesFinder
            {
            public:
                ///@brief SolARPoseEstimationPnpOpencv constructor;
                SolAR2D3DCorrespondencesFinderOpencv();
                ///@brief SolARPoseEstimationPnpOpencv destructor;
                ~SolAR2D3DCorrespondencesFinderOpencv()  override = default;
                /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D  world points. The estimation is based on opencv Perspective from N Points algorithm
                /// @param[in] Set of 2d_points seen in view_1.
                /// @param[in] Set of 3d_points corresponding to view_1.
                /// @param[out] Camera pose in the world coordinates system of the view_1.
                FrameworkReturnCode find(const SRef<Keyframe> referenceKeyframe,
                                         const SRef<Frame> currentFrame,
                                         const std::vector<DescriptorMatch> & current_matches,
										 const SRef<IPointCloudManager>& worldMap,
                                         std::vector<Point3Df> & shared_3dpoint,
                                         std::vector<Point2Df> & shared_2dpoint,
                                         std::vector<DescriptorMatch> & found_matches,
                                         std::vector<DescriptorMatch> & remaining_matches) override;
                /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D  world points. The estimation is based on opencv Perspective from N Points algorithm
                /// @param[in] Set of 2d_points seen in view_1.
                /// @param[in] Set of 3d_points corresponding to view_1.
                /// @param[out] Camera pose in the world coordinates system of the view_1.
				FrameworkReturnCode find(	const SRef<Frame> lastFrame,
											const SRef<Frame> currentFrame,
                                            const std::vector<DescriptorMatch> & current_matches,
											const SRef<IPointCloudManager>& worldMap,
                                            std::vector<Point3Df> & shared_3dpoint,
                                            std::vector<Point2Df> & shared_2dpoint,
											std::vector<DescriptorMatch> & found_matches,
											std::vector<DescriptorMatch> & remaining_matches) override;

                void unloadComponent () override final;
            private:
            };
        }
    }
}
#endif // SOLARPOSEESTIMATIONPNPOPENCV_H
