#ifndef SOLAR2D3DCORRESPONDENCESFINDER_H
#define SOLAR2D3DCORRESPONDENCESFINDER_H
#include <vector>
#include "opencv2/core.hpp"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/storage/IPointCloudManager.h"
#include "SolAROpencvAPI.h"

#include "xpcf/component/ComponentBase.h"

namespace SolAR {
using namespace datastructure;
using namespace api::storage;
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

	/// @brief Define 2D-3D point correspondences of the current frame based on keypoint matches between the current frame and the last frame.
	/// @param[in] lastFrame: The temporally last frame to the current frame.
	/// @param[in] currentFrame: The current framr for which we want to find 2D-3D correspondances.
	/// @param[in] currentMatches: The 2D matches between the current keyframe and its reference keyframe.
	/// @param[out] shared_3dpoint: The 3D points visible from the current frame.
	/// @param[out] shared_2dpoint: The 2D point in the current frame that correspond to a 3D point.
	/// @param[out] found_matches: The matches between the current frame and its reference keyframe which have a 3 correspondant.
	/// @param[out] remaining_matches: The matches between the current frame and its reference keyframe for which no 3D points have been found.
	FrameworkReturnCode find(const SRef<Frame> lastFrame,
							const SRef<Frame> currentFrame,
							const std::vector<DescriptorMatch> & current_matches,
							std::vector<Point3Df> & shared_3dpoint,
							std::vector<Point2Df> & shared_2dpoint,
							std::vector<DescriptorMatch> & found_matches,
							std::vector<DescriptorMatch> & remaining_matches) override;

    void unloadComponent () override final;

private:
	SRef<IPointCloudManager> m_pointCloudManager;
};
}
}
}
#endif // SOLARPOSEESTIMATIONPNPOPENCV_H
