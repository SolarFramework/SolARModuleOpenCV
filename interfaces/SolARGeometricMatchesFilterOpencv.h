#ifndef SOLARGEOMETRICMATCHESFILTEROPENCV_H
#define SOLARGEOMETRICMATCHESFILTEROPENCV_H

#include "api/features/IMatchesFilter.h"
#include "ComponentBase.h"
#include "SolAROpencvAPI.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
            /**
             * @class SolARGeometricMatchesFilterOpencv
             * @brief Filters matches based on geometric assumptions.
             */
            class SOLAROPENCV_EXPORT_API SolARGeometricMatchesFilterOpencv : public org::bcom::xpcf::ComponentBase,
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
                void filter(const std::vector<DescriptorMatch>&inputMatches,
                            std::vector<DescriptorMatch>&outputMatches,
                            const std::vector<SRef<Keypoint>>&inputKeyPointsA,
                            const std::vector<SRef<Keypoint>>&inputKeyPointsB);
                void unloadComponent () override final;

                    XPCF_DECLARE_UUID("3731691e-2c4c-4d37-a2ce-06d1918f8d41");

             private:

            };
        }
    }
}




#endif // SOLARMATCHESFILTEROPENCV_H
