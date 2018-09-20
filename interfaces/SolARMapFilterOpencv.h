#ifndef SOLARMAPFILTEROPENCV_H
#define SOLARMAPFILTEROPENCV_H


#include "api/solver/map/IMapFilter.h"
#include "xpcf/component/ConfigurableBase.h"
#include <vector>
#include "SolAROpencvAPI.h"

#include <string>

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
        /**
         * @class SolARMapFilterOpencv
         * @brief Filter a cloud of 3D points by removing points with a too important reporjection error or those which are behind the camera.
         * @brief The projection error threshold as well as the test of cheirality (removing points behind the camera) can be configured.
         */
            class SOLAROPENCV_EXPORT_API SolARMapFilterOpencv : public org::bcom::xpcf::ConfigurableBase,
                public api::solver::map::IMapFilter {
            public:
                SolARMapFilterOpencv();

                ~SolARMapFilterOpencv() = default;

                /// @brief  Filter point cloud reconstructed from 2 viewpoints
                /// @param[in] view1: the first keyframe used for building the point cloud.
                /// @param[in] view2: the second keyframe used for building the point cloud.
                /// @param[in] input: The set of points to filter
                /// @param[out] output: the filtered point cloud
                void  filter(const SRef<Keyframe> view1, const SRef<Keyframe> view2, const std::vector<SRef<CloudPoint>>& input,  std::vector<SRef<CloudPoint>>& output) override;

                void unloadComponent () override final;

            protected :

            private:
                //@brief maximum reprojection error to keep the triangulated 3D point
                float m_reprojErrorThreshold = 0.5f;

                //@brief if not null, the point reconstructed behind the camera are removed
                int m_cheiralityCheck = 1;


            };
        }
    }
}

#endif // MAPPEROPENCV_H
