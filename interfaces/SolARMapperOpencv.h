#ifndef SOLARMAPPEROPENCV_H
#define SOLARMAPPEROPENCV_H


#include "api/solver/map/IMapper.h"
#include "xpcf/component/ConfigurableBase.h"
#include <vector>
#include "SolAROpencvAPI.h"
#include "SolAROpenCVHelper.h"

#include <string>

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
        /**
         * @class SolARSVDTriangulationOpencv
         * @brief Triangulates set of corresponding 2D-2D points correspondances with known respective camera poses based on opencv SVD.
         */
            class SOLAROPENCV_EXPORT_API SolARMapperOpencv : public org::bcom::xpcf::ConfigurableBase,
                public api::solver::map::IMapper {
            public:
                SolARMapperOpencv();

                ~SolARMapperOpencv() = default;

                void addNewKeyFrame(const SRef<Frame> & frame, SRef<Keyframe>& newKeyframe) override;

                void removeKeyframe(const SRef<Keyframe>&new_frame);

                // this method should be called with timestamp
                SRef<Keyframe> getCurrentKeyframe(int idx);

                void associateReferenceKeyFrameToFrame(SRef<Frame> frame)  ;

                SRef<Map> getMap() ;

                bool isKeyFrameCandidate(const std::vector<SRef<Keypoint>>& KeypointsRef, const std::vector<SRef<Keypoint>>& keypointsCurrent, const std::vector<DescriptorMatch>& matches, const unsigned int imageWidth) ;

                bool initMap(SRef<Keyframe>&kframe_t0,
                             SRef<Keyframe>&kframe_t1,
                             std::vector<SRef<CloudPoint>>&new_cloud,
                             std::vector<DescriptorMatch>&initMatches);

                bool updateMap(const SRef<Keyframe>&new_kframe,
                               const std::vector<DescriptorMatch>& found_matches,
                               const std::vector<DescriptorMatch>&new_matches,
                               const std::vector<SRef<CloudPoint>>& newCloud) override;

                void unloadComponent () override final;

            private:

               // for the moment put this in public
                std::vector<SRef<Keyframe>>m_kframes;
                std::map<std::pair<int, int>, std::vector<DescriptorMatch> > m_gmatches;
                SRef<Map> m_map;

                // Minimum number of matches for a frame to be a keyframe
                int m_minNbMatchesIsKeyframe = 50;

                // Minimum mean distance for a frame to be a keyframe
                float m_minMeanDistanceIsKeyframe = 20.0;

                void addMatches(const std::pair<int,int>&working_views,
                                const std::vector<DescriptorMatch>& found_matches,
                                const std::vector<DescriptorMatch>&new_matches);
            };
        }
    }
}



#endif // MAPPEROPENCV_H
