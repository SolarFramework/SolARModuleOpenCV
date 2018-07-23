#ifndef SOLARMAPPEROPENCV_H
#define SOLARMAPPEROPENCV_H


#include "api/solver/map/IMapper.h"
#include "xpcf/component/ComponentBase.h"
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
            class SOLAROPENCV_EXPORT_API SolARMapperOpencv : public org::bcom::xpcf::ComponentBase,
                public api::solver::map::IMapper {
            public:
                SolARMapperOpencv();

                ~SolARMapperOpencv() = default;

				void addNewKeyFrame(const SRef<Keyframe> & kFrame) ;

                void removeKeyframe(const SRef<Keyframe>&new_frame);

                void addMatches(const std::pair<int,int>&working_views,
                                const std::vector<DescriptorMatch>&new_matches);

                // this method should be called with timestamp
                SRef<Keyframe> getCurrentKeyframe(int idx);

                void associateReferenceKeyFrameToFrame(SRef<Frame> frame)  ;

                SRef<Map> getMap() ;

                int isKeyFrameCandidate(SRef<Frame> frame) ;

                bool initMap(SRef<Keyframe>&kframe_t0,
                             SRef<Keyframe>&kframe_t1,
                             std::vector<SRef<CloudPoint>>&new_cloud,
                             std::vector<DescriptorMatch>&initMatches);



                void unloadComponent () override final;


               // for the moment put this in public
                std::vector<SRef<Keyframe>>m_kframes;
                std::map<std::pair<int, int>, std::vector<DescriptorMatch> > m_gmatches;
                SRef<Map> m_map;

            protected :

            bool updateMap(const SRef<Keyframe>&new_kframe,
                           const std::vector<DescriptorMatch>&new_matches);


            private:
            };
        }
    }
}



#endif // MAPPEROPENCV_H
