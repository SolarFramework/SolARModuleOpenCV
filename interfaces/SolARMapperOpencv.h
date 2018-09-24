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

    SRef<Map> getMap() ;

    virtual FrameworkReturnCode SolARMapperOpencv::update (SRef<Map> map,
                                                           SRef<Keyframe> newKeyframe,
                                                           const std::vector<SRef<CloudPoint>>& newCloud = {},
                                                           const std::vector<DescriptorMatch>& newPointsMatches = {},
                                                           const std::vector<DescriptorMatch>& existingPointsMatches = {}) override;

    void unloadComponent () override final;

private:

    std::vector<SRef<Keyframe>>m_kframes;
    std::map<std::pair<int, int>, std::vector<DescriptorMatch> > m_gmatches;
    SRef<Map> m_map;

    void addMatches(const std::pair<int,int>&working_views,
                    const std::vector<DescriptorMatch>& found_matches,
                    const std::vector<DescriptorMatch>&new_matches);
};
}
}
}



#endif // MAPPEROPENCV_H
