#ifndef SOLARBASICMATCHESFILTEROPENCV_H
#define SOLARBASICMATCHESFILTEROPENCV_H

#include "api/features/IMatchesFilter.h"
#include "ComponentBase.h"
#include "SolAROpencvAPI.h"

#include <vector>

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
        /**
         * @class SolARBasicMatchesFilterOpencv
         * @brief Filters matches based on basic strategies (redundancy, match score..).
         */
            class SOLAROPENCV_EXPORT_API SolARBasicMatchesFilterOpencv : public org::bcom::xpcf::ComponentBase,
                    public api::features::IMatchesFilter {
            public:
               ///@brief SolARBasicMatchesFilterOpencv constructor.
               SolARBasicMatchesFilterOpencv();
               ///@brief SolARBasicMatchesFilterOpencv destructor.
               ~SolARBasicMatchesFilterOpencv();

               /// @brief filter matches based on redundancy strategy. This filter removes all the mmulitples matches.
               /// @param[in] Original matches found between two descriptors "desc_1" and "desc_2".
               /// @param[out] Filtred matches based on redanduncy or geometric relations such as epipolar constraint.
               /// @param[in] Original keypoints associated to desc_1.
               /// @param[in] Original keypoints associated to desc_2.
               void filter(const std::vector<DescriptorMatch>&inputMatches,
                           std::vector<DescriptorMatch>&outputMatches,
                           const std::vector<SRef<Keypoint>>&inputKeyPointsA,
                           const std::vector<SRef<Keypoint>>&inputKeyPointsB);

                void unloadComponent () override final;

                    XPCF_DECLARE_UUID("cbb620c3-a7fc-42d7-bcbf-f59b475b23b0");

             private:

            };

}
}
}




#endif // SOLARMATCHESFILTEROPENCV_H
