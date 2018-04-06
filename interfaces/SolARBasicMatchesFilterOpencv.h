#ifndef SOLARBASICMATCHESFILTEROPENCV_H
#define SOLARBASICMATCHESFILTEROPENCV_H

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

class SOLAROPENCV_EXPORT_API SolARBasicMatchesFilterOpencv : public org::bcom::xpcf::ComponentBase,
        public api::features::IMatchesFilter {
public:

   SolARBasicMatchesFilterOpencv();
   ~SolARBasicMatchesFilterOpencv();

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
