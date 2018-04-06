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

class SOLAROPENCV_EXPORT_API SolARGeometricMatchesFilterOpencv : public org::bcom::xpcf::ComponentBase,
        public api::features::IMatchesFilter {
public:

    SolARGeometricMatchesFilterOpencv();
   ~SolARGeometricMatchesFilterOpencv();

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
