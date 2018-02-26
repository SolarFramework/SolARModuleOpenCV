#include "SolARBasicMatchesFilterOpencv.h"
#include "ComponentFactory.h"
#include "SolAROpenCVHelper.h"
#include <set>


using namespace org::bcom::xpcf;
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARBasicMatchesFilterOpencv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARBasicMatchesFilterOpencv::SolARBasicMatchesFilterOpencv()
{
    setUUID(SolARBasicMatchesFilterOpencv::UUID);
    addInterface<api::features::IMatchesFilter>(this,api::features::IMatchesFilter::UUID, "interface BasicMatchesilterOpencv");
}


SolARBasicMatchesFilterOpencv::~SolARBasicMatchesFilterOpencv(){

}



void SolARBasicMatchesFilterOpencv::filter(const std::vector<DescriptorMatch>&inputMatches,
                                           std::vector<DescriptorMatch>&outputMatches,
                                           const std::vector<SRef<Keypoint>>&inputKeyPoints,
                                           std::vector<SRef<Keypoint>>&outputKeyPoints){
           std::set<int> existing_trainIdx;
               for (unsigned int i = 0; i < inputMatches.size(); i++){
                   //"normalize" matching: somtimes imgIdx is the one holding the trainIdx
                   if (existing_trainIdx.find((inputMatches)[i].getIndexInDescriptorB()) == existing_trainIdx.end() &&
                       (inputMatches)[i].getIndexInDescriptorB() >= 0 && (inputMatches)[i].getIndexInDescriptorB() < inputKeyPoints.size()){
                      outputMatches.push_back((inputMatches)[i]);
                       existing_trainIdx.insert((inputMatches)[i].getIndexInDescriptorB());
                   }
               }
};

}
}
}
