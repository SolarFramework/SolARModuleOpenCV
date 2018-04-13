#include "SolARBasicMatchesFilterOpencv.h"
#include "ComponentFactory.h"
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


bool sortMatchByDistance(const std::pair<int,float> &lhs, const std::pair<int,float> &rhs)
{
    return lhs.second < rhs.second;
}

// filter matches : keep the best match in case of multiple matches per keypoint
void SolARBasicMatchesFilterOpencv::filter(const std::vector<DescriptorMatch>&inputMatches,
                                            std::vector<DescriptorMatch>&outputMatches,
                                            const std::vector<SRef<Keypoint>>&inputKeyPointsA,
                                            const std::vector<SRef<Keypoint>>&inputKeyPointsB){


    std::vector<DescriptorMatch>matches;
    std::map<int,std::vector<std::pair<int,float>>> matchesMap;

    for(auto itr:inputMatches){
        matchesMap[itr.getIndexInDescriptorA()].push_back(std::make_pair(itr.getIndexInDescriptorB(),itr.getMatchingScore()));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){
            std::sort(ptr.begin(),ptr.end(),sortMatchByDistance);
        }
        matches.push_back(DescriptorMatch(itr->first, ptr.begin()->first,ptr.begin()->second));
    }


    matchesMap.clear();
    for(auto itr:matches){
        matchesMap[itr.getIndexInDescriptorB()].push_back(std::make_pair(itr.getIndexInDescriptorA(),itr.getMatchingScore()));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){
            std::sort(ptr.begin(),ptr.end(),sortMatchByDistance);
        }
        matches.push_back(DescriptorMatch(ptr.begin()->first,itr->first,ptr.begin()->second));
    }


    outputMatches=matches;


 }

}
}
}
