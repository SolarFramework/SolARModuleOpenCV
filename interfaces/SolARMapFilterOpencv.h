#ifndef SOLARMAPFILTEROPENCV_H
#define SOLARMAPFILTEROPENCV_H


#include "api/solver/map/IMapFilter.h"
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
            class SOLAROPENCV_EXPORT_API SolARMapFilterOpencv : public org::bcom::xpcf::ComponentBase,
                public api::solver::map::IMapFilter {
            public:
                SolARMapFilterOpencv();

                ~SolARMapFilterOpencv() = default;


				/// @brief Check triangulation status.Warn negative-Z triangulated points.
				/// @param[in] Set of triangulated 3d_points.
				/// @param[in] Camera pose of the second view at triangulation step (the first one supposed canonical).
				/// @param[out] Status of each reprojected =3d_point (false: negative-z, true: non-negative z).
				/// @return validity of the triangulated points
				bool checkFrontCameraPoints(const std::vector<SRef<CloudPoint>>& pcloud, const Transform3Df & cameraPose, std::vector<bool> & isFrontCamera) ;

				/// @brief  Filter point cloud according to reprojection error and front camera status
				/// @param[in] Set of triangulated 3d_points.
				/// @param[in] Status of each reprojected =3d_point (false: negative-z, true: non-negative z).
				/// @param[out] filtered point cloud without z negative points and points with a large reprojection error
				void  filterPointCloud(const std::vector<SRef<CloudPoint>>& input, const std::vector<bool> & isFrontCamera, std::vector<SRef<CloudPoint>>& output) ;


                void unloadComponent () override final;

            protected :

            private:
				/// @brief Convert  the point cloud to opencv structure for CV processing.
				/// @param[in] Set of triangulated 3d_points.
				/// @return Set of triangulated 3d_points expressed with opencv data structure.
				std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<SRef<CloudPoint>> cpts);

            };
        }
    }
}



#endif // MAPPEROPENCV_H
