#include "SolARMapFilterOpencv.h"
#include <iostream>
#include <utility>


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMapFilterOpencv)

namespace SolAR {
    using namespace datastructure;
        namespace MODULES {
            namespace OPENCV {

                SolARMapFilterOpencv::SolARMapFilterOpencv():ComponentBase(xpcf::toUUID<SolARMapFilterOpencv>())
                {
                    addInterface<IMapFilter>(this);
                #ifdef DEBUG
                    std::cout << " SolARMapFilterOpencv constructor" << std::endl;
                #endif
                }

				std::vector<cv::Point3d> SolARMapFilterOpencv::CloudPointsToPoints(const std::vector<SRef<CloudPoint>> cpts) {
					std::vector<cv::Point3d> out;
					for (unsigned int i = 0; i<cpts.size(); i++) {
						out.push_back(cv::Point3d(cpts[i]->getX(), cpts[i]->getY(), cpts[i]->getZ()));
					}
					return out;
				}


				bool SolARMapFilterOpencv::checkFrontCameraPoints(const std::vector<SRef<CloudPoint>>& pcloud, const Transform3Df & cameraPose, std::vector<bool> & isFrontCamera)
				{

					cv::Matx34d P(cameraPose(0, 0), cameraPose(0, 1), cameraPose(0, 2), cameraPose(0, 3),
						cameraPose(1, 0), cameraPose(1, 1), cameraPose(1, 2), cameraPose(1, 3),
						cameraPose(2, 0), cameraPose(2, 1), cameraPose(2, 2), cameraPose(2, 3));

					std::vector<cv::Point3d> pcloud_pt3d = CloudPointsToPoints(pcloud);
					std::vector<cv::Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

					cv::Matx44d P4x4 = cv::Matx44d::eye();
					for (int i = 0; i<12; i++) P4x4.val[i] = P.val[i];

					perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);

					isFrontCamera.resize(pcloud.size(), 0);
					for (int i = 0; i<pcloud.size(); i++) {
						isFrontCamera[i] = (pcloud_pt3d_projected[i].z > 0) ? true: false;
					}
					int count = cv::countNonZero(isFrontCamera);

					double percentage = ((double)count / (double)pcloud.size());
					if (percentage < 0.75)
						return false; //less than 75% of the points are in front of the camera
					if (false) //not
					{
						cv::Mat_<double> cldm(pcloud.size(), 3);
						for (unsigned int i = 0; i<pcloud.size(); i++) {
							cldm.row(i)(0) = pcloud[i]->getX();
							cldm.row(i)(1) = pcloud[i]->getY();
							cldm.row(i)(2) = pcloud[i]->getZ();
						}
						cv::Mat_<double> mean;
						cv::PCA pca(cldm, mean, CV_PCA_DATA_AS_ROW);

						int num_inliers = 0;
						cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
						cv::Vec3d x0 = pca.mean;
						double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

						for (int i = 0; i<pcloud.size(); i++) {
							cv::Vec3d w = cv::Vec3d(pcloud[i]->getX(), pcloud[i]->getY(), pcloud[i]->getZ()) - x0;
							double D = fabs(nrm.dot(w));
							if (D < p_to_plane_thresh) num_inliers++;
						}

						std::cout << num_inliers << "/" << pcloud.size() << " are coplanar" << std::endl;
						if ((double)num_inliers / (double)(pcloud.size()) > 0.85)
							return false;
					}
					return true;
				}

				void  SolARMapFilterOpencv::filterPointCloud(const std::vector<SRef<CloudPoint>>& input, const std::vector<bool> & isFrontCamera, std::vector<SRef<CloudPoint>>& output)
				{
					//filter out outlier points with high reprojection
                                        if (input.size() == 0)
                                        {
                                            LOG_INFO("FilterPointCLoud has an empty vector as input");
                                        }
					std::vector<double> reprj_errors;
					for (int i = 0; i<input.size(); i++) {
						reprj_errors.push_back(input[i]->getReprojError());
					}
					std::sort(reprj_errors.begin(), reprj_errors.end());
					//get the 80% precentile
					double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4; //threshold from Snavely07 4.2
					for (int i = 0; i<input.size(); i++) {
						if (!isFrontCamera[i])
							continue; //point was not in front of camera



						if (input[i]->getReprojError() > 16.0) {
							continue; //reject point
						}
						if (input[i]->getReprojError() < 4.0 ||
							input[i]->getReprojError() < reprj_err_cutoff)
						{
							output.push_back(input[i]);
						}
						else
						{
							continue;
						}
					}
					
				}

            }
        }
}
