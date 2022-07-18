#ifndef APPLICATIONS_GLOBAL_MAPPER_H_
#define APPLICATIONS_GLOBAL_MAPPER_H_

#include <memory>
#include <string>

#include <colmap/base/database.h>
#include <colmap/base/database_cache.h>
#include <colmap/base/reconstruction.h>
#include <colmap/optim/bundle_adjustment.h>
#include <colmap/sfm/incremental_triangulator.h>
#include <colmap/util/alignment.h>

#include <gopt/graph/view_graph.h>

namespace gopt {

class GlobalMapper {
 public:
  struct Options {
    int num_threads = -1;

    std::string image_path = "";

    // The projection will be performed for the given number of iterations (we
    // recommend > 40 iterations).
    int num_iterations = 48;
    
    // After orientations are estimated, view pairs may be filtered/removed if the
    // relative rotation of the view pair differs from the relative rotation
    // formed by the global orientation estimations. Adjust this threshold to
    // control the threshold at which rotations are filtered.
    double rotation_filtering_max_difference_degrees = 5.0;

    // The parameter translation_projection_tolerance determines which
    // translations are considered "bad" after analyzing their projections over
    // many iterations (it corresponds to tau in the paper).
    double translation_projection_tolerance = 0.08;

    // Any edges in the view graph with fewer than min_num_two_view_inliers will
    // be removed as an initial filtering step.
    int min_num_two_view_inliers = 30;

    // Thresholds for bogus camera parameters. Images with bogus camera
    // parameters are filtered and ignored in triangulation.
    double min_focal_length_ratio = 0.1;  // Opening angle of ~130deg
    double max_focal_length_ratio = 10;   // Opening angle of ~5deg
    double max_extra_param = 1;

    // Maximum reprojection error in pixels for observations.
    double filter_max_reproj_error = 4.0;

    // Minimum triangulation angle in degrees for stable 3D points.
    double filter_min_tri_angle = 1.5;

    // Which intrinsic parameters to optimize during the reconstruction.
    bool ba_refine_focal_length = true;
    bool ba_refine_principal_point = false;
    bool ba_refine_extra_params = true;

    // The minimum number of residuals per bundle adjustment problem to
    // enable multi-threading solving of the problems.
    int ba_min_num_residuals_for_multi_threading = 50000;

    // Ceres solver function tolerance for global bundle adjustment
    double ba_global_function_tolerance = 0.0;

    // The maximum number of global bundle adjustment iterations.
    int ba_global_max_num_iterations = 50;

    // The thresholds for iterative bundle adjustment refinements.
    int ba_global_max_refinements = 5;
    double ba_global_max_refinement_change = 0.0005;

    bool optimize_relative_translations = true;

    bool filter_relative_translations = true;

    bool final_global_bundle = true;

    colmap::IncrementalTriangulator::Options Triangulation() const;
    colmap::BundleAdjustmentOptions GlobalBundleAdjustment() const;
   
   private:
    colmap::IncrementalTriangulator::Options triangulation;
  };

  explicit GlobalMapper(const GlobalMapper::Options& options,
                        graph::ViewGraph* view_graph,
                        const colmap::DatabaseCache* database_cache);

  void BeginReconstruction(colmap::Reconstruction* reconstruction);

  void EndReconstruction(const bool discard);

  void Run(
      const RotationEstimatorOptions& rotation_estimator_options,
      const PositionEstimatorOptions& position_estimator_options);
  
  void Triangulation();

  // Complete tracks by transitively following the scene graph correspondences.
  // This is especially effective after bundle adjustment, since many cameras
  // and point locations might have improved. Completion of tracks enables
  // better subsequent registration of new images.
  size_t CompleteTracks(const colmap::IncrementalTriangulator::Options& tri_options);

  // Merge tracks by using scene graph correspondences. Similar to
  // `CompleteTracks`, this is effective after bundle adjustment and improves
  // the redundancy in subsequent bundle adjustments.
  size_t MergeTracks(const colmap::IncrementalTriangulator::Options& tri_options);

 private:
  bool FilterInitialViewGraph();
  void FilterRotations();
  void FilterRelativeTranslation();
  void OptimizePairwiseTranslations();

  size_t FilterPoints(const Options& options);

  size_t TriangulateImage(
    const colmap::IncrementalTriangulator::Options& tri_options,
    const image_t image_id);

  void RegisterImageEvent(const image_t image_id);
  void DeRegisterImageEvent(const image_t image_id);

  GlobalMapper::Options options_;

  // Class that holds all necessary data from database in memory.
  const colmap::DatabaseCache* database_cache_;

  // Class that holds data of the reconstruction.
  colmap::Reconstruction* reconstruction_;

  std::unordered_map<image_t, Eigen::Vector3d> global_rotations_;
  std::unordered_map<image_t, Eigen::Vector3d> global_positions_;

  graph::ViewGraph* view_graph_;

  // Class that is responsible for incremental triangulation.
  std::unique_ptr<colmap::IncrementalTriangulator> triangulator_;
  
  // The number of registered images per camera. This information is used
  // to avoid duplicate refinement of camera parameters and degradation of
  // already refined camera parameters in local bundle adjustment when multiple
  // images share intrinsics.
  std::unordered_map<camera_t, size_t> num_reg_images_per_camera_;
  
  // Number of images that are registered in at least on reconstruction.
  size_t num_total_reg_images_;

  // Number of shared images between current reconstruction and all other
  // previous reconstructions.
  size_t num_shared_reg_images_;

  // The number of reconstructions in which images are registered.
  std::unordered_map<image_t, size_t> num_registrations_;
};

}  // namespace gopt


#endif // APPLICATIONS_GLOBAL_MAPPER_H_
