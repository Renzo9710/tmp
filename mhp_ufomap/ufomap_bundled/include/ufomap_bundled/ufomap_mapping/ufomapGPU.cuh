#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda/std/array>
#include <stdio.h>
#include <stdlib.h>
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <Eigen/Geometry>
#include <chrono>
#include <map>
#include <tuple>

namespace ufomapGPU
{

   class UFOMapGPU
   {
      // Node definitions from Ufomap source
      using INNER_NODE = ufo::map::OccupancyMapInnerNode<ufo::map::ColorOccupancyNode<float>>;
      using LEAF_NODE = ufo::map::OccupancyMapLeafNode<ufo::map::ColorOccupancyNode<float>>;
      using DepthType = unsigned int;

   public:
      UFOMapGPU() {};
      ~UFOMapGPU() {};

      void copyMapCPUtoGPU(ufo::map::OccupancyMapColor *map_host);
      void clearMapGPU();
      void calcInfoGain(int num, int threads, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points,
                        std::map<int, std::vector<ufo::map::Point3>> *endpoints, double max_range, Eigen::MatrixXd *results,
                        Eigen::VectorXd *time_metrics);
      void calcInfoAndOccGain(int num,int num_occ, int num_pred_occ,int threads, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points_occ,
                              std::map<int, std::vector<ufo::map::Point3>> *endpoints, std::map<int, std::vector<ufo::map::Point3>> *endpoints_occ, double max_range, Eigen::MatrixXd *results, Eigen::MatrixXd *results_occ,
                              Eigen::VectorXd *time_metrics, Eigen::VectorXd *time_metrics_occ);
      void calcOccGain(int num_occ,int num_pred_occ, int threads, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points_occ,
         std::map<int, std::vector<ufo::map::Point3>> *endpoints_occ, double max_range, Eigen::MatrixXd *results_occ,
         Eigen::VectorXd *time_metrics_occ);

   private:
      ufo::map::OccupancyMapColor *_h_map = nullptr;
      ufo::map::OccupancyMapColor *_d_map = nullptr;

      std::map<void *, void *> _h_children_map;

      int _depth_max = 16;
      // Helper functions
      void getChildrenPointer(INNER_NODE *root, ufo::map::OccupancyMapColor *map_h, int depth_cnt = 0, int depth_max = 16);
      void setPointerBackToHost(INNER_NODE *root, ufo::map::OccupancyMapColor *map_h, int depth_cnt = 0, int depth_max = 16);
      void updateMapGPU();
   };
} // namespace ufomapGPU