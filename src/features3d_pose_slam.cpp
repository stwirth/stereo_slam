#include <ros/ros.h> // for logging
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/slam3d/edge_se3.h>

#include "stereo_slam/features3d.h"
#include "stereo_slam/pose_node.h"
#include "stereo_slam/features3d_pose_slam.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

stereo_slam::Features3DPoseSlam::Features3DPoseSlam() :
  next_vertex_id_(0)
{
//  g2o::BlockSolver_6_3::LinearSolverType* linearSolver = 
//      new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();

  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

  graph_optimizer_.setAlgorithm(solverGauss);
}


void stereo_slam::Features3DPoseSlam::update(
    const Features3D& features,
    const Eigen::Isometry3d& pose_estimate, const TimeStamp& stamp)
{
  PoseNode* node = new PoseNode();
  node->setFeatures(features);
  node->setEstimate(pose_estimate);
  node->setId(next_vertex_id_);
  if (next_vertex_id_ == 0)
  {
    node->setFixed(true);
  }
  ++next_vertex_id_;
  graph_optimizer_.addVertex(node);

  std::vector<PoseNode*> match_candidates =
    findMatchCandidates(node);
  ROS_INFO("Found %zu match candidates.", match_candidates.size());
  bool edge_added = false;
  for (size_t i = 0; i < match_candidates.size(); ++i)
  {
    PoseNode* candidate = match_candidates[i];
    Eigen::Isometry3d transform;
    int num_inliers;
    bool transform_found = 
      findTransform(node, 
          candidate,
          &transform, &num_inliers);
    if (transform_found)
    {
      ROS_INFO("Found transform to candidate %i with %i inliers. "
                "Adding edge.", i, num_inliers);
      // create new edge
      g2o::EdgeSE3* edge = new g2o::EdgeSE3();
      edge->setVertex(0, node);
      edge->setVertex(1, candidate);
      edge->setMeasurement(transform);
      graph_optimizer_.addEdge(edge);
      edge_added = true;
    }
    else
    {
      ROS_INFO("No transform found to candidate %i.", i);
    }
  }

  if (edge_added)
  {
    ROS_INFO("Optimizing global pose graph...");
    graph_optimizer_.initializeOptimization();
    graph_optimizer_.optimize(10);
    ROS_INFO("Optimizing done.");
    //updateMapTf();
  }
}

std::vector<stereo_slam::PoseNode*> stereo_slam::Features3DPoseSlam::findMatchCandidates(const PoseNode* node)
{
  std::vector<PoseNode*> candidates;
  const g2o::SparseOptimizer::VertexIDMap& vertex_map = 
    graph_optimizer_.vertices();
  g2o::SparseOptimizer::VertexIDMap::const_iterator iter;
  for (iter = vertex_map.begin(); iter != vertex_map.end(); ++iter)
  {
    PoseNode* candidate = dynamic_cast<PoseNode*>(iter->second);
    if (haveOverlap(node, candidate))
    {
      candidates.push_back(candidate);
    }
  }
  return candidates;
}

bool stereo_slam::Features3DPoseSlam::haveOverlap(
    const PoseNode* node1, const PoseNode* node2) const
{
  // TODO insert fancy frustum intersection here
  return true;
}

