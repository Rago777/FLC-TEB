#ifndef _EDGE_SMOOTHNESS_H_
#define _EDGE_SMOOTHNESS_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/teb_config.h>


namespace teb_local_planner
{

/**
 * @class EdgeSmoothness
 * @brief Edge defining the cost function for minimizing the smoothness
*/
class EdgeSmoothness : public BaseTebBinaryEdge<1 ,double, VertexPose, VertexPose>
{
public:
  /**
   * @brief Construct edge.
   */
  EdgeSmoothness()
  {
    this->setMeasurement(0.);
  }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeSmoothness()");
    const VertexPose *pose1 = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *pose2 = static_cast<const VertexPose *>(_vertices[1]);

    // Calculate change value of angle
    _error[0] = pow(pose2->theta() - pose1->theta(), 2);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSmoothness::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace


#endif /* EDGE_SMOOTHNESS_H_ */