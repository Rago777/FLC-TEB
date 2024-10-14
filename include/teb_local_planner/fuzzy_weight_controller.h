#ifndef FUZZY_WEIGHT_CONTROLLER_H_
#define FUZZY_WEIGHT_CONTROLLER_H_


#include <ros/ros.h>

#include <fuzzylite/Headers.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>

// timed-elastic-band related classes
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/timed_elastic_band.h>
#include <geometry_msgs/PoseStamped.h>

// dynamic reconfigure
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>


namespace teb_local_planner
{
  
enum FuzzySet {
  S, // Small
  M, // Medium
  L  // Large
};

class FuzzyWeightController
{
public:
  /**
   * @brief Default constructor
   */
  FuzzyWeightController();

  /**
   * @brief Constructor and initialize the FuzzyWeightController class
   * @param cfg Reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param teb Shared pointer to the teb-trajectory container
   * @param robot_model Shared pointer to the robot shape model used for optimization
   */
  FuzzyWeightController(TebConfig& cfg, RobotFootprintModelPtr robot_model);

  /**
   * @brief Default destructor
   */
  ~FuzzyWeightController();

  /**
   * @brief Initialize the FuzzyWeightController class
   * @param cfg Reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param teb Shared pointer to the teb-trajectory container
   * @param robot_model Shared pointer to the robot shape model used for optimization
   */
  void initialize(TebConfig& cfg, RobotFootprintModelPtr robot_model);

  /**
   * @brief Start adjust weights based on narrowness and turning complexity
   */
  void startFuzzyWeights(double average_distance, double turn_complex);

  /**
   * @brief Obtain the minimum turning radius of a car-like mobile robot
   * @param robot_model Shared pointer to the robot shape model used for optimization
   */
  double robotWheelbaseLength(RobotFootprintModelPtr robot_model);

  /**
   * @brief Update weights
   */
  bool updateWeights();

  /**
   * @brief set narrowness input
   * @param value narrowness value
   */
  void setInputN(double value);

  /**
   * @brief set turning complexity input
   * @param value turning complexity value
   */
  void setInputTc(double value);

  /**
   * @brief get obs weight value
   * @return obs weight
   */
  double getOutputObs();

  /**
   * @brief get velocity weight value
   * @return velocity and acceleration weight
   */
  double getOutputV();

  /**
   * @brief get jerk weight value( The jerk weight make a serious impact to trajectory, should not be too large)
   * @return jerk weight
   */
  double getOutputW();

  /**
   * @brief get smooth weight value
   * @return smooth weight
   */
  double getOutputSm();
  
  /**
   * @brief get shortest path weight value
   * @return shortest path weight
   */
  double getOutputSp();
  
  /**
   * @brief get time optimal weight value
   * @return time optima weight
   */
  double getOutputTo();

  /**
   * @brief start fuzzylite process
   */
  void process();

  /**
   * @brief Define input variables narrowness
   */
  bool inputFuzzyNarrowness(double min_radius, double wheel_base);

  /**
   * @brief Define input variables turning complexity
   */
  bool inputFuzzyTurningComplexity();

  /**
   * @brief Define output variables obstacle distance
   */
  void outputObstacleWeight();

  /**
   * @brief Define output variables smoothness
   */
  void outputSmoothWeight();

  /**
   * @brief Define output variables linear Velocity Weight
   */
  void outputLinearVelocityWeight();

  /**
   * @brief Define output variables angular Velocity Weight
   */
  void outputAngularVelocityWeight();

  /**
   * @brief Define output variables shortest path
   */
  void outputShortestPathWeight();

  /**
   * @brief Define output variables time optimal
   */
  void outputTimeOptimalWeight();

  /**
   * @brief Create fuzzy rule block
   */
  void createRuleBlock();

  /**
   * @brief add fuzzy rules
   * @param ruleblock pointer of fuzzy rule block
   */
  void addRules(fuzzylite::RuleBlock* ruleblock);

  /**
   * @brief reverse the output value to tebconfig's weights
   * @param value the output value from fuzzy controller
   */
  double reverseMapping(double value);

  FuzzySet determineFuzzySetForN(double value, double min_radius, double wheel_base);
  FuzzySet determineFuzzySetForTC(double value);

public:
  TebConfig* cfg_;  //!< Config class that stores and manages all related parameters
  // TimedElasticBandPtr teb_; //!< pointer to teb trajectory
  RobotFootprintModelPtr robot_model_;//!< pointer to robot model
  int n_turn_; //!< num of turn segment
  int n_sharp_; //!< num of sharp turn segement
  fuzzylite::Engine *engine_;   //!< Fuzzy controller engine
  fuzzylite::InputVariable *n_; //!< Narrowness input
  fuzzylite::InputVariable *tc_; //!< Turning Complexity input
  fuzzylite::OutputVariable *obs_; //!< Obstacle weight
  fuzzylite::OutputVariable *sm_; //!< Smooth weight
  fuzzylite::OutputVariable *v_; //!< Velocity weight
  fuzzylite::OutputVariable *w_; //!< Jerk weight
  fuzzylite::OutputVariable *sp_; //!< Shortest path weight
  fuzzylite::OutputVariable *to_; //!< time optimal weight
  fuzzylite::RuleBlock *ruleblock_;
  bool initialized_;
};

} // namespace teb_local_planner


#endif