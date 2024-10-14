#include <teb_local_planner/fuzzy_weight_controller.h>
#include <numeric>

namespace teb_local_planner
{

FuzzyWeightController::FuzzyWeightController() : cfg_(NULL), initialized_(false), engine_(NULL), robot_model_(NULL)
{
}

FuzzyWeightController::FuzzyWeightController(TebConfig& cfg, RobotFootprintModelPtr robot_model)
{
  initialize(cfg, robot_model);
}

FuzzyWeightController::~FuzzyWeightController()
{
}

void FuzzyWeightController::initialize(TebConfig& cfg, RobotFootprintModelPtr robot_model)
{
  if (!initialized_)
  {
    // get pointer of teb config
    cfg_ = &cfg;

    // get robot model
    robot_model_ = robot_model;

    // initialize fuzzylite engine
    engine_ = new fuzzylite::Engine("TEB_Fuzzy_Controller");

    // Add input variables
    if (!inputFuzzyNarrowness(cfg_->robot.min_turning_radius, robotWheelbaseLength(robot_model_)) || !inputFuzzyTurningComplexity())
    {
      ROS_WARN("Fail to input fuzzy Narrowness or Turning Complexity");
    }

    outputObstacleWeight();

    outputSmoothWeight();

    outputLinearVelocityWeight();

    outputAngularVelocityWeight();

    outputShortestPathWeight();

    outputTimeOptimalWeight();

    createRuleBlock();

    // set initialized flag
    initialized_ = true;
  }
  else
  {
    ROS_WARN("FuzzyWeightController has already been initialized, skipping initialization.");
  }

  // ROS_INFO("Finish initialize fuzzy controller");
}

void FuzzyWeightController::startFuzzyWeights(double average_distance, double turn_complex)
{
  if (average_distance == -1 || turn_complex == -1)
  {
    ROS_WARN("narrowness or turn_complex is not available");
    return;
  }

  setInputN(average_distance);

  setInputTc(turn_complex);

  process();

  if (!updateWeights())
  {
    ROS_WARN("Fail to update weights from fuzzy controller");
  }
}

double FuzzyWeightController::robotWheelbaseLength(RobotFootprintModelPtr robot_model)
{
  // Types of robot models used
  // point
  if (boost::shared_ptr<PointRobotFootprint> point_model = boost::dynamic_pointer_cast<PointRobotFootprint>(robot_model))
  {
    return 0.0;
  }
  // circular
  if (boost::shared_ptr<CircularRobotFootprint> circular_model = boost::dynamic_pointer_cast<CircularRobotFootprint>(robot_model))
  {
    return circular_model->getInscribedRadius();
  }
  // line
  if (boost::shared_ptr<LineRobotFootprint> line_model = boost::dynamic_pointer_cast<LineRobotFootprint>(robot_model))
  {
    return 0.0;
  }
  // two circle
  if (boost::shared_ptr<TwoCirclesRobotFootprint> two_circles_model = boost::dynamic_pointer_cast<TwoCirclesRobotFootprint>(robot_model))
  {
    return two_circles_model->getInscribedRadius();
  }
  // polygon
  if (boost::shared_ptr<PolygonRobotFootprint> polygon_model = boost::dynamic_pointer_cast<PolygonRobotFootprint>(robot_model))
  {
    return polygon_model->getInscribedRadius();
  }

  // uknown model
  ROS_ERROR("Unknown robot footprint model.");
  return 0.0;
}

bool FuzzyWeightController::updateWeights()
{
  double weight_obs = reverseMapping(getOutputObs());

  double weight_sm = reverseMapping(getOutputSm());

  double weight_v = reverseMapping(getOutputV());

  double weight_w = reverseMapping(getOutputW());

  double weight_sp = getOutputSp();

  double weight_to = getOutputTo();

  ROS_INFO("Obstacle Weight: %f", weight_obs);
  ROS_INFO("Smoothness Weight: %f", weight_sm);
  ROS_INFO("Linear velocity Weight: %f", weight_v);
  ROS_INFO("Angular velocity Weight: %f", weight_w);
  ROS_INFO("Shortest Path Weight: %f", weight_sp);
  ROS_INFO("Time Optimal Weight: %f", weight_to);

  cfg_->optim.weight_obstacle = weight_obs;

  cfg_->optim.weight_smoothness = weight_sm;

  // non-holonomic robot
  cfg_->optim.weight_max_vel_x = weight_v / 5;
  cfg_->optim.weight_acc_lim_x = weight_v / 5;
  cfg_->optim.weight_jerk_lim_x = weight_v / 5;

  cfg_->optim.weight_max_vel_theta = weight_w;
  cfg_->optim.weight_acc_lim_theta = weight_w;
  cfg_->optim.weight_jerk_lim_theta = weight_w;
  
  if (cfg_->robot.max_vel_y > 0) // holonomic robot
  {
    cfg_->optim.weight_max_vel_y = weight_v;
    cfg_->optim.weight_acc_lim_y = weight_v;
    cfg_->optim.weight_jerk_lim_y = weight_v;
  }

  cfg_->optim.weight_shortest_path = weight_sp;

  cfg_->optim.weight_optimaltime = weight_to;
  return true;
}

void FuzzyWeightController::setInputN(double value)
{
  n_->setValue(value);
}

void FuzzyWeightController::setInputTc(double value)
{
  tc_->setValue(value);
}

double FuzzyWeightController::getOutputObs()
{
  return obs_->getValue();
}

double FuzzyWeightController::getOutputSm()
{
  return sm_->getValue();
}

double FuzzyWeightController::getOutputV()
{
  return v_->getValue();
}

double FuzzyWeightController::getOutputW()
{
  return w_->getValue();
}

double FuzzyWeightController::getOutputSp()
{
  return sp_->getValue();
}

double FuzzyWeightController::getOutputTo()
{
  return to_->getValue();
}

void FuzzyWeightController::process()
{
  engine_->process();
}

bool FuzzyWeightController::inputFuzzyNarrowness(double min_radius, double wheel_base)
{
  if ((wheel_base / 2) > min_radius)
  {
    ROS_INFO("The min_radius is small than robot length!");
    return false;
  }

  // Narrowness
  n_ = new fuzzylite::InputVariable;
  n_->setName("N");
  n_->setEnabled(true);
  n_->setRange(0.0, 4 * min_radius);
  n_->addTerm(new fuzzylite::Trapezoid("L", 0.0, 0.0, (min_radius + wheel_base / 2), (1.5 * min_radius + wheel_base / 4)));
  n_->addTerm(new fuzzylite::Triangle("M", (min_radius + wheel_base / 2), (1.5 * min_radius + wheel_base / 4), 2 * min_radius));
  n_->addTerm(new fuzzylite::Trapezoid("S", (1.5 * min_radius + wheel_base / 4), 2 * min_radius, 4 * min_radius, 4 * min_radius));
  engine_->addInputVariable(n_);

  return true;
}

bool FuzzyWeightController::inputFuzzyTurningComplexity()
{
  // Turning Complexity
  tc_ = new fuzzylite::InputVariable;
  tc_->setName("Tc");
  tc_->setEnabled(true);
  tc_->setRange(0.0, 2.0);
  tc_->addTerm(new fuzzylite::Trapezoid("S", 0.0, 0.0, 0.25, 0.5));
  tc_->addTerm(new fuzzylite::Triangle("M", 0.25, 0.5, 0.75));
  tc_->addTerm(new fuzzylite::Trapezoid("L", 0.5, 0.75, 2.0, 2.0));
  engine_->addInputVariable(tc_);

  return true;
}

void FuzzyWeightController::outputObstacleWeight()
{
  // Obstacle Weight

  obs_ = new fuzzylite::OutputVariable;
  obs_->setName("obs");
  obs_->setEnabled(true);
  obs_->setRange(-1.0, 3.0);
  obs_->setAggregation(new fuzzylite::Maximum);
  obs_->setDefuzzifier(new fuzzylite::Centroid(100));
  obs_->setDefaultValue(fuzzylite::nan);
  obs_->addTerm(new fuzzylite::Triangle("VL", -1.0, -1.0, 0.0));
  obs_->addTerm(new fuzzylite::Triangle("L", -1.0, 0.0, 1.0));
  obs_->addTerm(new fuzzylite::Triangle("M", 0.0, 1.0, 2.0));
  obs_->addTerm(new fuzzylite::Triangle("H", 1.0, 2.0, 3.0));
  obs_->addTerm(new fuzzylite::Triangle("VH", 2.0, 3.0, 3.0));

  obs_->setLockPreviousValue(false);

  engine_->addOutputVariable(obs_);
}

void FuzzyWeightController::outputSmoothWeight()
{
  // Smoothness Weight

  sm_ = new fuzzylite::OutputVariable;
  sm_->setName("sm");
  sm_->setEnabled(true);
  sm_->setRange(-1.0, 2.0);
  sm_->setAggregation(new fuzzylite::Maximum);
  sm_->setDefuzzifier(new fuzzylite::Centroid(100));
  sm_->setDefaultValue(fuzzylite::nan);
  sm_->addTerm(new fuzzylite::Trapezoid("VL", -1.0, -1.0, 0.0, 0.5));
  sm_->addTerm(new fuzzylite::Triangle("L", 0.0, 0.5, 1.0));
  sm_->addTerm(new fuzzylite::Triangle("M", 0.5, 1.0, 1.5));
  sm_->addTerm(new fuzzylite::Triangle("H", 1.0, 1.5, 2.0));
  sm_->addTerm(new fuzzylite::Triangle("VH", 1.5, 2.0, 2.0));

  sm_->setLockPreviousValue(false);

  engine_->addOutputVariable(sm_);
}

void FuzzyWeightController::outputLinearVelocityWeight()
{
  // Linear Velocity Weight

  v_ = new fuzzylite::OutputVariable;
  v_->setName("v");
  v_->setEnabled(true);
  v_->setRange(-1.0, 2.0);
  v_->setAggregation(new fuzzylite::Maximum);
  v_->setDefuzzifier(new fuzzylite::Centroid(100));
  v_->setDefaultValue(fuzzylite::nan);
  v_->addTerm(new fuzzylite::Trapezoid("VL", -1.0, -1.0, 0.0, 0.5));
  v_->addTerm(new fuzzylite::Triangle("L", 0.0, 0.5, 1.0));
  v_->addTerm(new fuzzylite::Triangle("M", 0.5, 1.0, 1.5));
  v_->addTerm(new fuzzylite::Triangle("H", 1.0, 1.5, 2.0));
  v_->addTerm(new fuzzylite::Triangle("VH", 1.5, 2.0, 2.0));

  v_->setLockPreviousValue(false);

  engine_->addOutputVariable(v_);
}

void FuzzyWeightController::outputAngularVelocityWeight()
{
  // Angular Velocity Weight

  w_ = new fuzzylite::OutputVariable;
  w_->setName("w");
  w_->setEnabled(true);
  w_->setRange(-1.0, 2.0);
  w_->setAggregation(new fuzzylite::Maximum);
  w_->setDefuzzifier(new fuzzylite::Centroid(100));
  w_->setDefaultValue(fuzzylite::nan);
  w_->addTerm(new fuzzylite::Trapezoid("VL", -1.0, -1.0, 0.0, 0.5));
  w_->addTerm(new fuzzylite::Triangle("L", 0.0, 0.5, 1.0));
  w_->addTerm(new fuzzylite::Triangle("M", 0.5, 1.0, 1.5));
  w_->addTerm(new fuzzylite::Triangle("H", 1.0, 1.5, 2.0));
  w_->addTerm(new fuzzylite::Triangle("VH", 1.5, 2.0, 2.0));

  w_->setLockPreviousValue(false);

  engine_->addOutputVariable(w_);
}

void FuzzyWeightController::outputShortestPathWeight()
{
  // Shortest Path Weight

  sp_ = new fuzzylite::OutputVariable;
  sp_->setName("sp");
  sp_->setEnabled(true);
  sp_->setRange(0.0, 5.0);
  sp_->setAggregation(new fuzzylite::Maximum);
  sp_->setDefuzzifier(new fuzzylite::Centroid(100));
  sp_->setDefaultValue(fuzzylite::nan);
  sp_->addTerm(new fuzzylite::Trapezoid("VL", 0.0, 0.0, 1.0, 2.0));
  sp_->addTerm(new fuzzylite::Triangle("L", 1.0, 2.0, 3.0));
  sp_->addTerm(new fuzzylite::Triangle("H", 2.0, 3.0, 4.0));
  sp_->addTerm(new fuzzylite::Trapezoid("VH", 3.0, 4.0, 5.0, 5.0));

  sp_->setLockPreviousValue(false);

  engine_->addOutputVariable(sp_);
}

void FuzzyWeightController::outputTimeOptimalWeight()
{
  // Time Optimal Weight

  to_ = new fuzzylite::OutputVariable;
  to_->setName("to");
  to_->setEnabled(true);
  to_->setRange(0.0, 5.0);
  to_->setAggregation(new fuzzylite::Maximum);
  to_->setDefuzzifier(new fuzzylite::Centroid(100));
  to_->setDefaultValue(fuzzylite::nan);
  to_->addTerm(new fuzzylite::Trapezoid("VL", 0.0, 0.0, 1.0, 2.0));
  to_->addTerm(new fuzzylite::Triangle("L", 1.0, 2.0, 3.0));
  to_->addTerm(new fuzzylite::Triangle("H", 2.0, 3.0, 4.0));
  to_->addTerm(new fuzzylite::Trapezoid("VH", 3.0, 4.0, 5.0, 5.0));

  to_->setLockPreviousValue(false);

  engine_->addOutputVariable(to_);
}

void FuzzyWeightController::createRuleBlock()
{
  // create rule block
  ruleblock_ = new fuzzylite::RuleBlock;
  ruleblock_->setName("RB");
  ruleblock_->setEnabled(true);
  ruleblock_->setConjunction(new fuzzylite::AlgebraicProduct);
  ruleblock_->setDisjunction(fuzzylite::null);
  ruleblock_->setImplication(new fuzzylite::AlgebraicProduct);
  ruleblock_->setActivation(new fuzzylite::General);

  // add rules
  addRules(ruleblock_);
  engine_->addRuleBlock(ruleblock_);
}

void FuzzyWeightController::addRules(fuzzylite::RuleBlock* ruleblock)
{
  // narrow S and turn complex S
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is S then obs is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is S then sm is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is S then v is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is S then w is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is S then sp is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is S then to is VH", engine_));

  // narrow S and turn complex ,
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is M then obs is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is M then sm is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is M then v is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is M then w is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is M then sp is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is M then to is H", engine_));

  // narrow S and turn complex L
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is L then obs is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is L then sm is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is L then v is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is L then w is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is L then sp is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is S and Tc is L then to is L", engine_));

  // narrow M and turn complex S
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is S then obs is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is S then sm is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is S then v is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is S then w is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is S then sp is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is S then to is H", engine_));

  // narrow M and turn complex M
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is M then obs is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is M then sm is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is M then v is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is M then w is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is M then sp is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is M then to is L", engine_));

  // narrow M and turn complex L
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is L then obs is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is L then sm is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is L then v is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is L then w is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is L then sp is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is M and Tc is L then to is L", engine_));

  // narrow L and turn complex S
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is S then obs is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is S then sm is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is S then v is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is S then w is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is S then sp is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is S then to is L", engine_));

  // narrow L and turn complex M
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is M then obs is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is M then sm is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is M then v is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is M then w is L", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is M then sp is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is M then to is L", engine_));

  // narrow L and turn complex L
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is L then obs is VH", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is L then sm is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is L then v is H", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is L then w is M", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is L then sp is VL", engine_));
  ruleblock->addRule(fuzzylite::Rule::parse("if N is L and Tc is L then to is L", engine_));
}

double FuzzyWeightController::reverseMapping(double value)
{
  if (value >= -1 && value < 0)
  {
    return -2 / (value - 1) - 1;
  }
  else if (value >= 0 && value <= 3)
  {
    return pow(10, value);
  }
  else
  {
    ROS_ERROR("Output is out of expected range: %f", value);
    return -1;
  }
}

FuzzySet FuzzyWeightController::determineFuzzySetForN(double value, double min_radius, double wheel_base) 
{
  if (value < 0.0 || value > 4 * min_radius) {
    std::cerr << "Error: The input value is out of the expected range." << std::endl;
    return L; // Default to returning S, or you can choose to throw an exception or other error handling methods
  }

  // Calculate the boundary points for each set
  double s_end = (min_radius + wheel_base / 2);
  double m_start = s_end;
  double m_peak = (1.5 * min_radius + wheel_base / 4);
  double m_end = 2 * min_radius;
  double l_start = m_peak;
  double l_end = 4 * min_radius;

  // Determine which set the value belongs to
  if (value <= s_end) {
    return S; // Belongs to the S set
  } else if (value >= m_start && value <= m_end) {
    return M; // Belongs to the M set
  } else if (value >= l_start && value <= l_end) {
    return L; // Belongs to the L set
  } else {
    // If the value is exactly at the boundary between two sets, one of the sets can be chosen here
    // or handle it according to the actual situation
    if (value == s_end) {
      return S; // Can choose S or M
    } else if (value == m_end) {
      return M; // Can choose M or L
    }
    // This should not happen, as the range of value has already been checked
    std::cerr << "Unexpected error in determineFuzzySet function." << std::endl;
    return L; // Default to returning S
  }
}

FuzzySet FuzzyWeightController::determineFuzzySetForTC(double value)
{
  if (value < 0.0 || value > 2.0) {
    std::cerr << "Error: The input value is out of the expected range [0.0, 2.0]." << std::endl;
    return S; // Default to returning S, or you can choose to throw an exception or other error handling methods
  }

  // Calculate the boundary points for each set
  double s_end = 0.5;
  double m_start = 0.25;
  double m_peak = 0.5;
  double m_end = 0.75;
  double l_start = 0.5;
  double l_end = 2.0;

  // Determine which set the value belongs to
  if (value <= s_end) {
    return S; // Belongs to the S set
  } else if (value >= m_start && value <= m_end) {
    return M; // Belongs to the M set
  } else if (value >= l_start && value <= l_end) {
    return L; // Belongs to the L set
  } else {
    // If the value is exactly at the boundary between two sets, one of the sets can be chosen here
    // or handle it according to the actual situation
    if (value == s_end) {
      return S; // Can choose S or M
    } else if (value == m_end) {
      return M; // Can choose M or L
    }
    // This should not happen, as the range of value has already been checked
    std::cerr << "Unexpected error in determineFuzzySetForTC function." << std::endl;
    return S; // Default to returning S
  }
}

} // namespace teb_local_planner
