#include <vtkMRMLROS2RobotNode.h>

#include <vtkEventBroker.h>
#include <vtkTransform.h>
#include <vtkPointSet.h>
#include <vtkSTLReader.h>
#include <vtkTransformFilter.h>

#include <vtkMRMLScene.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLModelNode.h>
#include <vtkMRMLModelDisplayNode.h>

#include <vtkMRMLROS2Utils.h>
#include <vtkMRMLROS2NodeNode.h>
#include "vtkMRMLROS2NodeInternals.h"
#include <vtkMRMLROS2ParameterNode.h>
#include <vtkMRMLROS2Tf2LookupNode.h>

#include <regex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vtkMRMLROS2NodeInternals.h>
#include <eigen3/Eigen/Geometry>
#include <sstream>

// MoveIt kinematics includes (commented out for faster build)
// #include <moveit/robot_model_loader/robot_model_loader.h>

// KDL includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

auto const MM_TO_M_CONVERSION = 1000.00;

vtkStandardNewMacro(vtkMRMLROS2RobotNode);

vtkMRMLNode * vtkMRMLROS2RobotNode::CreateNodeInstance(void)
{
  return SelfType::New();
}


const char * vtkMRMLROS2RobotNode::GetNodeTagName(void)
{
  return "ROS2RobotNode";
}


vtkMRMLROS2RobotNode::vtkMRMLROS2RobotNode()
{
  mInternals = std::make_unique<vtkMRMLROS2RobotNodeInternals>();
}


vtkMRMLROS2RobotNode::~vtkMRMLROS2RobotNode()
{
}


bool vtkMRMLROS2RobotNode::AddToROS2Node(const char * nodeId,
                                         const std::string & robotName,
                                         const std::string & parameterNodeName,
                                         const std::string & parameterName,
                                         const std::string & fixedFrame,
                                         const std::string & tfPrefix)
{
  this->SetName(mMRMLNodeName.c_str());
  std::string errorMessage;
  vtkMRMLROS2NodeNode * mrmlROSNodePtr = vtkMRMLROS2::CheckROS2NodeExists(this, nodeId, errorMessage);
  if (!mrmlROSNodePtr) {
    vtkErrorMacro(<< "AddToROS2Node: " << errorMessage);
    return false;
  }
  // Add the robot to the ros2 node
  mrmlROSNodePtr->SetNthNodeReferenceID("robot",
                                        mrmlROSNodePtr->GetNumberOfNodeReferences("robot"),
                                        this->GetID());
  this->SetNodeReferenceID("node", nodeId);
  mMRMLROS2Node = mrmlROSNodePtr;
  mNthRobot.mParameterNodeName = parameterNodeName;
  mNthRobot.mParameterName = parameterName;
  mNthRobot.mFixedFrame = fixedFrame;
  if ((tfPrefix == "") || ((*(tfPrefix.crend()) == '/'))) {
    mNthRobot.mTfPrefix = tfPrefix;
  } else {
    mNthRobot.mTfPrefix = tfPrefix + '/';
  }
  SetRobotDescriptionParameterNode();
  SetRobotName(robotName);
  return true;
}


bool vtkMRMLROS2RobotNode::SetRobotDescriptionParameterNode(void)
{
  // Check if the node is in the scene
  if (!this->GetScene()) {
    vtkErrorMacro(<< "SetRobotDescriptionParameterNode: robot node needs to be added to the scene first");
    return false;
  }
  // Create a new parameter node
  mNthRobot.mRobotDescriptionParameterNode = vtkMRMLROS2ParameterNode::New();
  this->GetScene()->AddNode(mNthRobot.mRobotDescriptionParameterNode);
  mNthRobot.mRobotDescriptionParameterNode->AddToROS2Node(mMRMLROS2Node->GetID(), mNthRobot.mParameterNodeName);
  mNthRobot.mRobotDescriptionParameterNode->AddParameter(mNthRobot.mParameterName);
  ObserveParameterNode(mNthRobot.mRobotDescriptionParameterNode);
  return true;
}


void vtkMRMLROS2RobotNode::ObserveParameterNode(vtkMRMLROS2ParameterNode * node)
{
  // Set up the observer for the robot state publisher
  if (!this->GetScene()->GetNodeByID(node->GetID())) {
    vtkErrorMacro(<< "ObserveParameterNode: robot node is not in the scene.");
    return;
  }
  node->AddObserver(vtkMRMLROS2ParameterNode::ParameterModifiedEvent, this, &vtkMRMLROS2RobotNode::ObserveParameterNodeCallback);
  this->SetAndObserveNodeReferenceID("parameter", node->GetID());
}


void vtkMRMLROS2RobotNode::ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData))
{
  // Manage parameter callback when robot description is available
  vtkMRMLROS2ParameterNode* parameterNode = vtkMRMLROS2ParameterNode::SafeDownCast(caller);
  if (!parameterNode) {
    vtkErrorMacro(<< "ObserveParameterNodeCallback: parameter node is not valid");
    return;
  }
  // Uaw IsParameterSet to check if the parameter is set
  if (!mNthRobot.mRobotDescriptionParameterNode->IsParameterSet("robot_description")) {
    vtkErrorMacro(<< "ObserveParameterNodeCallback: parameter \"robot_description\" is not set.");
    return;
  }

  if (mNthRobot.mRobotDescriptionParameterNode->GetParameterType("robot_description") != "string") {
    std::string outtype = mNthRobot.mRobotDescriptionParameterNode->GetParameterType("robot_description");
    vtkErrorMacro(<< "ObserveParameterNodeCallback: parameter \"robot_description\" is of type " << outtype << " and not string.");
    return;
  }

  mNthRobot.mRobotDescription = mNthRobot.mRobotDescriptionParameterNode->GetParameterAsString("robot_description");
  if (mNumberOfLinks == 0) {
    ParseRobotDescription();
    SetupRobotVisualization();
  }

}


bool vtkMRMLROS2RobotNode::ParseRobotDescription(void)
{
  // Parser the urdf file into an urdf model - to get names of links and pos/ rpy
  if (!mInternals->mURDFModel.initString(mNthRobot.mRobotDescription)) {
    vtkErrorMacro(<< "ParseRobotDescription: failed to parse robot description");
    return false;
  }
  return true;
}


void vtkMRMLROS2RobotNode::InitializeLookupListFromURDF(void)
{
  // This function goes through the urdf file and populates a list of the parents and children of
  // each link transform. This is later used to initialize the robots lookup nodes.

  // Start with the root (base of the robot)
  auto root = mInternals->mURDFModel.getRoot();
  std::string root_name = root->name;
  mNthRobot.mLinkNames.push_back(root_name);
  mNthRobot.mLinkParentNames.push_back(root_name);
  mInternals->mVisualVector.push_back(root->visual);
  mInternals->mMaterialsMap = mInternals->mURDFModel.materials_;
  if (root->visual != nullptr) {
    mInternals->mLinkMaterials.push_back(root->visual->material_name);
  } else {
    mInternals->mLinkMaterials.push_back("");
  }
  // Go through the rest of the robot and save to list
  size_t lastExplored = 0;
  while (lastExplored != mInternals->mVisualVector.size()) {
    mInternals->mParentLinkPointer = mInternals->mURDFModel.getLink(mNthRobot.mLinkNames[lastExplored]);
    mInternals->mChildLinkPointer =  mInternals->mParentLinkPointer->child_links;

    for (auto i: mInternals->mChildLinkPointer) {
      mNthRobot.mLinkNames.push_back(i->name);
      mNthRobot.mLinkParentNames.push_back(mInternals->mParentLinkPointer->name);
      mInternals->mVisualVector.push_back(i->visual); // need to get the origin from the visual
      if (i->visual != nullptr) {
        mInternals->mLinkMaterials.push_back(i->visual->material_name);
      } else {
        mInternals->mLinkMaterials.push_back("");
      }
    }
    lastExplored++;
  }
  mNumberOfLinks = mNthRobot.mLinkNames.size();
  vtkDebugMacro(<< "InitializeLookupListFromURDF complete");
}


void vtkMRMLROS2RobotNode::InitializeOffsetListAndModelFilesFromURDF(void)
{
  // This function goes through the urdf file to obtain the offset for each link and store it in a list.
  // We also get the filename for each stl model for visual loading later.

  // Resize the storage arrays
  mNthRobot.mLinkModelFiles.resize(mInternals->mVisualVector.size());
  mInternals->mLinkOrigins.resize(mInternals->mVisualVector.size());

  // Get the origin and the file names
  for (size_t index = 0; index < mNumberOfLinks; ++index) {
    auto i = mInternals->mVisualVector[index];
    if (i == nullptr) {
      vtkWarningMacro(<< "InitializeOffsetListAndModelFilesFromURDF: no visual vector available for link " << index);
    } else {
      //   urdf::Pose origin;
      auto origin = i->origin;
      mInternals->mLinkOrigins[index] = origin;
      // Get stl file name and add it to a list of vectors for python parsing later
      std::shared_ptr<urdf::Mesh> mesh =  std::dynamic_pointer_cast<urdf::Mesh>(i->geometry); // How do I put this in the internals??
      if (mesh != nullptr) {
	mNthRobot.mRobotScale = {mesh->scale.x, mesh->scale.y, mesh->scale.z};
	// See if the file name uses a package url
        std::string filename = mesh->filename;
        std::regex param_regex("^package:\\/\\/(\\w+)\\/(.*)");
        std::smatch match;
        if (std::regex_search(filename, match, param_regex)) {
          const std::string package = match[1];
          const std::string relativeFile = match[2];
          try {
	    const std::string packageShareDirectory
	      = ament_index_cpp::get_package_share_directory(package);
	    filename = packageShareDirectory + "/" + relativeFile;
	  } catch (...) {
	    vtkErrorMacro(<< "failed to find directory for package "
			  << package
			  << ", did you source the correct workspace setup.bash?");
	  }
        }
        mNthRobot.mLinkModelFiles[index] = filename;
      } else {
        vtkWarningMacro(<< "InitializeOffsetListAndModelFilesFromURDF: link " <<  index << " has a visual, but not a mesh so it won't be displayed");
      }
    }
  }
  vtkDebugMacro(<< "InitializeOffsetListAndModelFilesFromURDF complete");
}


void vtkMRMLROS2RobotNode::InitializeLookups(void)
{
  // Initialize the lookups for the robot based on the previously stored parent and children names of the transform.
  for (size_t i = 0; i < mNumberOfLinks; i++) {
    if (i == 0 && !mNthRobot.mFixedFrame.empty()){
      std::cerr <<  "MY FIXED FRAME: " << mNthRobot.mFixedFrame << std::endl;
      vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookup
        = mMRMLROS2Node->CreateAndAddTf2LookupNode(mNthRobot.mFixedFrame,
                                                   mNthRobot.mTfPrefix + mNthRobot.mLinkNames[i]);
      mNthRobot.mLookupNodes.push_back(lookup);
      this->SetNthNodeReferenceID("lookup", i, lookup->GetID());
    }
    else {
      vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookup
        = mMRMLROS2Node->CreateAndAddTf2LookupNode(mNthRobot.mTfPrefix + mNthRobot.mLinkParentNames[i],
                                                   mNthRobot.mTfPrefix + mNthRobot.mLinkNames[i]);
      mNthRobot.mLookupNodes.push_back(lookup);
      this->SetNthNodeReferenceID("lookup", i, lookup->GetID());
    }
  }
  vtkDebugMacro(<< "InitializeLookups complete");
}


void vtkMRMLROS2RobotNode::InitializeOffsetsAndLinkModels(void)
{
  // Initialize the offset transforms for each link
  for (size_t i = 0; i < mNumberOfLinks; i++) {

    // Create the transform node
    vtkSmartPointer<vtkMRMLTransformNode> transformNode = vtkMRMLTransformNode::New();

    // Translate
    // put this and rpy in the ROS2ToSlicer
    auto origin = mInternals->mLinkOrigins[i];
    vtkSmartPointer<vtkTransform> transform = vtkTransform::New();
    transform->Translate(origin.position.x * MM_TO_M_CONVERSION, origin.position.y * MM_TO_M_CONVERSION, origin.position.z * MM_TO_M_CONVERSION);
    transformNode->SetAndObserveTransformToParent(transform);

    // Rotate
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;
    origin.rotation.getRPY(r, p, y);
    transform->RotateZ(y * (180.0/M_PI)); // RAD to degree conversion - use math.pi instead
    transform->RotateY(p * (180.0/M_PI));
    transform->RotateX(r * (180.0/M_PI));
    transform->Modified();

    // Scale
    vtkNew<vtkMatrix4x4> offsetMatrix;
    transformNode->GetMatrixTransformToParent(offsetMatrix);
    vtkNew<vtkMatrix4x4> MmToM_Transform;
    MmToM_Transform->SetElement(0, 0, MM_TO_M_CONVERSION);
    MmToM_Transform->SetElement(1, 1, MM_TO_M_CONVERSION);
    MmToM_Transform->SetElement(2, 2, MM_TO_M_CONVERSION);
    MmToM_Transform->Multiply4x4(offsetMatrix, MmToM_Transform, offsetMatrix);

    transformNode->SetAndObserveTransformToParent(transform);
    transformNode->SetMatrixTransformToParent(offsetMatrix);
    transform->Modified();

    // Load the model and apply the offset
    std::string filename = mNthRobot.mLinkModelFiles[i];
    vtkNew<vtkSTLReader> reader; // default is STL
    reader->SetFileName(mNthRobot.mLinkModelFiles[i].c_str());
    reader->Update();

    // Apply the scaling factor
    vtkSmartPointer<vtkTransform> scaleRobotTransform = vtkSmartPointer<vtkTransform>::New();
    scaleRobotTransform->Scale(mNthRobot.mRobotScale[0], mNthRobot.mRobotScale[1], mNthRobot.mRobotScale[2]);
    vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
    transformFilter->SetInputConnection(reader->GetOutputPort());
    transformFilter->SetTransform(scaleRobotTransform);
    transformFilter->Update();

    vtkSmartPointer<vtkPointSet> meshFromFile;
    // meshFromFile = reader->GetOutput();
    meshFromFile = transformFilter->GetOutput();
    vtkSmartPointer<vtkPointSet> meshToSetInNode;

    vtkSmartPointer<vtkMRMLModelNode> modelNode = vtkMRMLModelNode::New();
    this->GetScene()->AddNode( modelNode.GetPointer() );
    modelNode->SetName((mNthRobot.mLinkNames[i] + "_model").c_str());
    modelNode->SetAndObserveMesh(meshFromFile);
    mNthRobot.mLinkModels.push_back(modelNode);
    this->SetNthNodeReferenceID("model", i, modelNode->GetID());

    // Create display node
    if (modelNode->GetDisplayNode() == NULL) {
      vtkNew< vtkMRMLModelDisplayNode > displayNode;
      this->GetScene()->AddNode( displayNode.GetPointer() );
      displayNode->SetName((mNthRobot.mLinkNames[i] + "_model_display_node").c_str());
      modelNode->SetAndObserveDisplayNodeID( displayNode->GetID() );
      if (!mInternals->mMaterialsMap.empty()) {
        if (mInternals->mLinkMaterials[i] == "") {
          displayNode->SetColor(0.5, 0.5, 0.5);
        } else {
          displayNode->SetColor((mInternals->mMaterialsMap[mInternals->mLinkMaterials[i]])->color.r, (mInternals->mMaterialsMap[mInternals->mLinkMaterials[i]])->color.g, (mInternals->mMaterialsMap[mInternals->mLinkMaterials[i]])->color.b);
        }
      }
    }
    modelNode->ApplyTransform(transform); // instead of set and observe
  }
  vtkDebugMacro(<< "InitializeOffsetsAndLinkModels complete");
}

void vtkMRMLROS2RobotNode::SetupTransformTree(void)
{
  // This function is used to setup the transform hierarchy to visualize the robot
  // The tree is cascaded lookups (which correspond to the transforms that come from tf2) and each
  // lookup has an offset associated with it. This offset corresponds to the transformation between that
  // link (the child of the lookup) to the base of the robot. The model for each link sits on (observes)
  // it's corresponding offset

  // Cascade the lookups
  for (size_t i = 0; i < mNumberOfLinks; i++) {
    vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookup = mNthRobot.mLookupNodes[i];
    lookup->SetModifiedOnLookup(i == 0); // force modified only for the first link
    std::string parent = lookup->GetParentID();
    for (size_t j = 0; j < mNumberOfLinks; j++) {
      vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> potentialParent = mNthRobot.mLookupNodes[j];
      std::string child = potentialParent->GetChildID();
      if (child == parent) {
        lookup->SetAndObserveTransformNodeID(potentialParent->GetID());
      }
    }
  }

  // Setup models on their corresponding offsets
  for (size_t i = 0; i < mNumberOfLinks; i++) {
    vtkSmartPointer<vtkMRMLModelNode> linkModel = mNthRobot.mLinkModels[i];
    vtkSmartPointer<vtkMRMLROS2Tf2LookupNode> lookup = mNthRobot.mLookupNodes[i];
    linkModel->SetAndObserveTransformNodeID(lookup->GetID());
  }
  vtkDebugMacro(<< "SetupTransformTree complete");
}


void vtkMRMLROS2RobotNode::SetupRobotVisualization(void)
{
  // This function pulls all the pieces together
  // Initialize lookups and offsets, load models, setup the transform tree
  InitializeLookupListFromURDF();
  InitializeOffsetListAndModelFilesFromURDF();
  InitializeOffsetsAndLinkModels();
  InitializeLookups();
  SetupTransformTree();

  mNthRobot.mLinkModels.clear();
  mNthRobot.mLookupNodes.clear();
  mNthRobot.mLinkNames.clear();
  mNthRobot.mLinkParentNames.clear();
  mNthRobot.mLinkModelFiles.clear();
}


void vtkMRMLROS2RobotNode::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os,indent);
}


void vtkMRMLROS2RobotNode::WriteXML(ostream& of, int nIndent)
{
  Superclass::WriteXML(of, nIndent); // This will take care of referenced nodes
  vtkMRMLWriteXMLBeginMacro(of);
  vtkMRMLWriteXMLStdStringMacro(RobotName, RobotName);
  vtkMRMLWriteXMLEndMacro();
}


void vtkMRMLROS2RobotNode::ReadXMLAttributes(const char** atts)
{
  int wasModifying = this->StartModify();
  Superclass::ReadXMLAttributes(atts); // This will take care of referenced nodes
  vtkMRMLReadXMLBeginMacro(atts);
  vtkMRMLReadXMLStdStringMacro(RobotName, RobotName);
  vtkMRMLReadXMLEndMacro();
  this->EndModify(wasModifying);
}

/*
// MoveIt IK implementation (commented out for faster build)
bool vtkMRMLROS2RobotNode::setupIK(const std::string & groupName)
{
  if (!mMRMLROS2Node) {
    vtkErrorMacro(<< "setupIK: ROS2 node not available");
    return false;
  }

  if (mNthRobot.mRobotDescription.empty()) {
    vtkErrorMacro(<< "setupIK: robot description not available");
    return false;
  }

  try {
    auto node = mMRMLROS2Node->mInternals->mNodePointer;
    std::string prefix = "robot_description_kinematics." + groupName;

    // Helper for declaring/updating parameters
    auto ensureParam = [&](const std::string& name, auto value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter(name, value);
      } else {
        node->set_parameter(rclcpp::Parameter(name, value));
      }
    };

    // Set kinematics parameters
    ensureParam(prefix + ".kinematics_solver", std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
    ensureParam(prefix + ".kinematics_solver_search_resolution", 0.005);
    ensureParam(prefix + ".kinematics_solver_timeout", 0.05);

    // Load and cache RobotModel
    RobotModelLoaderPtr = std::make_unique<robot_model_loader::RobotModelLoader>(node, "robot_description");
    RobotModelPtr = RobotModelLoaderPtr->getModel();

    if (!RobotModelPtr) {
      vtkErrorMacro(<< "setupIK: Failed to load RobotModel");
      return false;
    }

    // Cache JointModelGroup
    JointModelGroupPtr = RobotModelPtr->getJointModelGroup(groupName);
    if (!JointModelGroupPtr) {
      vtkErrorMacro(<< "setupIK: joint model group '" << groupName << "' not found");
      return false;
    }

    // Verify solver is available
    const auto& solver = JointModelGroupPtr->getSolverInstance();
    if (!solver) {
      vtkErrorMacro(<< "setupIK: no kinematics solver for group '" << groupName << "'");
      return false;
    }

    IKGroupName = groupName;
    return true;
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "setupIK: exception - " << e.what());
    return false;
  }
}


std::string vtkMRMLROS2RobotNode::FindIK(const std::string& groupName, vtkMatrix4x4* targetPose, const std::string& tipLink, const std::vector<double>& seedJointValues, double timeout)
{
  if (!targetPose) {
    vtkErrorMacro(<< "FindIK: target pose is null");
    return "";
  }

  // Setup IK if needed (or if group changed)
  if (IKGroupName != groupName || !RobotModelPtr || !JointModelGroupPtr) {
    if (!setupIK(groupName)) {
      vtkErrorMacro(<< "FindIK: setupIK failed for group '" << groupName << "'");
      return "";
    }
  }

  try {
    // Create robot state for solving
    moveit::core::RobotState robot_state(RobotModelPtr);

    if (!seedJointValues.empty()) {
      robot_state.setJointGroupPositions(JointModelGroupPtr, seedJointValues);
    } else {
      robot_state.setToDefaultValues();
    }

    // Convert vtkMatrix4x4 to geometry_msgs Pose
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = targetPose->GetElement(0, 3) / 1000.0;  // mm to m
    pose_msg.position.y = targetPose->GetElement(1, 3) / 1000.0;
    pose_msg.position.z = targetPose->GetElement(2, 3) / 1000.0;

    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rot_matrix;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rot_matrix(i, j) = targetPose->GetElement(i, j);
      }
    }

    Eigen::AngleAxisd rotX(-M_PI/2.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotZ(-M_PI/2.0, Eigen::Vector3d::UnitZ());

    rot_matrix = (rotZ.toRotationMatrix() * rotX.toRotationMatrix()) * rot_matrix;
    

    Eigen::Quaterniond quat(rot_matrix);
    pose_msg.orientation.x = quat.x();
    pose_msg.orientation.y = quat.y();
    pose_msg.orientation.z = quat.z();
    pose_msg.orientation.w = quat.w();

    // Call IK using setFromIK
    bool found_ik = robot_state.setFromIK(JointModelGroupPtr, pose_msg, tipLink, timeout);

    // Extract joint values (or create NaN values if no solution found)
    std::vector<double> solution;
    robot_state.copyJointGroupPositions(JointModelGroupPtr, solution);

    // Convert to comma-separated string
    std::ostringstream oss;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i > 0) oss << ",";
      oss << solution[i];
    }

    return oss.str();
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "FindIK: exception - " << e.what());
    return "";
  }
}
*/

// KDL IK Implementation
bool vtkMRMLROS2RobotNode::setupKDLIK(const std::string& rootLink, const std::string& tipLink)
{
  if (mNthRobot.mRobotDescription.empty()) {
    vtkErrorMacro(<< "setupKDLIK: robot description not available");
    return false;
  }

  try {
    // Parse URDF to KDL tree
    KDL::Tree kdlTree;
    if (!kdl_parser::treeFromString(mNthRobot.mRobotDescription, kdlTree)) {
      vtkErrorMacro(<< "setupKDLIK: Failed to parse URDF to KDL tree");
      return false;
    }

    // Extract chain from tree
    KDLChain = std::make_unique<KDL::Chain>();
    if (!kdlTree.getChain(rootLink, tipLink, *KDLChain)) {
      vtkErrorMacro(<< "setupKDLIK: Failed to extract chain from " << rootLink << " to " << tipLink);
      return false;
    }

    // Create solvers (NR without joint limits)
    KDLFkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(*KDLChain);
    KDLIkSolverVel = std::make_unique<KDL::ChainIkSolverVel_pinv>(*KDLChain);
    KDLIkSolver = std::make_unique<KDL::ChainIkSolverPos_NR>(
        *KDLChain, *KDLFkSolver, *KDLIkSolverVel, 100, 1e-6);

    KDLRootLink = rootLink;
    KDLTipLink = tipLink;
    KDLUseJointLimits = false;

    vtkInfoMacro(<< "setupKDLIK: Successfully initialized KDL IK solver (NR) for chain " 
                 << rootLink << " -> " << tipLink 
                 << " with " << KDLChain->getNrOfJoints() << " joints");
    return true;
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "setupKDLIK: exception - " << e.what());
    return false;
  }
}


bool vtkMRMLROS2RobotNode::setupKDLIKWithLimits(const std::string& rootLink, const std::string& tipLink)
{
  if (mNthRobot.mRobotDescription.empty()) {
    vtkErrorMacro(<< "setupKDLIKWithLimits: robot description not available");
    return false;
  }

  try {
    // Parse URDF to KDL tree
    KDL::Tree kdlTree;
    if (!kdl_parser::treeFromString(mNthRobot.mRobotDescription, kdlTree)) {
      vtkErrorMacro(<< "setupKDLIKWithLimits: Failed to parse URDF to KDL tree");
      return false;
    }

    // Extract chain from tree
    KDLChain = std::make_unique<KDL::Chain>();
    if (!kdlTree.getChain(rootLink, tipLink, *KDLChain)) {
      vtkErrorMacro(<< "setupKDLIKWithLimits: Failed to extract chain from " << rootLink << " to " << tipLink);
      return false;
    }else{
      vtkInfoMacro(<< "setupKDLIKWithLimits: Successfully extracted chain from " << rootLink << " to " << tipLink);
    }

    unsigned int nj = KDLChain->getNrOfJoints();

    // Initialize joint limits arrays
    KDLJointMin = KDL::JntArray(nj);
    KDLJointMax = KDL::JntArray(nj);

    // Extract joint limits from URDF
    urdf::Model urdfModel;
    if (!urdfModel.initString(mNthRobot.mRobotDescription)) {
      vtkErrorMacro(<< "setupKDLIKWithLimits: Failed to parse URDF model");
      return false;
    }

  unsigned int joint_idx = 0;
      for (unsigned int i = 0; i < KDLChain->getNrOfSegments(); i++) {
        const KDL::Segment& segment = KDLChain->getSegment(i);
        const KDL::Joint& joint = segment.getJoint();
        
        if (joint.getType() != KDL::Joint::None) {
          std::string joint_name = joint.getName();
          auto urdf_joint = urdfModel.getJoint(joint_name);
          
          // Check if joint is continuous
          if (urdf_joint && urdf_joint->type == urdf::Joint::CONTINUOUS) {
              KDLJointMin(joint_idx) = -2 * M_PI; 
              KDLJointMax(joint_idx) = 2 * M_PI;
              vtkInfoMacro(<< "  Joint " << joint_name << " is CONTINUOUS. Setting wide limits [-2pi, 2pi].");
          } 
          else if (urdf_joint && urdf_joint->limits) {
            // Otherwise, get limits if available
            KDLJointMin(joint_idx) = urdf_joint->limits->lower;
            KDLJointMax(joint_idx) = urdf_joint->limits->upper;
          } 
          else {
            // Default limits if not specified
            KDLJointMin(joint_idx) = -M_PI;
            KDLJointMax(joint_idx) = M_PI;
          }
          
          joint_idx++;
        }
      }

    // Create solvers (NR_JL with joint limits)
    KDLFkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(*KDLChain);
    KDLIkSolverVel = std::make_unique<KDL::ChainIkSolverVel_pinv>(*KDLChain);
    KDLIkSolverJL = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
        *KDLChain, KDLJointMin, KDLJointMax, *KDLFkSolver, *KDLIkSolverVel, 100, 1e-6);

    KDLRootLink = rootLink;
    KDLTipLink = tipLink;
    KDLUseJointLimits = true;

    vtkInfoMacro(<< "setupKDLIKWithLimits: Successfully initialized KDL IK solver (NR_JL) for chain " 
                 << rootLink << " -> " << tipLink 
                 << " with " << KDLChain->getNrOfJoints() << " joints and joint limits");

    // print joint limits for debugging
    for (unsigned int j = 0; j < nj; j++) {
      vtkInfoMacro(<< "  Joint " << j << " limits: [" 
                   << KDLJointMin(j) << ", " << KDLJointMax(j) << "]");
    } 
    return true;
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "setupKDLIKWithLimits: exception - " << e.what());
    return false;
  }
}


std::string vtkMRMLROS2RobotNode::FindKDLIK(vtkMatrix4x4* targetPose, 
                                             const std::vector<double>& seedJointValues)
{
  if (!targetPose) {
    vtkErrorMacro(<< "FindKDLIK: target pose is null");
    return "";
  }

  if (!KDLUseJointLimits && !KDLIkSolver) {
    vtkErrorMacro(<< "FindKDLIK: KDL IK solver not initialized. Call setupKDLIK or setupKDLIKWithLimits first");
    return "";
  }

  if (KDLUseJointLimits && !KDLIkSolverJL) {
    vtkErrorMacro(<< "FindKDLIK: KDL IK solver with joint limits not initialized. Call setupKDLIKWithLimits first");
    return "";
  }

  try {
    // Convert vtkMatrix4x4 to KDL Frame
    KDL::Frame targetFrame;
    
    // Set position (convert mm to m)
    targetFrame.p.x(targetPose->GetElement(0, 3) / 1000.0);
    targetFrame.p.y(targetPose->GetElement(1, 3) / 1000.0);
    targetFrame.p.z(targetPose->GetElement(2, 3) / 1000.0);
    
    // Set rotation
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        targetFrame.M(i, j) = targetPose->GetElement(i, j);
      }
    }

    // Setup seed configuration
    KDL::JntArray qSeed(KDLChain->getNrOfJoints());
    if (!seedJointValues.empty() && seedJointValues.size() == KDLChain->getNrOfJoints()) {
      for (size_t i = 0; i < seedJointValues.size(); i++) {
        qSeed(i) = seedJointValues[i];
      }
    } else {
      // Use zeros as default seed
      qSeed.data.setZero();
    }

    // Solve IK
    KDL::JntArray qSolution(KDLChain->getNrOfJoints());
    int result;
    
    if (KDLUseJointLimits) {
      result = KDLIkSolverJL->CartToJnt(qSeed, targetFrame, qSolution);
    } else {
      result = KDLIkSolver->CartToJnt(qSeed, targetFrame, qSolution);
    }

    if (result < 0) {
      vtkWarningMacro(<< "FindKDLIK: IK solution not found (error code: " << result << ")");
      return "";
    }

    // Convert solution to comma-separated string
    std::ostringstream oss;
    for (unsigned int i = 0; i < qSolution.rows(); i++) {
      if (i > 0) oss << ",";
      oss << qSolution(i);
    }

    return oss.str();
  }
  catch (const std::exception& e) {
    vtkErrorMacro(<< "FindKDLIK: exception - " << e.what());
    return "";
  }
}


void vtkMRMLROS2RobotNode::UpdateScene(vtkMRMLScene *scene)
{
  Superclass::UpdateScene(scene);
  int nbNodeRefs = this->GetNumberOfNodeReferences("node");
  if (nbNodeRefs == 0) {
    // assigned to the default ROS node
    auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
    auto nodeId = defaultNode->GetID();
    if(!defaultNode) {
      vtkErrorMacro(<< "UpdateScene: default ros2 node unavailable. Unable to set reference for broadcaster \"" << GetName() << "\"");
      return;
    }
    defaultNode->SetNthNodeReferenceID("robot", defaultNode->GetNumberOfNodeReferences("robot"),this->GetID());
    this->SetNodeReferenceID("node", nodeId);
  } else if (nbNodeRefs == 1) {
    auto defaultNode = scene->GetFirstNodeByName("ros2:node:slicer");
    auto nodeId = defaultNode->GetID();
    defaultNode->SetNthNodeReferenceID("robot", defaultNode->GetNumberOfNodeReferences("robot"), this->GetID());
    this->SetNodeReferenceID("node", nodeId);
  } else {
    vtkErrorMacro(<< "UpdateScene: more than one ROS2 node reference defined for broadcaster \"" << GetName() << "\"");
  }
}
