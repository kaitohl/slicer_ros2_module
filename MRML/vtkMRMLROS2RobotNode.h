#ifndef __vtkMRMLROS2RobotNode_h
#define __vtkMRMLROS2RobotNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <vtkMatrix4x4.h>

#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <vtkMRMLROS2RobotNodeInternals.h>

// MoveIt includes (libraries are linked)
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class vtkMRMLROS2NodeNode;
class vtkMRMLROS2ParameterNode;
class vtkMRMLROS2Tf2LookupNode;
class vtkMRMLModelNode;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2RobotNode: public vtkMRMLNode
{

 public:
  typedef vtkMRMLROS2RobotNode SelfType;
  vtkTypeMacro(vtkMRMLROS2RobotNode, vtkMRMLNode);
  static SelfType * New(void);
  void PrintSelf(std::ostream& os, vtkIndent indent) override;
  vtkMRMLNode * CreateNodeInstance(void) override;
  const char * GetNodeTagName(void) override;

  inline const std::string GetRobotName(void) const {
    return mRobotName;
  }

  // For ReadXMLAttributes
  inline void SetRobotName(const std::string & name) {
    mRobotName = name;
    mMRMLNodeName = "ros2:robot:" + name;
    this->SetName(mMRMLNodeName.c_str()); 
  }

  bool AddToROS2Node(const char * nodeId,
                     const std::string & robotName,
		     const std::string & parameterNodeName,
		     const std::string & parameterName = "robot_description",
		     const std::string & fixedFrame = "",
                     const std::string & tfPrefix = "");

  bool SetRobotDescriptionParameterNode();
  void ObserveParameterNode(vtkMRMLROS2ParameterNode * node);

  bool ParseRobotDescription(void);
  void InitializeLookupListFromURDF(void);
  void InitializeOffsetListAndModelFilesFromURDF(void);

  void InitializeLookups(void);
  void InitializeOffsetsAndLinkModels(void);
  void SetupTransformTree(void);
  void SetupRobotVisualization(void);

  // Test MoveIt: parse URDF and return joint count
  int TestMoveItURDFParse(void);

  // Find IK solution: returns joint values as comma-separated string, empty if failed
  std::string FindIK(const std::string& groupName, 
                     vtkMatrix4x4* targetPose, 
                     const std::string& tipLink,
                     double timeout = 1.0);

  // Save and load
  void ReadXMLAttributes(const char** atts) override;
  void WriteXML(std::ostream& of, int indent) override;
  void UpdateScene(vtkMRMLScene *scene) override;

 protected:
  vtkMRMLROS2RobotNode();
  ~vtkMRMLROS2RobotNode();

  void ObserveParameterNodeCallback( vtkObject* caller, unsigned long, void* vtkNotUsed(callData));

  struct {
    std::vector<std::string> mLinkNames;
    std::vector<std::string> mLinkParentNames;
    std::vector<std::string> mLinkModelFiles;
    std::vector<double> mRobotScale;
    std::vector<vtkSmartPointer<vtkMRMLModelNode>> mLinkModels;
    std::vector<vtkSmartPointer<vtkMRMLROS2Tf2LookupNode>> mLookupNodes;
    std::string mRobotDescription = "";
    vtkSmartPointer<vtkMRMLROS2ParameterNode> mRobotDescriptionParameterNode;
    std::string mParameterNodeName;
    std::string mParameterName;
    std::string mFixedFrame;
    std::string mTfPrefix;
  } mNthRobot;

  std::string mRobotName = "undefined";
  std::string mMRMLNodeName = "ros2:robot";
  vtkSmartPointer<vtkMRMLROS2NodeNode> mMRMLROS2Node;
  std::unique_ptr<vtkMRMLROS2RobotNodeInternals> mInternals;
  size_t mNumberOfLinks = 0;

  // (MoveIt member variables removed)

};

#endif // __vtkMRMLROS2RobotNode_h
