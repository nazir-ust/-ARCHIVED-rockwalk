package moveit_msgs;

public interface MoveGroupResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "moveit_msgs/MoveGroupResult";
  static final java.lang.String _DEFINITION = "\n# An error code reflecting what went wrong\nMoveItErrorCodes error_code\n\n# The full starting state of the robot at the start of the trajectory\nmoveit_msgs/RobotState trajectory_start\n\n# The trajectory that moved group produced for execution\nmoveit_msgs/RobotTrajectory planned_trajectory\n\n# The trace of the trajectory recorded during execution\nmoveit_msgs/RobotTrajectory executed_trajectory\n\n# The amount of time it took to complete the motion plan\nfloat64 planning_time\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  moveit_msgs.MoveItErrorCodes getErrorCode();
  void setErrorCode(moveit_msgs.MoveItErrorCodes value);
  moveit_msgs.RobotState getTrajectoryStart();
  void setTrajectoryStart(moveit_msgs.RobotState value);
  moveit_msgs.RobotTrajectory getPlannedTrajectory();
  void setPlannedTrajectory(moveit_msgs.RobotTrajectory value);
  moveit_msgs.RobotTrajectory getExecutedTrajectory();
  void setExecutedTrajectory(moveit_msgs.RobotTrajectory value);
  double getPlanningTime();
  void setPlanningTime(double value);
}