classdef MotionPlanResponse < robotics.ros.Message
    %MotionPlanResponse MATLAB implementation of moveit_msgs/MotionPlanResponse
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/MotionPlanResponse' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'e493d20ab41424c48f671e152c70fc74' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        MoveitMsgsMoveItErrorCodesClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/MoveItErrorCodes') % Dispatch to MATLAB class for message type moveit_msgs/MoveItErrorCodes
        MoveitMsgsRobotStateClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/RobotState') % Dispatch to MATLAB class for message type moveit_msgs/RobotState
        MoveitMsgsRobotTrajectoryClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/RobotTrajectory') % Dispatch to MATLAB class for message type moveit_msgs/RobotTrajectory
    end
    
    properties (Dependent)
        TrajectoryStart
        GroupName
        Trajectory
        PlanningTime
        ErrorCode
    end
    
    properties (Access = protected)
        Cache = struct('TrajectoryStart', [], 'Trajectory', [], 'ErrorCode', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'ErrorCode', 'GroupName', 'PlanningTime', 'Trajectory', 'TrajectoryStart'} % List of non-constant message properties
        ROSPropertyList = {'error_code', 'group_name', 'planning_time', 'trajectory', 'trajectory_start'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = MotionPlanResponse(msg)
            %MotionPlanResponse Construct the message object MotionPlanResponse
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function trajectorystart = get.TrajectoryStart(obj)
            %get.TrajectoryStart Get the value for property TrajectoryStart
            if isempty(obj.Cache.TrajectoryStart)
                obj.Cache.TrajectoryStart = feval(obj.MoveitMsgsRobotStateClass, obj.JavaMessage.getTrajectoryStart);
            end
            trajectorystart = obj.Cache.TrajectoryStart;
        end
        
        function set.TrajectoryStart(obj, trajectorystart)
            %set.TrajectoryStart Set the value for property TrajectoryStart
            validateattributes(trajectorystart, {obj.MoveitMsgsRobotStateClass}, {'nonempty', 'scalar'}, 'MotionPlanResponse', 'TrajectoryStart');
            
            obj.JavaMessage.setTrajectoryStart(trajectorystart.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.TrajectoryStart)
                obj.Cache.TrajectoryStart.setJavaObject(trajectorystart.getJavaObject);
            end
        end
        
        function groupname = get.GroupName(obj)
            %get.GroupName Get the value for property GroupName
            groupname = char(obj.JavaMessage.getGroupName);
        end
        
        function set.GroupName(obj, groupname)
            %set.GroupName Set the value for property GroupName
            validateattributes(groupname, {'char'}, {}, 'MotionPlanResponse', 'GroupName');
            
            obj.JavaMessage.setGroupName(groupname);
        end
        
        function trajectory = get.Trajectory(obj)
            %get.Trajectory Get the value for property Trajectory
            if isempty(obj.Cache.Trajectory)
                obj.Cache.Trajectory = feval(obj.MoveitMsgsRobotTrajectoryClass, obj.JavaMessage.getTrajectory);
            end
            trajectory = obj.Cache.Trajectory;
        end
        
        function set.Trajectory(obj, trajectory)
            %set.Trajectory Set the value for property Trajectory
            validateattributes(trajectory, {obj.MoveitMsgsRobotTrajectoryClass}, {'nonempty', 'scalar'}, 'MotionPlanResponse', 'Trajectory');
            
            obj.JavaMessage.setTrajectory(trajectory.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Trajectory)
                obj.Cache.Trajectory.setJavaObject(trajectory.getJavaObject);
            end
        end
        
        function planningtime = get.PlanningTime(obj)
            %get.PlanningTime Get the value for property PlanningTime
            planningtime = double(obj.JavaMessage.getPlanningTime);
        end
        
        function set.PlanningTime(obj, planningtime)
            %set.PlanningTime Set the value for property PlanningTime
            validateattributes(planningtime, {'numeric'}, {'nonempty', 'scalar'}, 'MotionPlanResponse', 'PlanningTime');
            
            obj.JavaMessage.setPlanningTime(planningtime);
        end
        
        function errorcode = get.ErrorCode(obj)
            %get.ErrorCode Get the value for property ErrorCode
            if isempty(obj.Cache.ErrorCode)
                obj.Cache.ErrorCode = feval(obj.MoveitMsgsMoveItErrorCodesClass, obj.JavaMessage.getErrorCode);
            end
            errorcode = obj.Cache.ErrorCode;
        end
        
        function set.ErrorCode(obj, errorcode)
            %set.ErrorCode Set the value for property ErrorCode
            validateattributes(errorcode, {obj.MoveitMsgsMoveItErrorCodesClass}, {'nonempty', 'scalar'}, 'MotionPlanResponse', 'ErrorCode');
            
            obj.JavaMessage.setErrorCode(errorcode.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.ErrorCode)
                obj.Cache.ErrorCode.setJavaObject(errorcode.getJavaObject);
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.TrajectoryStart = [];
            obj.Cache.Trajectory = [];
            obj.Cache.ErrorCode = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.GroupName = obj.GroupName;
            cpObj.PlanningTime = obj.PlanningTime;
            
            % Recursively copy compound properties
            cpObj.TrajectoryStart = copy(obj.TrajectoryStart);
            cpObj.Trajectory = copy(obj.Trajectory);
            cpObj.ErrorCode = copy(obj.ErrorCode);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.GroupName = strObj.GroupName;
            obj.PlanningTime = strObj.PlanningTime;
            obj.TrajectoryStart = feval([obj.MoveitMsgsRobotStateClass '.loadobj'], strObj.TrajectoryStart);
            obj.Trajectory = feval([obj.MoveitMsgsRobotTrajectoryClass '.loadobj'], strObj.Trajectory);
            obj.ErrorCode = feval([obj.MoveitMsgsMoveItErrorCodesClass '.loadobj'], strObj.ErrorCode);
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.GroupName = obj.GroupName;
            strObj.PlanningTime = obj.PlanningTime;
            strObj.TrajectoryStart = saveobj(obj.TrajectoryStart);
            strObj.Trajectory = saveobj(obj.Trajectory);
            strObj.ErrorCode = saveobj(obj.ErrorCode);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.MotionPlanResponse.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.MotionPlanResponse;
            obj.reload(strObj);
        end
    end
end
