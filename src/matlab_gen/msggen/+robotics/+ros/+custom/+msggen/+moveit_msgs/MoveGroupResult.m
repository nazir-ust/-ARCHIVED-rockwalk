classdef MoveGroupResult < robotics.ros.Message
    %MoveGroupResult MATLAB implementation of moveit_msgs/MoveGroupResult
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/MoveGroupResult' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '34098589d402fee7ae9c3fd413e5a6c6' % The MD5 Checksum of the message definition
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
        ErrorCode
        TrajectoryStart
        PlannedTrajectory
        ExecutedTrajectory
        PlanningTime
    end
    
    properties (Access = protected)
        Cache = struct('ErrorCode', [], 'TrajectoryStart', [], 'PlannedTrajectory', [], 'ExecutedTrajectory', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'ErrorCode', 'ExecutedTrajectory', 'PlannedTrajectory', 'PlanningTime', 'TrajectoryStart'} % List of non-constant message properties
        ROSPropertyList = {'error_code', 'executed_trajectory', 'planned_trajectory', 'planning_time', 'trajectory_start'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = MoveGroupResult(msg)
            %MoveGroupResult Construct the message object MoveGroupResult
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
        
        function errorcode = get.ErrorCode(obj)
            %get.ErrorCode Get the value for property ErrorCode
            if isempty(obj.Cache.ErrorCode)
                obj.Cache.ErrorCode = feval(obj.MoveitMsgsMoveItErrorCodesClass, obj.JavaMessage.getErrorCode);
            end
            errorcode = obj.Cache.ErrorCode;
        end
        
        function set.ErrorCode(obj, errorcode)
            %set.ErrorCode Set the value for property ErrorCode
            validateattributes(errorcode, {obj.MoveitMsgsMoveItErrorCodesClass}, {'nonempty', 'scalar'}, 'MoveGroupResult', 'ErrorCode');
            
            obj.JavaMessage.setErrorCode(errorcode.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.ErrorCode)
                obj.Cache.ErrorCode.setJavaObject(errorcode.getJavaObject);
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
            validateattributes(trajectorystart, {obj.MoveitMsgsRobotStateClass}, {'nonempty', 'scalar'}, 'MoveGroupResult', 'TrajectoryStart');
            
            obj.JavaMessage.setTrajectoryStart(trajectorystart.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.TrajectoryStart)
                obj.Cache.TrajectoryStart.setJavaObject(trajectorystart.getJavaObject);
            end
        end
        
        function plannedtrajectory = get.PlannedTrajectory(obj)
            %get.PlannedTrajectory Get the value for property PlannedTrajectory
            if isempty(obj.Cache.PlannedTrajectory)
                obj.Cache.PlannedTrajectory = feval(obj.MoveitMsgsRobotTrajectoryClass, obj.JavaMessage.getPlannedTrajectory);
            end
            plannedtrajectory = obj.Cache.PlannedTrajectory;
        end
        
        function set.PlannedTrajectory(obj, plannedtrajectory)
            %set.PlannedTrajectory Set the value for property PlannedTrajectory
            validateattributes(plannedtrajectory, {obj.MoveitMsgsRobotTrajectoryClass}, {'nonempty', 'scalar'}, 'MoveGroupResult', 'PlannedTrajectory');
            
            obj.JavaMessage.setPlannedTrajectory(plannedtrajectory.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.PlannedTrajectory)
                obj.Cache.PlannedTrajectory.setJavaObject(plannedtrajectory.getJavaObject);
            end
        end
        
        function executedtrajectory = get.ExecutedTrajectory(obj)
            %get.ExecutedTrajectory Get the value for property ExecutedTrajectory
            if isempty(obj.Cache.ExecutedTrajectory)
                obj.Cache.ExecutedTrajectory = feval(obj.MoveitMsgsRobotTrajectoryClass, obj.JavaMessage.getExecutedTrajectory);
            end
            executedtrajectory = obj.Cache.ExecutedTrajectory;
        end
        
        function set.ExecutedTrajectory(obj, executedtrajectory)
            %set.ExecutedTrajectory Set the value for property ExecutedTrajectory
            validateattributes(executedtrajectory, {obj.MoveitMsgsRobotTrajectoryClass}, {'nonempty', 'scalar'}, 'MoveGroupResult', 'ExecutedTrajectory');
            
            obj.JavaMessage.setExecutedTrajectory(executedtrajectory.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.ExecutedTrajectory)
                obj.Cache.ExecutedTrajectory.setJavaObject(executedtrajectory.getJavaObject);
            end
        end
        
        function planningtime = get.PlanningTime(obj)
            %get.PlanningTime Get the value for property PlanningTime
            planningtime = double(obj.JavaMessage.getPlanningTime);
        end
        
        function set.PlanningTime(obj, planningtime)
            %set.PlanningTime Set the value for property PlanningTime
            validateattributes(planningtime, {'numeric'}, {'nonempty', 'scalar'}, 'MoveGroupResult', 'PlanningTime');
            
            obj.JavaMessage.setPlanningTime(planningtime);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.ErrorCode = [];
            obj.Cache.TrajectoryStart = [];
            obj.Cache.PlannedTrajectory = [];
            obj.Cache.ExecutedTrajectory = [];
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
            cpObj.PlanningTime = obj.PlanningTime;
            
            % Recursively copy compound properties
            cpObj.ErrorCode = copy(obj.ErrorCode);
            cpObj.TrajectoryStart = copy(obj.TrajectoryStart);
            cpObj.PlannedTrajectory = copy(obj.PlannedTrajectory);
            cpObj.ExecutedTrajectory = copy(obj.ExecutedTrajectory);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.PlanningTime = strObj.PlanningTime;
            obj.ErrorCode = feval([obj.MoveitMsgsMoveItErrorCodesClass '.loadobj'], strObj.ErrorCode);
            obj.TrajectoryStart = feval([obj.MoveitMsgsRobotStateClass '.loadobj'], strObj.TrajectoryStart);
            obj.PlannedTrajectory = feval([obj.MoveitMsgsRobotTrajectoryClass '.loadobj'], strObj.PlannedTrajectory);
            obj.ExecutedTrajectory = feval([obj.MoveitMsgsRobotTrajectoryClass '.loadobj'], strObj.ExecutedTrajectory);
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
            
            strObj.PlanningTime = obj.PlanningTime;
            strObj.ErrorCode = saveobj(obj.ErrorCode);
            strObj.TrajectoryStart = saveobj(obj.TrajectoryStart);
            strObj.PlannedTrajectory = saveobj(obj.PlannedTrajectory);
            strObj.ExecutedTrajectory = saveobj(obj.ExecutedTrajectory);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.MoveGroupResult.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.MoveGroupResult;
            obj.reload(strObj);
        end
    end
end
