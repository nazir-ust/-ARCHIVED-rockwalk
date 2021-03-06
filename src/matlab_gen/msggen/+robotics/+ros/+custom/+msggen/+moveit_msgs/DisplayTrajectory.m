classdef DisplayTrajectory < robotics.ros.Message
    %DisplayTrajectory MATLAB implementation of moveit_msgs/DisplayTrajectory
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/DisplayTrajectory' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'c3c039261ab9e8a11457dac56b6316c8' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        MoveitMsgsRobotStateClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/RobotState') % Dispatch to MATLAB class for message type moveit_msgs/RobotState
        MoveitMsgsRobotTrajectoryClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/RobotTrajectory') % Dispatch to MATLAB class for message type moveit_msgs/RobotTrajectory
    end
    
    properties (Dependent)
        ModelId
        TrajectoryStart
        Trajectory
    end
    
    properties (Access = protected)
        Cache = struct('Trajectory', [], 'TrajectoryStart', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'ModelId', 'Trajectory', 'TrajectoryStart'} % List of non-constant message properties
        ROSPropertyList = {'model_id', 'trajectory', 'trajectory_start'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = DisplayTrajectory(msg)
            %DisplayTrajectory Construct the message object DisplayTrajectory
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
        
        function modelid = get.ModelId(obj)
            %get.ModelId Get the value for property ModelId
            modelid = char(obj.JavaMessage.getModelId);
        end
        
        function set.ModelId(obj, modelid)
            %set.ModelId Set the value for property ModelId
            validateattributes(modelid, {'char'}, {}, 'DisplayTrajectory', 'ModelId');
            
            obj.JavaMessage.setModelId(modelid);
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
            validateattributes(trajectorystart, {obj.MoveitMsgsRobotStateClass}, {'nonempty', 'scalar'}, 'DisplayTrajectory', 'TrajectoryStart');
            
            obj.JavaMessage.setTrajectoryStart(trajectorystart.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.TrajectoryStart)
                obj.Cache.TrajectoryStart.setJavaObject(trajectorystart.getJavaObject);
            end
        end
        
        function trajectory = get.Trajectory(obj)
            %get.Trajectory Get the value for property Trajectory
            if isempty(obj.Cache.Trajectory)
                javaArray = obj.JavaMessage.getTrajectory;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsRobotTrajectoryClass);
                obj.Cache.Trajectory = feval(obj.MoveitMsgsRobotTrajectoryClass, array);
            end
            trajectory = obj.Cache.Trajectory;
        end
        
        function set.Trajectory(obj, trajectory)
            %set.Trajectory Set the value for property Trajectory
            if ~isvector(trajectory) && isempty(trajectory)
                % Allow empty [] input
                trajectory = feval([obj.MoveitMsgsRobotTrajectoryClass '.empty'], 0, 1);
            end
            
            validateattributes(trajectory, {obj.MoveitMsgsRobotTrajectoryClass}, {'vector'}, 'DisplayTrajectory', 'Trajectory');
            
            javaArray = obj.JavaMessage.getTrajectory;
            array = obj.writeJavaArray(trajectory, javaArray, obj.MoveitMsgsRobotTrajectoryClass);
            obj.JavaMessage.setTrajectory(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Trajectory)
                obj.Cache.Trajectory = [];
                obj.Cache.Trajectory = obj.Trajectory;
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Trajectory = [];
            obj.Cache.TrajectoryStart = [];
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
            cpObj.ModelId = obj.ModelId;
            
            % Recursively copy compound properties
            cpObj.TrajectoryStart = copy(obj.TrajectoryStart);
            cpObj.Trajectory = copy(obj.Trajectory);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.ModelId = strObj.ModelId;
            obj.TrajectoryStart = feval([obj.MoveitMsgsRobotStateClass '.loadobj'], strObj.TrajectoryStart);
            TrajectoryCell = arrayfun(@(x) feval([obj.MoveitMsgsRobotTrajectoryClass '.loadobj'], x), strObj.Trajectory, 'UniformOutput', false);
            obj.Trajectory = vertcat(TrajectoryCell{:});
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
            
            strObj.ModelId = obj.ModelId;
            strObj.TrajectoryStart = saveobj(obj.TrajectoryStart);
            strObj.Trajectory = arrayfun(@(x) saveobj(x), obj.Trajectory);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.DisplayTrajectory.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.DisplayTrajectory;
            obj.reload(strObj);
        end
    end
end
