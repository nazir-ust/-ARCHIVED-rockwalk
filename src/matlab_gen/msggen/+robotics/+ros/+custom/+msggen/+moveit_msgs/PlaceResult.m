classdef PlaceResult < robotics.ros.Message
    %PlaceResult MATLAB implementation of moveit_msgs/PlaceResult
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/PlaceResult' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'da2eea14de05cf0aa280f150a84ded50' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        MoveitMsgsMoveItErrorCodesClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/MoveItErrorCodes') % Dispatch to MATLAB class for message type moveit_msgs/MoveItErrorCodes
        MoveitMsgsPlaceLocationClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/PlaceLocation') % Dispatch to MATLAB class for message type moveit_msgs/PlaceLocation
        MoveitMsgsRobotStateClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/RobotState') % Dispatch to MATLAB class for message type moveit_msgs/RobotState
        MoveitMsgsRobotTrajectoryClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/RobotTrajectory') % Dispatch to MATLAB class for message type moveit_msgs/RobotTrajectory
    end
    
    properties (Dependent)
        ErrorCode
        TrajectoryStart
        PlaceLocation
        TrajectoryStages
        TrajectoryDescriptions
    end
    
    properties (Access = protected)
        Cache = struct('ErrorCode', [], 'TrajectoryStart', [], 'TrajectoryStages', [], 'PlaceLocation', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'ErrorCode', 'PlaceLocation', 'TrajectoryDescriptions', 'TrajectoryStages', 'TrajectoryStart'} % List of non-constant message properties
        ROSPropertyList = {'error_code', 'place_location', 'trajectory_descriptions', 'trajectory_stages', 'trajectory_start'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = PlaceResult(msg)
            %PlaceResult Construct the message object PlaceResult
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
            validateattributes(errorcode, {obj.MoveitMsgsMoveItErrorCodesClass}, {'nonempty', 'scalar'}, 'PlaceResult', 'ErrorCode');
            
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
            validateattributes(trajectorystart, {obj.MoveitMsgsRobotStateClass}, {'nonempty', 'scalar'}, 'PlaceResult', 'TrajectoryStart');
            
            obj.JavaMessage.setTrajectoryStart(trajectorystart.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.TrajectoryStart)
                obj.Cache.TrajectoryStart.setJavaObject(trajectorystart.getJavaObject);
            end
        end
        
        function placelocation = get.PlaceLocation(obj)
            %get.PlaceLocation Get the value for property PlaceLocation
            if isempty(obj.Cache.PlaceLocation)
                obj.Cache.PlaceLocation = feval(obj.MoveitMsgsPlaceLocationClass, obj.JavaMessage.getPlaceLocation);
            end
            placelocation = obj.Cache.PlaceLocation;
        end
        
        function set.PlaceLocation(obj, placelocation)
            %set.PlaceLocation Set the value for property PlaceLocation
            validateattributes(placelocation, {obj.MoveitMsgsPlaceLocationClass}, {'nonempty', 'scalar'}, 'PlaceResult', 'PlaceLocation');
            
            obj.JavaMessage.setPlaceLocation(placelocation.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.PlaceLocation)
                obj.Cache.PlaceLocation.setJavaObject(placelocation.getJavaObject);
            end
        end
        
        function trajectorystages = get.TrajectoryStages(obj)
            %get.TrajectoryStages Get the value for property TrajectoryStages
            if isempty(obj.Cache.TrajectoryStages)
                javaArray = obj.JavaMessage.getTrajectoryStages;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsRobotTrajectoryClass);
                obj.Cache.TrajectoryStages = feval(obj.MoveitMsgsRobotTrajectoryClass, array);
            end
            trajectorystages = obj.Cache.TrajectoryStages;
        end
        
        function set.TrajectoryStages(obj, trajectorystages)
            %set.TrajectoryStages Set the value for property TrajectoryStages
            if ~isvector(trajectorystages) && isempty(trajectorystages)
                % Allow empty [] input
                trajectorystages = feval([obj.MoveitMsgsRobotTrajectoryClass '.empty'], 0, 1);
            end
            
            validateattributes(trajectorystages, {obj.MoveitMsgsRobotTrajectoryClass}, {'vector'}, 'PlaceResult', 'TrajectoryStages');
            
            javaArray = obj.JavaMessage.getTrajectoryStages;
            array = obj.writeJavaArray(trajectorystages, javaArray, obj.MoveitMsgsRobotTrajectoryClass);
            obj.JavaMessage.setTrajectoryStages(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.TrajectoryStages)
                obj.Cache.TrajectoryStages = [];
                obj.Cache.TrajectoryStages = obj.TrajectoryStages;
            end
        end
        
        function trajectorydescriptions = get.TrajectoryDescriptions(obj)
            %get.TrajectoryDescriptions Get the value for property TrajectoryDescriptions
            javaArray = obj.JavaMessage.getTrajectoryDescriptions;
            array = obj.readJavaArray(javaArray, 'char');
            trajectorydescriptions = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.TrajectoryDescriptions(obj, trajectorydescriptions)
            %set.TrajectoryDescriptions Set the value for property TrajectoryDescriptions
            if ~isvector(trajectorydescriptions) && isempty(trajectorydescriptions)
                % Allow empty [] input
                trajectorydescriptions = cell.empty(0,1);
            end
            
            validateattributes(trajectorydescriptions, {'cell'}, {'vector'}, 'PlaceResult', 'TrajectoryDescriptions');
            if any(cellfun(@(x) ~ischar(x), trajectorydescriptions))
                error(message('robotics:ros:message:CellArrayStringError', ...
                    'trajectorydescriptions'));
            end
            
            javaArray = obj.JavaMessage.getTrajectoryDescriptions;
            array = obj.writeJavaArray(trajectorydescriptions, javaArray, 'char');
            obj.JavaMessage.setTrajectoryDescriptions(array);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.ErrorCode = [];
            obj.Cache.TrajectoryStart = [];
            obj.Cache.TrajectoryStages = [];
            obj.Cache.PlaceLocation = [];
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
            cpObj.TrajectoryDescriptions = obj.TrajectoryDescriptions;
            
            % Recursively copy compound properties
            cpObj.ErrorCode = copy(obj.ErrorCode);
            cpObj.TrajectoryStart = copy(obj.TrajectoryStart);
            cpObj.PlaceLocation = copy(obj.PlaceLocation);
            cpObj.TrajectoryStages = copy(obj.TrajectoryStages);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.TrajectoryDescriptions = strObj.TrajectoryDescriptions;
            obj.ErrorCode = feval([obj.MoveitMsgsMoveItErrorCodesClass '.loadobj'], strObj.ErrorCode);
            obj.TrajectoryStart = feval([obj.MoveitMsgsRobotStateClass '.loadobj'], strObj.TrajectoryStart);
            obj.PlaceLocation = feval([obj.MoveitMsgsPlaceLocationClass '.loadobj'], strObj.PlaceLocation);
            TrajectoryStagesCell = arrayfun(@(x) feval([obj.MoveitMsgsRobotTrajectoryClass '.loadobj'], x), strObj.TrajectoryStages, 'UniformOutput', false);
            obj.TrajectoryStages = vertcat(TrajectoryStagesCell{:});
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
            
            strObj.TrajectoryDescriptions = obj.TrajectoryDescriptions;
            strObj.ErrorCode = saveobj(obj.ErrorCode);
            strObj.TrajectoryStart = saveobj(obj.TrajectoryStart);
            strObj.PlaceLocation = saveobj(obj.PlaceLocation);
            strObj.TrajectoryStages = arrayfun(@(x) saveobj(x), obj.TrajectoryStages);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.PlaceResult.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.PlaceResult;
            obj.reload(strObj);
        end
    end
end
