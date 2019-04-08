classdef TrajectoryConstraints < robotics.ros.Message
    %TrajectoryConstraints MATLAB implementation of moveit_msgs/TrajectoryConstraints
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/TrajectoryConstraints' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '461e1a732dfebb01e7d6c75d51a51eac' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        MoveitMsgsConstraintsClass = robotics.ros.msg.internal.MessageFactory.getClassForType('moveit_msgs/Constraints') % Dispatch to MATLAB class for message type moveit_msgs/Constraints
    end
    
    properties (Dependent)
        Constraints
    end
    
    properties (Access = protected)
        Cache = struct('Constraints', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Constraints'} % List of non-constant message properties
        ROSPropertyList = {'constraints'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = TrajectoryConstraints(msg)
            %TrajectoryConstraints Construct the message object TrajectoryConstraints
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
        
        function constraints = get.Constraints(obj)
            %get.Constraints Get the value for property Constraints
            if isempty(obj.Cache.Constraints)
                javaArray = obj.JavaMessage.getConstraints;
                array = obj.readJavaArray(javaArray, obj.MoveitMsgsConstraintsClass);
                obj.Cache.Constraints = feval(obj.MoveitMsgsConstraintsClass, array);
            end
            constraints = obj.Cache.Constraints;
        end
        
        function set.Constraints(obj, constraints)
            %set.Constraints Set the value for property Constraints
            if ~isvector(constraints) && isempty(constraints)
                % Allow empty [] input
                constraints = feval([obj.MoveitMsgsConstraintsClass '.empty'], 0, 1);
            end
            
            validateattributes(constraints, {obj.MoveitMsgsConstraintsClass}, {'vector'}, 'TrajectoryConstraints', 'Constraints');
            
            javaArray = obj.JavaMessage.getConstraints;
            array = obj.writeJavaArray(constraints, javaArray, obj.MoveitMsgsConstraintsClass);
            obj.JavaMessage.setConstraints(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Constraints)
                obj.Cache.Constraints = [];
                obj.Cache.Constraints = obj.Constraints;
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Constraints = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Recursively copy compound properties
            cpObj.Constraints = copy(obj.Constraints);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            ConstraintsCell = arrayfun(@(x) feval([obj.MoveitMsgsConstraintsClass '.loadobj'], x), strObj.Constraints, 'UniformOutput', false);
            obj.Constraints = vertcat(ConstraintsCell{:});
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
            
            strObj.Constraints = arrayfun(@(x) saveobj(x), obj.Constraints);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.TrajectoryConstraints.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.TrajectoryConstraints;
            obj.reload(strObj);
        end
    end
end