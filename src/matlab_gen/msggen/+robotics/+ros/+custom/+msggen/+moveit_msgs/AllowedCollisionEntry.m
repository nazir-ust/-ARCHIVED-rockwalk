classdef AllowedCollisionEntry < robotics.ros.Message
    %AllowedCollisionEntry MATLAB implementation of moveit_msgs/AllowedCollisionEntry
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/AllowedCollisionEntry' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '90d1ae1850840724bb043562fe3285fc' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Enabled
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Enabled'} % List of non-constant message properties
        ROSPropertyList = {'enabled'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = AllowedCollisionEntry(msg)
            %AllowedCollisionEntry Construct the message object AllowedCollisionEntry
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
        
        function enabled = get.Enabled(obj)
            %get.Enabled Get the value for property Enabled
            javaArray = obj.JavaMessage.getEnabled;
            array = obj.readJavaArray(javaArray, 'logical');
            enabled = logical(array);
        end
        
        function set.Enabled(obj, enabled)
            %set.Enabled Set the value for property Enabled
            if ~isvector(enabled) && isempty(enabled)
                % Allow empty [] input
                enabled = logical.empty(0,1);
            end
            
            validateattributes(enabled, {'logical', 'numeric'}, {'vector'}, 'AllowedCollisionEntry', 'Enabled');
            
            javaArray = obj.JavaMessage.getEnabled;
            array = obj.writeJavaArray(enabled, javaArray, 'logical');
            obj.JavaMessage.setEnabled(array);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Enabled = obj.Enabled;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Enabled = strObj.Enabled;
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
            
            strObj.Enabled = obj.Enabled;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.AllowedCollisionEntry.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.AllowedCollisionEntry;
            obj.reload(strObj);
        end
    end
end
