classdef PlannerParams < robotics.ros.Message
    %PlannerParams MATLAB implementation of moveit_msgs/PlannerParams
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/PlannerParams' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'cebdf4927996b9026bcf59a160d64145' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Keys
        Values
        Descriptions
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Descriptions', 'Keys', 'Values'} % List of non-constant message properties
        ROSPropertyList = {'descriptions', 'keys', 'values'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = PlannerParams(msg)
            %PlannerParams Construct the message object PlannerParams
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
        
        function keys = get.Keys(obj)
            %get.Keys Get the value for property Keys
            javaArray = obj.JavaMessage.getKeys;
            array = obj.readJavaArray(javaArray, 'char');
            keys = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.Keys(obj, keys)
            %set.Keys Set the value for property Keys
            if ~isvector(keys) && isempty(keys)
                % Allow empty [] input
                keys = cell.empty(0,1);
            end
            
            validateattributes(keys, {'cell'}, {'vector'}, 'PlannerParams', 'Keys');
            if any(cellfun(@(x) ~ischar(x), keys))
                error(message('robotics:ros:message:CellArrayStringError', ...
                    'keys'));
            end
            
            javaArray = obj.JavaMessage.getKeys;
            array = obj.writeJavaArray(keys, javaArray, 'char');
            obj.JavaMessage.setKeys(array);
        end
        
        function values = get.Values(obj)
            %get.Values Get the value for property Values
            javaArray = obj.JavaMessage.getValues;
            array = obj.readJavaArray(javaArray, 'char');
            values = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.Values(obj, values)
            %set.Values Set the value for property Values
            if ~isvector(values) && isempty(values)
                % Allow empty [] input
                values = cell.empty(0,1);
            end
            
            validateattributes(values, {'cell'}, {'vector'}, 'PlannerParams', 'Values');
            if any(cellfun(@(x) ~ischar(x), values))
                error(message('robotics:ros:message:CellArrayStringError', ...
                    'values'));
            end
            
            javaArray = obj.JavaMessage.getValues;
            array = obj.writeJavaArray(values, javaArray, 'char');
            obj.JavaMessage.setValues(array);
        end
        
        function descriptions = get.Descriptions(obj)
            %get.Descriptions Get the value for property Descriptions
            javaArray = obj.JavaMessage.getDescriptions;
            array = obj.readJavaArray(javaArray, 'char');
            descriptions = arrayfun(@(x) char(x), array, 'UniformOutput', false);
        end
        
        function set.Descriptions(obj, descriptions)
            %set.Descriptions Set the value for property Descriptions
            if ~isvector(descriptions) && isempty(descriptions)
                % Allow empty [] input
                descriptions = cell.empty(0,1);
            end
            
            validateattributes(descriptions, {'cell'}, {'vector'}, 'PlannerParams', 'Descriptions');
            if any(cellfun(@(x) ~ischar(x), descriptions))
                error(message('robotics:ros:message:CellArrayStringError', ...
                    'descriptions'));
            end
            
            javaArray = obj.JavaMessage.getDescriptions;
            array = obj.writeJavaArray(descriptions, javaArray, 'char');
            obj.JavaMessage.setDescriptions(array);
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
            cpObj.Keys = obj.Keys;
            cpObj.Values = obj.Values;
            cpObj.Descriptions = obj.Descriptions;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Keys = strObj.Keys;
            obj.Values = strObj.Values;
            obj.Descriptions = strObj.Descriptions;
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
            
            strObj.Keys = obj.Keys;
            strObj.Values = obj.Values;
            strObj.Descriptions = obj.Descriptions;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.PlannerParams.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.PlannerParams;
            obj.reload(strObj);
        end
    end
end
