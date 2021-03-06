classdef IOStates < robotics.ros.Message
    %IOStates MATLAB implementation of ur_msgs/IOStates
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'ur_msgs/IOStates' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '0a5c7b73e3189e9a2caf8583d1bae2e2' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        UrMsgsAnalogClass = robotics.ros.msg.internal.MessageFactory.getClassForType('ur_msgs/Analog') % Dispatch to MATLAB class for message type ur_msgs/Analog
        UrMsgsDigitalClass = robotics.ros.msg.internal.MessageFactory.getClassForType('ur_msgs/Digital') % Dispatch to MATLAB class for message type ur_msgs/Digital
    end
    
    properties (Dependent)
        DigitalInStates
        DigitalOutStates
        FlagStates
        AnalogInStates
        AnalogOutStates
    end
    
    properties (Access = protected)
        Cache = struct('DigitalInStates', [], 'DigitalOutStates', [], 'FlagStates', [], 'AnalogInStates', [], 'AnalogOutStates', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'AnalogInStates', 'AnalogOutStates', 'DigitalInStates', 'DigitalOutStates', 'FlagStates'} % List of non-constant message properties
        ROSPropertyList = {'analog_in_states', 'analog_out_states', 'digital_in_states', 'digital_out_states', 'flag_states'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = IOStates(msg)
            %IOStates Construct the message object IOStates
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
        
        function digitalinstates = get.DigitalInStates(obj)
            %get.DigitalInStates Get the value for property DigitalInStates
            if isempty(obj.Cache.DigitalInStates)
                javaArray = obj.JavaMessage.getDigitalInStates;
                array = obj.readJavaArray(javaArray, obj.UrMsgsDigitalClass);
                obj.Cache.DigitalInStates = feval(obj.UrMsgsDigitalClass, array);
            end
            digitalinstates = obj.Cache.DigitalInStates;
        end
        
        function set.DigitalInStates(obj, digitalinstates)
            %set.DigitalInStates Set the value for property DigitalInStates
            if ~isvector(digitalinstates) && isempty(digitalinstates)
                % Allow empty [] input
                digitalinstates = feval([obj.UrMsgsDigitalClass '.empty'], 0, 1);
            end
            
            validateattributes(digitalinstates, {obj.UrMsgsDigitalClass}, {'vector'}, 'IOStates', 'DigitalInStates');
            
            javaArray = obj.JavaMessage.getDigitalInStates;
            array = obj.writeJavaArray(digitalinstates, javaArray, obj.UrMsgsDigitalClass);
            obj.JavaMessage.setDigitalInStates(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.DigitalInStates)
                obj.Cache.DigitalInStates = [];
                obj.Cache.DigitalInStates = obj.DigitalInStates;
            end
        end
        
        function digitaloutstates = get.DigitalOutStates(obj)
            %get.DigitalOutStates Get the value for property DigitalOutStates
            if isempty(obj.Cache.DigitalOutStates)
                javaArray = obj.JavaMessage.getDigitalOutStates;
                array = obj.readJavaArray(javaArray, obj.UrMsgsDigitalClass);
                obj.Cache.DigitalOutStates = feval(obj.UrMsgsDigitalClass, array);
            end
            digitaloutstates = obj.Cache.DigitalOutStates;
        end
        
        function set.DigitalOutStates(obj, digitaloutstates)
            %set.DigitalOutStates Set the value for property DigitalOutStates
            if ~isvector(digitaloutstates) && isempty(digitaloutstates)
                % Allow empty [] input
                digitaloutstates = feval([obj.UrMsgsDigitalClass '.empty'], 0, 1);
            end
            
            validateattributes(digitaloutstates, {obj.UrMsgsDigitalClass}, {'vector'}, 'IOStates', 'DigitalOutStates');
            
            javaArray = obj.JavaMessage.getDigitalOutStates;
            array = obj.writeJavaArray(digitaloutstates, javaArray, obj.UrMsgsDigitalClass);
            obj.JavaMessage.setDigitalOutStates(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.DigitalOutStates)
                obj.Cache.DigitalOutStates = [];
                obj.Cache.DigitalOutStates = obj.DigitalOutStates;
            end
        end
        
        function flagstates = get.FlagStates(obj)
            %get.FlagStates Get the value for property FlagStates
            if isempty(obj.Cache.FlagStates)
                javaArray = obj.JavaMessage.getFlagStates;
                array = obj.readJavaArray(javaArray, obj.UrMsgsDigitalClass);
                obj.Cache.FlagStates = feval(obj.UrMsgsDigitalClass, array);
            end
            flagstates = obj.Cache.FlagStates;
        end
        
        function set.FlagStates(obj, flagstates)
            %set.FlagStates Set the value for property FlagStates
            if ~isvector(flagstates) && isempty(flagstates)
                % Allow empty [] input
                flagstates = feval([obj.UrMsgsDigitalClass '.empty'], 0, 1);
            end
            
            validateattributes(flagstates, {obj.UrMsgsDigitalClass}, {'vector'}, 'IOStates', 'FlagStates');
            
            javaArray = obj.JavaMessage.getFlagStates;
            array = obj.writeJavaArray(flagstates, javaArray, obj.UrMsgsDigitalClass);
            obj.JavaMessage.setFlagStates(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.FlagStates)
                obj.Cache.FlagStates = [];
                obj.Cache.FlagStates = obj.FlagStates;
            end
        end
        
        function analoginstates = get.AnalogInStates(obj)
            %get.AnalogInStates Get the value for property AnalogInStates
            if isempty(obj.Cache.AnalogInStates)
                javaArray = obj.JavaMessage.getAnalogInStates;
                array = obj.readJavaArray(javaArray, obj.UrMsgsAnalogClass);
                obj.Cache.AnalogInStates = feval(obj.UrMsgsAnalogClass, array);
            end
            analoginstates = obj.Cache.AnalogInStates;
        end
        
        function set.AnalogInStates(obj, analoginstates)
            %set.AnalogInStates Set the value for property AnalogInStates
            if ~isvector(analoginstates) && isempty(analoginstates)
                % Allow empty [] input
                analoginstates = feval([obj.UrMsgsAnalogClass '.empty'], 0, 1);
            end
            
            validateattributes(analoginstates, {obj.UrMsgsAnalogClass}, {'vector'}, 'IOStates', 'AnalogInStates');
            
            javaArray = obj.JavaMessage.getAnalogInStates;
            array = obj.writeJavaArray(analoginstates, javaArray, obj.UrMsgsAnalogClass);
            obj.JavaMessage.setAnalogInStates(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.AnalogInStates)
                obj.Cache.AnalogInStates = [];
                obj.Cache.AnalogInStates = obj.AnalogInStates;
            end
        end
        
        function analogoutstates = get.AnalogOutStates(obj)
            %get.AnalogOutStates Get the value for property AnalogOutStates
            if isempty(obj.Cache.AnalogOutStates)
                javaArray = obj.JavaMessage.getAnalogOutStates;
                array = obj.readJavaArray(javaArray, obj.UrMsgsAnalogClass);
                obj.Cache.AnalogOutStates = feval(obj.UrMsgsAnalogClass, array);
            end
            analogoutstates = obj.Cache.AnalogOutStates;
        end
        
        function set.AnalogOutStates(obj, analogoutstates)
            %set.AnalogOutStates Set the value for property AnalogOutStates
            if ~isvector(analogoutstates) && isempty(analogoutstates)
                % Allow empty [] input
                analogoutstates = feval([obj.UrMsgsAnalogClass '.empty'], 0, 1);
            end
            
            validateattributes(analogoutstates, {obj.UrMsgsAnalogClass}, {'vector'}, 'IOStates', 'AnalogOutStates');
            
            javaArray = obj.JavaMessage.getAnalogOutStates;
            array = obj.writeJavaArray(analogoutstates, javaArray, obj.UrMsgsAnalogClass);
            obj.JavaMessage.setAnalogOutStates(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.AnalogOutStates)
                obj.Cache.AnalogOutStates = [];
                obj.Cache.AnalogOutStates = obj.AnalogOutStates;
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.DigitalInStates = [];
            obj.Cache.DigitalOutStates = [];
            obj.Cache.FlagStates = [];
            obj.Cache.AnalogInStates = [];
            obj.Cache.AnalogOutStates = [];
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
            cpObj.DigitalInStates = copy(obj.DigitalInStates);
            cpObj.DigitalOutStates = copy(obj.DigitalOutStates);
            cpObj.FlagStates = copy(obj.FlagStates);
            cpObj.AnalogInStates = copy(obj.AnalogInStates);
            cpObj.AnalogOutStates = copy(obj.AnalogOutStates);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            DigitalInStatesCell = arrayfun(@(x) feval([obj.UrMsgsDigitalClass '.loadobj'], x), strObj.DigitalInStates, 'UniformOutput', false);
            obj.DigitalInStates = vertcat(DigitalInStatesCell{:});
            DigitalOutStatesCell = arrayfun(@(x) feval([obj.UrMsgsDigitalClass '.loadobj'], x), strObj.DigitalOutStates, 'UniformOutput', false);
            obj.DigitalOutStates = vertcat(DigitalOutStatesCell{:});
            FlagStatesCell = arrayfun(@(x) feval([obj.UrMsgsDigitalClass '.loadobj'], x), strObj.FlagStates, 'UniformOutput', false);
            obj.FlagStates = vertcat(FlagStatesCell{:});
            AnalogInStatesCell = arrayfun(@(x) feval([obj.UrMsgsAnalogClass '.loadobj'], x), strObj.AnalogInStates, 'UniformOutput', false);
            obj.AnalogInStates = vertcat(AnalogInStatesCell{:});
            AnalogOutStatesCell = arrayfun(@(x) feval([obj.UrMsgsAnalogClass '.loadobj'], x), strObj.AnalogOutStates, 'UniformOutput', false);
            obj.AnalogOutStates = vertcat(AnalogOutStatesCell{:});
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
            
            strObj.DigitalInStates = arrayfun(@(x) saveobj(x), obj.DigitalInStates);
            strObj.DigitalOutStates = arrayfun(@(x) saveobj(x), obj.DigitalOutStates);
            strObj.FlagStates = arrayfun(@(x) saveobj(x), obj.FlagStates);
            strObj.AnalogInStates = arrayfun(@(x) saveobj(x), obj.AnalogInStates);
            strObj.AnalogOutStates = arrayfun(@(x) saveobj(x), obj.AnalogOutStates);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.ur_msgs.IOStates.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.ur_msgs.IOStates;
            obj.reload(strObj);
        end
    end
end
