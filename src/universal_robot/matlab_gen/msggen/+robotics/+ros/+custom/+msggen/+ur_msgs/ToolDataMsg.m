classdef ToolDataMsg < robotics.ros.Message
    %ToolDataMsg MATLAB implementation of ur_msgs/ToolDataMsg
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'ur_msgs/ToolDataMsg' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '404fc266f37d89f75b372d12fa94a122' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant)
        ANALOGINPUTRANGECURRENT = int8(0)
        ANALOGINPUTRANGEVOLTAGE = int8(1)
        TOOLBOOTLOADERMODE = uint8(249)
        TOOLRUNNINGMODE = uint8(253)
        TOOLIDLEMODE = uint8(255)
    end
    
    properties (Dependent)
        AnalogInputRange2
        AnalogInputRange3
        AnalogInput2
        AnalogInput3
        ToolVoltage48v
        ToolOutputVoltage
        ToolCurrent
        ToolTemperature
        ToolMode
    end
    
    properties (Constant, Hidden)
        PropertyList = {'AnalogInput2', 'AnalogInput3', 'AnalogInputRange2', 'AnalogInputRange3', 'ToolCurrent', 'ToolMode', 'ToolOutputVoltage', 'ToolTemperature', 'ToolVoltage48v'} % List of non-constant message properties
        ROSPropertyList = {'analog_input2', 'analog_input3', 'analog_input_range2', 'analog_input_range3', 'tool_current', 'tool_mode', 'tool_output_voltage', 'tool_temperature', 'tool_voltage_48v'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = ToolDataMsg(msg)
            %ToolDataMsg Construct the message object ToolDataMsg
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
        
        function analoginputrange2 = get.AnalogInputRange2(obj)
            %get.AnalogInputRange2 Get the value for property AnalogInputRange2
            analoginputrange2 = int8(obj.JavaMessage.getAnalogInputRange2);
        end
        
        function set.AnalogInputRange2(obj, analoginputrange2)
            %set.AnalogInputRange2 Set the value for property AnalogInputRange2
            validateattributes(analoginputrange2, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'AnalogInputRange2');
            
            obj.JavaMessage.setAnalogInputRange2(analoginputrange2);
        end
        
        function analoginputrange3 = get.AnalogInputRange3(obj)
            %get.AnalogInputRange3 Get the value for property AnalogInputRange3
            analoginputrange3 = int8(obj.JavaMessage.getAnalogInputRange3);
        end
        
        function set.AnalogInputRange3(obj, analoginputrange3)
            %set.AnalogInputRange3 Set the value for property AnalogInputRange3
            validateattributes(analoginputrange3, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'AnalogInputRange3');
            
            obj.JavaMessage.setAnalogInputRange3(analoginputrange3);
        end
        
        function analoginput2 = get.AnalogInput2(obj)
            %get.AnalogInput2 Get the value for property AnalogInput2
            analoginput2 = double(obj.JavaMessage.getAnalogInput2);
        end
        
        function set.AnalogInput2(obj, analoginput2)
            %set.AnalogInput2 Set the value for property AnalogInput2
            validateattributes(analoginput2, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'AnalogInput2');
            
            obj.JavaMessage.setAnalogInput2(analoginput2);
        end
        
        function analoginput3 = get.AnalogInput3(obj)
            %get.AnalogInput3 Get the value for property AnalogInput3
            analoginput3 = double(obj.JavaMessage.getAnalogInput3);
        end
        
        function set.AnalogInput3(obj, analoginput3)
            %set.AnalogInput3 Set the value for property AnalogInput3
            validateattributes(analoginput3, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'AnalogInput3');
            
            obj.JavaMessage.setAnalogInput3(analoginput3);
        end
        
        function toolvoltage48v = get.ToolVoltage48v(obj)
            %get.ToolVoltage48v Get the value for property ToolVoltage48v
            toolvoltage48v = single(obj.JavaMessage.getToolVoltage48v);
        end
        
        function set.ToolVoltage48v(obj, toolvoltage48v)
            %set.ToolVoltage48v Set the value for property ToolVoltage48v
            validateattributes(toolvoltage48v, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'ToolVoltage48v');
            
            obj.JavaMessage.setToolVoltage48v(toolvoltage48v);
        end
        
        function tooloutputvoltage = get.ToolOutputVoltage(obj)
            %get.ToolOutputVoltage Get the value for property ToolOutputVoltage
            tooloutputvoltage = typecast(int8(obj.JavaMessage.getToolOutputVoltage), 'uint8');
        end
        
        function set.ToolOutputVoltage(obj, tooloutputvoltage)
            %set.ToolOutputVoltage Set the value for property ToolOutputVoltage
            validateattributes(tooloutputvoltage, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'ToolOutputVoltage');
            
            obj.JavaMessage.setToolOutputVoltage(tooloutputvoltage);
        end
        
        function toolcurrent = get.ToolCurrent(obj)
            %get.ToolCurrent Get the value for property ToolCurrent
            toolcurrent = single(obj.JavaMessage.getToolCurrent);
        end
        
        function set.ToolCurrent(obj, toolcurrent)
            %set.ToolCurrent Set the value for property ToolCurrent
            validateattributes(toolcurrent, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'ToolCurrent');
            
            obj.JavaMessage.setToolCurrent(toolcurrent);
        end
        
        function tooltemperature = get.ToolTemperature(obj)
            %get.ToolTemperature Get the value for property ToolTemperature
            tooltemperature = single(obj.JavaMessage.getToolTemperature);
        end
        
        function set.ToolTemperature(obj, tooltemperature)
            %set.ToolTemperature Set the value for property ToolTemperature
            validateattributes(tooltemperature, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'ToolTemperature');
            
            obj.JavaMessage.setToolTemperature(tooltemperature);
        end
        
        function toolmode = get.ToolMode(obj)
            %get.ToolMode Get the value for property ToolMode
            toolmode = typecast(int8(obj.JavaMessage.getToolMode), 'uint8');
        end
        
        function set.ToolMode(obj, toolmode)
            %set.ToolMode Set the value for property ToolMode
            validateattributes(toolmode, {'numeric'}, {'nonempty', 'scalar'}, 'ToolDataMsg', 'ToolMode');
            
            obj.JavaMessage.setToolMode(toolmode);
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
            cpObj.AnalogInputRange2 = obj.AnalogInputRange2;
            cpObj.AnalogInputRange3 = obj.AnalogInputRange3;
            cpObj.AnalogInput2 = obj.AnalogInput2;
            cpObj.AnalogInput3 = obj.AnalogInput3;
            cpObj.ToolVoltage48v = obj.ToolVoltage48v;
            cpObj.ToolOutputVoltage = obj.ToolOutputVoltage;
            cpObj.ToolCurrent = obj.ToolCurrent;
            cpObj.ToolTemperature = obj.ToolTemperature;
            cpObj.ToolMode = obj.ToolMode;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.AnalogInputRange2 = strObj.AnalogInputRange2;
            obj.AnalogInputRange3 = strObj.AnalogInputRange3;
            obj.AnalogInput2 = strObj.AnalogInput2;
            obj.AnalogInput3 = strObj.AnalogInput3;
            obj.ToolVoltage48v = strObj.ToolVoltage48v;
            obj.ToolOutputVoltage = strObj.ToolOutputVoltage;
            obj.ToolCurrent = strObj.ToolCurrent;
            obj.ToolTemperature = strObj.ToolTemperature;
            obj.ToolMode = strObj.ToolMode;
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
            
            strObj.AnalogInputRange2 = obj.AnalogInputRange2;
            strObj.AnalogInputRange3 = obj.AnalogInputRange3;
            strObj.AnalogInput2 = obj.AnalogInput2;
            strObj.AnalogInput3 = obj.AnalogInput3;
            strObj.ToolVoltage48v = obj.ToolVoltage48v;
            strObj.ToolOutputVoltage = obj.ToolOutputVoltage;
            strObj.ToolCurrent = obj.ToolCurrent;
            strObj.ToolTemperature = obj.ToolTemperature;
            strObj.ToolMode = obj.ToolMode;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.ur_msgs.ToolDataMsg.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.ur_msgs.ToolDataMsg;
            obj.reload(strObj);
        end
    end
end
