classdef ContactInformation < robotics.ros.Message
    %ContactInformation MATLAB implementation of moveit_msgs/ContactInformation
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'moveit_msgs/ContactInformation' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '116228ca08b0c286ec5ca32a50fdc17b' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant)
        ROBOTLINK = uint32(0)
        WORLDOBJECT = uint32(1)
        ROBOTATTACHED = uint32(2)
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsPointClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Point') % Dispatch to MATLAB class for message type geometry_msgs/Point
        GeometryMsgsVector3Class = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Vector3') % Dispatch to MATLAB class for message type geometry_msgs/Vector3
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        Position
        Normal
        Depth
        ContactBody1
        BodyType1
        ContactBody2
        BodyType2
    end
    
    properties (Access = protected)
        Cache = struct('Header', [], 'Position', [], 'Normal', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'BodyType1', 'BodyType2', 'ContactBody1', 'ContactBody2', 'Depth', 'Header', 'Normal', 'Position'} % List of non-constant message properties
        ROSPropertyList = {'body_type_1', 'body_type_2', 'contact_body_1', 'contact_body_2', 'depth', 'header', 'normal', 'position'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = ContactInformation(msg)
            %ContactInformation Construct the message object ContactInformation
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
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'ContactInformation', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function position = get.Position(obj)
            %get.Position Get the value for property Position
            if isempty(obj.Cache.Position)
                obj.Cache.Position = feval(obj.GeometryMsgsPointClass, obj.JavaMessage.getPosition);
            end
            position = obj.Cache.Position;
        end
        
        function set.Position(obj, position)
            %set.Position Set the value for property Position
            validateattributes(position, {obj.GeometryMsgsPointClass}, {'nonempty', 'scalar'}, 'ContactInformation', 'Position');
            
            obj.JavaMessage.setPosition(position.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Position)
                obj.Cache.Position.setJavaObject(position.getJavaObject);
            end
        end
        
        function normal = get.Normal(obj)
            %get.Normal Get the value for property Normal
            if isempty(obj.Cache.Normal)
                obj.Cache.Normal = feval(obj.GeometryMsgsVector3Class, obj.JavaMessage.getNormal);
            end
            normal = obj.Cache.Normal;
        end
        
        function set.Normal(obj, normal)
            %set.Normal Set the value for property Normal
            validateattributes(normal, {obj.GeometryMsgsVector3Class}, {'nonempty', 'scalar'}, 'ContactInformation', 'Normal');
            
            obj.JavaMessage.setNormal(normal.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Normal)
                obj.Cache.Normal.setJavaObject(normal.getJavaObject);
            end
        end
        
        function depth = get.Depth(obj)
            %get.Depth Get the value for property Depth
            depth = double(obj.JavaMessage.getDepth);
        end
        
        function set.Depth(obj, depth)
            %set.Depth Set the value for property Depth
            validateattributes(depth, {'numeric'}, {'nonempty', 'scalar'}, 'ContactInformation', 'Depth');
            
            obj.JavaMessage.setDepth(depth);
        end
        
        function contactbody1 = get.ContactBody1(obj)
            %get.ContactBody1 Get the value for property ContactBody1
            contactbody1 = char(obj.JavaMessage.getContactBody1);
        end
        
        function set.ContactBody1(obj, contactbody1)
            %set.ContactBody1 Set the value for property ContactBody1
            validateattributes(contactbody1, {'char'}, {}, 'ContactInformation', 'ContactBody1');
            
            obj.JavaMessage.setContactBody1(contactbody1);
        end
        
        function bodytype1 = get.BodyType1(obj)
            %get.BodyType1 Get the value for property BodyType1
            bodytype1 = typecast(int32(obj.JavaMessage.getBodyType1), 'uint32');
        end
        
        function set.BodyType1(obj, bodytype1)
            %set.BodyType1 Set the value for property BodyType1
            validateattributes(bodytype1, {'numeric'}, {'nonempty', 'scalar'}, 'ContactInformation', 'BodyType1');
            
            obj.JavaMessage.setBodyType1(bodytype1);
        end
        
        function contactbody2 = get.ContactBody2(obj)
            %get.ContactBody2 Get the value for property ContactBody2
            contactbody2 = char(obj.JavaMessage.getContactBody2);
        end
        
        function set.ContactBody2(obj, contactbody2)
            %set.ContactBody2 Set the value for property ContactBody2
            validateattributes(contactbody2, {'char'}, {}, 'ContactInformation', 'ContactBody2');
            
            obj.JavaMessage.setContactBody2(contactbody2);
        end
        
        function bodytype2 = get.BodyType2(obj)
            %get.BodyType2 Get the value for property BodyType2
            bodytype2 = typecast(int32(obj.JavaMessage.getBodyType2), 'uint32');
        end
        
        function set.BodyType2(obj, bodytype2)
            %set.BodyType2 Set the value for property BodyType2
            validateattributes(bodytype2, {'numeric'}, {'nonempty', 'scalar'}, 'ContactInformation', 'BodyType2');
            
            obj.JavaMessage.setBodyType2(bodytype2);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
            obj.Cache.Position = [];
            obj.Cache.Normal = [];
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
            cpObj.Depth = obj.Depth;
            cpObj.ContactBody1 = obj.ContactBody1;
            cpObj.BodyType1 = obj.BodyType1;
            cpObj.ContactBody2 = obj.ContactBody2;
            cpObj.BodyType2 = obj.BodyType2;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
            cpObj.Position = copy(obj.Position);
            cpObj.Normal = copy(obj.Normal);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Depth = strObj.Depth;
            obj.ContactBody1 = strObj.ContactBody1;
            obj.BodyType1 = strObj.BodyType1;
            obj.ContactBody2 = strObj.ContactBody2;
            obj.BodyType2 = strObj.BodyType2;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
            obj.Position = feval([obj.GeometryMsgsPointClass '.loadobj'], strObj.Position);
            obj.Normal = feval([obj.GeometryMsgsVector3Class '.loadobj'], strObj.Normal);
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
            
            strObj.Depth = obj.Depth;
            strObj.ContactBody1 = obj.ContactBody1;
            strObj.BodyType1 = obj.BodyType1;
            strObj.ContactBody2 = obj.ContactBody2;
            strObj.BodyType2 = obj.BodyType2;
            strObj.Header = saveobj(obj.Header);
            strObj.Position = saveobj(obj.Position);
            strObj.Normal = saveobj(obj.Normal);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.moveit_msgs.ContactInformation.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.moveit_msgs.ContactInformation;
            obj.reload(strObj);
        end
    end
end
