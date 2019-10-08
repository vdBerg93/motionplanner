classdef Reference < robotics.ros.Message
    %Reference MATLAB implementation of car_msgs/Reference
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'car_msgs/Reference' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'f97866a7b7246dda074866e8a1b4c24d' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Dir
        X
        Y
        V
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Dir', 'V', 'X', 'Y'} % List of non-constant message properties
        ROSPropertyList = {'dir', 'v', 'x', 'y'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = Reference(msg)
            %Reference Construct the message object Reference
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
        
        function dir = get.Dir(obj)
            %get.Dir Get the value for property Dir
            dir = int32(obj.JavaMessage.getDir);
        end
        
        function set.Dir(obj, dir)
            %set.Dir Set the value for property Dir
            validateattributes(dir, {'numeric'}, {'nonempty', 'scalar'}, 'Reference', 'Dir');
            
            obj.JavaMessage.setDir(dir);
        end
        
        function x = get.X(obj)
            %get.X Get the value for property X
            javaArray = obj.JavaMessage.getX;
            array = obj.readJavaArray(javaArray, 'double');
            x = double(array);
        end
        
        function set.X(obj, x)
            %set.X Set the value for property X
            if ~isvector(x) && isempty(x)
                % Allow empty [] input
                x = double.empty(0,1);
            end
            
            validateattributes(x, {'numeric'}, {'vector'}, 'Reference', 'X');
            
            javaArray = obj.JavaMessage.getX;
            array = obj.writeJavaArray(x, javaArray, 'double');
            obj.JavaMessage.setX(array);
        end
        
        function y = get.Y(obj)
            %get.Y Get the value for property Y
            javaArray = obj.JavaMessage.getY;
            array = obj.readJavaArray(javaArray, 'double');
            y = double(array);
        end
        
        function set.Y(obj, y)
            %set.Y Set the value for property Y
            if ~isvector(y) && isempty(y)
                % Allow empty [] input
                y = double.empty(0,1);
            end
            
            validateattributes(y, {'numeric'}, {'vector'}, 'Reference', 'Y');
            
            javaArray = obj.JavaMessage.getY;
            array = obj.writeJavaArray(y, javaArray, 'double');
            obj.JavaMessage.setY(array);
        end
        
        function v = get.V(obj)
            %get.V Get the value for property V
            javaArray = obj.JavaMessage.getV;
            array = obj.readJavaArray(javaArray, 'double');
            v = double(array);
        end
        
        function set.V(obj, v)
            %set.V Set the value for property V
            if ~isvector(v) && isempty(v)
                % Allow empty [] input
                v = double.empty(0,1);
            end
            
            validateattributes(v, {'numeric'}, {'vector'}, 'Reference', 'V');
            
            javaArray = obj.JavaMessage.getV;
            array = obj.writeJavaArray(v, javaArray, 'double');
            obj.JavaMessage.setV(array);
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
            cpObj.Dir = obj.Dir;
            cpObj.X = obj.X;
            cpObj.Y = obj.Y;
            cpObj.V = obj.V;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Dir = strObj.Dir;
            obj.X = strObj.X;
            obj.Y = strObj.Y;
            obj.V = strObj.V;
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
            
            strObj.Dir = obj.Dir;
            strObj.X = obj.X;
            strObj.Y = obj.Y;
            strObj.V = obj.V;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.car_msgs.Reference.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.car_msgs.Reference;
            obj.reload(strObj);
        end
    end
end
