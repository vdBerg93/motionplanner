classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties (Constant)
        car_msgs_Control = 'car_msgs/Control'
        car_msgs_MotionPlan = 'car_msgs/MotionPlan'
        car_msgs_Reference = 'car_msgs/Reference'
        car_msgs_State = 'car_msgs/State'
        car_msgs_Trajectory = 'car_msgs/Trajectory'
        car_msgs_getobstacles = 'car_msgs/getobstacles'
        car_msgs_getobstaclesRequest = 'car_msgs/getobstaclesRequest'
        car_msgs_getobstaclesResponse = 'car_msgs/getobstaclesResponse'
        car_msgs_planmotion = 'car_msgs/planmotion'
        car_msgs_planmotionRequest = 'car_msgs/planmotionRequest'
        car_msgs_planmotionResponse = 'car_msgs/planmotionResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(9, 1);
                msgList{1} = 'car_msgs/Control';
                msgList{2} = 'car_msgs/MotionPlan';
                msgList{3} = 'car_msgs/Reference';
                msgList{4} = 'car_msgs/State';
                msgList{5} = 'car_msgs/Trajectory';
                msgList{6} = 'car_msgs/getobstaclesRequest';
                msgList{7} = 'car_msgs/getobstaclesResponse';
                msgList{8} = 'car_msgs/planmotionRequest';
                msgList{9} = 'car_msgs/planmotionResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(2, 1);
                svcList{1} = 'car_msgs/getobstacles';
                svcList{2} = 'car_msgs/planmotion';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
