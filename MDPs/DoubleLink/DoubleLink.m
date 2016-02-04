classdef DoubleLink < MDP
% REFERENCE
% T Yoshikawa
% Foundations of Robotics: Analysis and Control (1990)

    %% Properties
    properties
        % Environment variables
        lengths = [1 1];
        masses = [1 1];
        friction_coeff = [2.5 2.5]'; % viscous friction coefficients
        g = 9.81;
        dt = 0.01;
        q_des = [3/2*pi 0]';
        qd_des  = [0 0]';
        qdd_des = [0 0]';
        
        % MDP variables
        dstate = 4;
        daction = 2;
        dreward = 1;
        isAveraged = 0;
        gamma = 1;

        % Bounds : state = [theta(1) thetad(1) theta(2) thetad(2)]
        stateLB = [-pi, -50, -pi, -50]';
        stateUB = [pi, 50, pi, 50]';
        actionLB = [-10, -10]';
        actionUB = [10, 10]';
        rewardLB = -inf;
        rewardUB = 0;
    end
    
    methods
        %% Simulator
        function state = initstate(obj,n)
            state = repmat([pi 0 0 0]',1,n);
            if obj.realtimeplot, obj.showplot; obj.updateplot(state); end
        end
        
        function [nextstate, reward, absorb] = simulator(obj, state, action)
            % Check action
            real_action = bsxfun(@max, bsxfun(@min,action,obj.actionUB), obj.actionLB);
%             penalty_action = matrixnorms(real_action-action,2);
            action = real_action;

            % State transition
            [gravity, coriolis, invM, friction] = obj.getDynamicsMatrices(state);
            qdd = mtimes32(invM, action - coriolis - gravity - friction);
            qd = state(2:2:end,:) + obj.dt * qdd;
            q = state(1:2:end,:) + obj.dt * qd;

            % Check velocity 
%             real_vel = bsxfun(@max, bsxfun(@min,qd,obj.stateUB(2:2:end)), obj.stateLB(2:2:end));
%             penalty_vel = matrixnorms(real_vel-qd,2);
%             qd = real_vel;
            
            nextstate = [q(1,:); qd(1,:); q(2,:); qd(2,:)];

            % Compute reward
            goalstate = [obj.q_des(1), obj.qd_des(1), obj.q_des(2), obj.qd_des(2)]';
%             distance = bsxfun(@minus,q,obj.q_des);
            distance = bsxfun(@minus,nextstate,goalstate);
            reward = -matrixnorms(distance,2).^2;

            absorb = false(1,size(state,2)); % Infinite horizon

            if obj.realtimeplot, obj.updateplot(nextstate); end
        end
        
        %% Dynamics
        function [gravity, coriolis, invM, friction] = getDynamicsMatrices(obj, state)
            inertias = obj.masses .* (obj.lengths.^2 + 0.00001) ./ 3.0;
            m1 = obj.masses(1);
            m2 = obj.masses(2);
            l1 = obj.lengths(1);
            l2 = obj.lengths(2);
            lg1 = l1 / 2;
            lg2 = l2 / 2;
            q1 = state(1,:);
            q2 = state(3,:);
            q1d = state(2,:);
            q2d = state(4,:);
%             s1 = sin(q1);
            c1 = cos(q1);
            s2 = sin(q2);
            c2 = cos(q2);
%             s12 = sin(q1 + q2);
            c12 = cos(q1 + q2);
            
            M11 = m1 * lg1^2 + inertias(1) + m2 * (l1^2 + lg2^2 + 2 * l1 * lg2 * c2) + inertias(2);
            M12 = m2 * (lg2^2 + l1 * lg2 * c2) + inertias(2);
            M21 = M12;
            M22 = repmat(m2 * lg2^2 + inertias(2), 1, size(state,2));
            invdetM = 1 ./ (M11 .* M22 - M12 .* M21);
            invM(1,1,:) = M22;
            invM(1,2,:) = -M21;
            invM(2,1,:) = -M12;
            invM(2,2,:) = M11;
            invM = bsxfun(@times, invM, permute(invdetM, [3 1 2]));

            gravity = [m1 * obj.g * lg1 * c1 + m2 * obj.g * (l1 * c1 + lg2 * c12)
                m2 * obj.g * lg2 * c12];
            coriolis = [-m2 * l1 * lg2 * s2 .* (2 .* q1d .* q2d + q2d.^2)
                m2 * l1 * lg2 * s2 .* q1d.^2];
            friction = [obj.friction_coeff(1) * q1d
                obj.friction_coeff(2) * q2d];
        end

        %% Kinematics
        function X = getJointsInTaskSpace(obj, state)
            % Each column of X is a state in the task space (x1,y1,xd1,yd1,x2,y2,xd1,yd2)
            xy1 = obj.lengths(1) .* [sin(state(1,:)); cos(state(1,:))];
            xy2 = xy1 + obj.lengths(2) .* [sin(state(3,:)+state(1,:)); cos(state(3,:)+state(1,:))];
            
            xy1d = obj.lengths(1) .* ...
                [state(2,:).*cos(state(1,:)); ...
                -state(2,:).*sin(state(1,:))];
            xy2d = xy1d + obj.lengths(2) .* ...
                [(state(4,:)+state(2,:)).*cos(state(3,:)+state(1,:)); ...
                -(state(4,:)+state(2,:)).*sin(state(3,:)+state(1,:))];
            
            X = [xy1; xy1d; xy2; xy2d];
        end
        
    end
    
    %% Plotting
    methods(Hidden = true)
        
        function initplot(obj)
            obj.handleEnv = figure(); hold all
            
            line(sum(obj.lengths)*[-1.1 1.1], [0 0], 'LineStyle', '--');
            axis(sum(obj.lengths)*[-1.1 1.1 -1.1 1.1]);
            
            % Agent starting position
            initstate = obj.initstate(1);
            X = obj.getJointsInTaskSpace(initstate);
            xy1 = X(1:2,:);
            xy2 = X(5:6,:);
            
            % Agent handle
            lw = 4.0;
            r = 0.1;
            colors = {[0.1 0.1 0.4], [0.4 0.4 0.8]};
            obj.handleAgent{1} = line([0 xy1(1)], [0, xy1(2)], 'linewidth', lw, 'color', colors{1});
            obj.handleAgent{2} = line([xy1(1) xy2(1)], [xy1(2) xy2(2)], 'linewidth', lw, 'color', colors{2});
            %obj.handleAgent{3} = rectangle('Position',[0,0,2*r,2*r],'Curvature',[1,1],'FaceColor',colors{1});
            %obj.handleAgent{4} = rectangle('Position',[0,0,2*r,2*r],'Curvature',[1,1],'FaceColor',colors{2});
            obj.handleAgent{3} = rectangle('Position',[xy1(1)-r,xy1(2)-r,2*r,2*r],'Curvature',[1,1],'FaceColor',colors{1});
            obj.handleAgent{4} = rectangle('Position',[xy2(1)-r,xy2(2)-r,2*r,2*r],'Curvature',[1,1],'FaceColor',colors{2});
            % 'Ghost' desired state
            r = 0.1;
            goalstate = [obj.q_des(1), obj.qd_des(1), obj.q_des(2), obj.qd_des(2)]';
            X = obj.getJointsInTaskSpace(goalstate);
            xy1 = X(1:2,:);
            xy2 = X(5:6,:);

            % Ghost' desired state
            h = line([0 xy1(1)], [0, xy1(2)], 'linewidth', lw, 'color', 'r');
            h.Color(4) = 0.3;
            h = line([xy1(1) xy2(1)], [xy1(2) xy2(2)], 'linewidth', lw, 'color', 'r');
            h.Color(4) = 0.3;
            h = rectangle('Position',[xy1(1)-r,xy1(2)-r,2*r,2*r],'Curvature',[1,1],'FaceColor','r');
            h.EdgeColor(4) = 0.1;
            h.FaceColor(4) = 0.1;
            h = rectangle('Position',[xy2(1)-r,xy2(2)-r,2*r,2*r],'Curvature',[1,1],'FaceColor','r');
            h.EdgeColor(4) = 0.1;
            h.FaceColor(4) = 0.1;
        end
        
        function updateplot(obj, state)
            lw = 4.0;
            r = 0.1; 
            X = obj.getJointsInTaskSpace(state);
            xy1 = X(1:2,:);
            xy2 = X(5:6,:);
                 
            colors = {[0.1 0.1 0.4], [0.4 0.4 0.8]};
            h1 = line([0 xy1(1)], [0, xy1(2)], 'linewidth', lw, 'color', colors{1});
            h2 = line([xy1(1) xy2(1)], [xy1(2) xy2(2)], 'linewidth', lw, 'color', colors{2});
            h3 = rectangle('Position',[xy1(1)-r,xy1(2)-r,2*r,2*r],'Curvature',[1,1],'FaceColor',colors{1});
            h4 = rectangle('Position',[xy2(1)-r,xy2(2)-r,2*r,2*r],'Curvature',[1,1],'FaceColor',colors{2});
            
            delete(obj.handleAgent{1});
            delete(obj.handleAgent{2});
            delete(obj.handleAgent{3});
            delete(obj.handleAgent{4});
          
            obj.handleAgent{1} = h1;
            obj.handleAgent{2} = h2;
            obj.handleAgent{3} = h3;
            obj.handleAgent{4} = h4;
            
            %obj.handleAgent{1}.XData = [0 xy1(1)];
            %obj.handleAgent{1}.YData = [0 xy1(2)];
            %obj.handleAgent{2}.XData = [xy1(1) xy2(1)];
            %obj.handleAgent{2}.YData = [xy1(2) xy2(2)];
            %obj.handleAgent{3}.Position = [xy1(1)-r,xy1(2)-r,2*r,2*r];
            %obj.handleAgent{4}.Position = [xy2(1)-r,xy2(2)-r,2*r,2*r];

            %drawnow limitrate
            drawnow;
        end
        
    end
    
end