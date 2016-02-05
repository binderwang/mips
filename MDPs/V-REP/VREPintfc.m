classdef VREPintfc < MDP
    %% Properties
    properties
        % Environment variables
        vAREP              % api interface
        V_arm              % robot interface
        V_part             % set of handle to get the cartesian pose of a specific robot part
        controltype        % kind of controller : 'position' , 'velocity' , 'torque' 
        statecomposition   % statecomposition.jpos (bool) ; obj.statecomposition.jvel (bool) ; obj.statecomposition.cartpos (cartpos)
        componentstatedim  % vector of dimension one entry for each element that we want to measure
        extractpose        % function handle that elaborate a pose define as a rototraslation matrix R|t (4x4) 
        rewardfunction     % function handle for rewards (one or many) 
        startingstate      % vector of starting state 
        % set of the possible state that i can read
        stateset = ['position' , 'velocity' , 'cartpos'];
        
        % MDP variables
        isAveraged = 0;
        gamma = 0.9;

        % Bounds : state = [x xd theta thetad])
        %stateLB = [-2.4, -inf, -deg2rad(15), -inf]';
        %stateUB = [2.4, inf, deg2rad(15), inf]';
        %actionLB = 1;
        %actionUB = 2;
        %rewardLB = -1;
        %rewardUB = 0;
    end
    %% public methods
    methods
       
        function obj = VREPintfc(modelname,jointformat,part_names,controltype,statevec,...
                                extractpose,rewardfunction,startingstate)
          % vrep model
          obj.vAREP = VAREP('~');%,'nosyncronous');
          obj.V_arm = VAREP_arm(v,modelname,'fmt',jointformat);
          for i = 1:length(part_names)
             obj.V_part(i) = obj.vAREP.object(part_names(i));
          end
          obj.controltype = controltype;
          % set joint properties for control
          obj.daction = obj.V_arm.setactivejointsmode;
          % create structure for reading state and componentstatedim
          obj.statecomposition.jpos = false;
          obj.statecomposition.jvel = false;
          obj.statecomposition.cartpos = false;
          index = 1;
          for i = 1:length(statevec)
             if(strcmp(statevec(i),obj.stateset(1)))
                 obj.statecomposition.jpos = true;
                 obj.componentstatedim(index) = obj.dstate;
                 index = index + 1;
             elseif(strcmp(statevec(i),obj.stateset(2)))
                 obj.statecomposition.jvel = true;
                 obj.componentstatedim(index) = obj.dstate;
                 index = index + 1;
             elseif(strcmp(statevec(i),obj.stateset(3)))
                 obj.statecomposition.cartpos = true;
                 for j=1:length(obj.V_part)
                    obj.componentstatedim(index) = 3;
                    index = index + 1;
                 end
             end
          end
          % set state dimension
          obj.dstate = sum(obj.componentstatedim,1);
          % set the function to compute the pose of the object
          if(ishandle(extractpose) || isempty(extractpose))
             obj.extractpose = extractpose;
          else
             error('Error. \n extractpose must be an handle')
          end
          % set reward function and reward dimension
          if(ishandle(rewardfunction(1)))
             obj.rewardfunction = rewardfunction;
             obj.dreward = length(rewardfunction);
          else
            error('Error. \n rewardfunction must be an vector of handle')
          end
          % initialize 
          obj.startingstate = startingstate; 
        end
        % init state for simulator
        function state = initstate(obj,n)
           setq(obj.V_arm, obj.startingstate)
           % start simulation
            v.simstart();
           %if obj.realtimeplot, obj.showplot; obj.updateplot(state); end
        end
        % simulator
        function [nextstate, reward, absorb] = simulator(obj, state, action)
           % send control action
           obj.SendControl(action)
           % run one step of the simulator
           if(v.syncronous)
              obj.vAREP.SendTriggerSync()
              % update state
              nextstate = obj.GetState();
           end
           reward = feval(obj.rewardfunction,nexstate,action);
        end
    end
    methods (Access = private)
       function SendControl(obj,action)
          if(strcmp(obj.controltype,'position'))
            obj.V_arm.SetTargetQ(action)
          elseif(strcmp(obj.controltype,'velocity'))
            obj.V_arm.SetTargetQd(action);
          elseif(strcmp(obj.controltype,'torque'))
            obj.V_arm.SetTau(action);
          end
       end
       % get new state from simulation
       function state = GetState(obj)
         state = [];
         if(obj.statecomposition.jpos)
            q = obj.V_arm.getq();
            state = [state;q];
         elseif(obj.statecomposition.jvel)
            qd = obj.V_arm.GetQd();
            state = [state;qd];
         elseif(obj.statecomposition.cartpos)
            for i=1:length(obj.V_part)
               T = obj.V_part(i).getpose();
               state = [state;obj.extractpose(T)];
            end
         end
       end
       % decompose state
       function map = StateDecompose(obj,state)
         map = containers.Map;
         index = 1;
         i = 1;
         if(obj.statecomposition.jpos)
            map('position') = state(index:index + obj.componentstatedim(i));
            index = index + obj.componentstatedim(i);
            i = i + 1;
         elseif(obj.statecomposition.jvel)
            map('velocity') = state(index:index + obj.componentstatedim(i));
            index = index + obj.componentstatedim(i);
            i = i + 1;
         elseif(obj.statecomposition.cartpos)
            name = 'cartpos';
            for j=1:length(obj.V_part)
               map(strcat(name, num2str(j))) = state(index:index + obj.componentstatedim(i));
               index = index + obj.componentstatedim(i);
               i = i + 1;
            end
         end     
       end
    end
    
    
    %% Plotting
    
    
end