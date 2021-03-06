clear all
close all

%% ===================================================================== %%
%  ======================== LOW LEVEL SETTINGS =========================  %
mdp = Pong1P;
robj = 1;
dreward = mdp.dreward;
gamma = mdp.gamma;
nactions = mdp.actionUB;

bfs = @(varargin)basis_poly(1,mdp.dstate,0,varargin{:});

% policy = EGreedy(bfs, zeros((bfs()+1)*nactions,1), mdp.actionLB:mdp.actionUB, 0.1);
policy = Gibbs(bfs, zeros((bfs()+1)*(nactions-1),1), mdp.actionLB:mdp.actionUB);


%% ===================================================================== %%
%  ======================= HIGH LEVEL SETTINGS =========================  %
makeDet = 0; % 1 to learn deterministic low level policies
n_params = policy.dparams;
mu0 = policy.theta(1:n_params);
Sigma0high = 10 * eye(n_params);
Sigma0high = Sigma0high + diag(abs(mu0)).^2;
Sigma0high = nearestSPD(Sigma0high);
policy_high = GaussianConstantChol(n_params, mu0, Sigma0high);
% policy_high = GaussianConstantDiag(n_params, mu0, Sigma0high);
% policy_high = GmmConstant(mu0,Sigma0high,5);


%% ===================================================================== %%
%  ======================== LEARNING SETTINGS ==========================  %
episodes_eval = 100;
steps_eval = 10000;
episodes_learn = 50;
steps_learn = 10000;
