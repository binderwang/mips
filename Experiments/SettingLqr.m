clear all
close all

%% ===================================================================== %%
%  ======================== LOW LEVEL SETTINGS =========================  %
dim = 5;
mdp = LQR(dim);
robj = 1;
dreward = mdp.dreward;
gamma = mdp.gamma;
daction = mdp.daction;

bfs = @(varargin)basis_poly(1,dim,0,varargin{:});

A0 = zeros(dim,bfs()+1);
Sigma0 = eye(dim);
w0 = ones(dim,1);
tau = ones(dim,1);
% policy = GaussianLinearChol(bfs, dim, A0, Sigma0);
% policy = GaussianLinearDiag(bfs, dim, A0, Sigma0);
% policy = GaussianLinearLogistic(bfs, dim, A0, w0, tau);
% policy = GaussianLinearFixedvar(bfs, dim, A0, Sigma0);
% policy = GaussianLinearFixedvarDiagmean(bfs, dim, A0(:,2:end), Sigma0);
policy = GaussianLinearFixedvarDiagmean(bfs, dim, -0.5*eye(dim), Sigma0);


%% ===================================================================== %%
%  ======================= HIGH LEVEL SETTINGS =========================  %
makeDet = 0; % 1 to learn deterministic low level policies
n_params = policy.dparams*~makeDet + numel(A0)*makeDet;
mu0 = policy.theta(1:n_params);
Sigma0high = 0.1 * eye(n_params);
tau = diag(Sigma0high);
% policy_high = GaussianConstantChol(n_params, mu0, Sigma0high);
policy_high = GaussianConstantDiag(n_params, mu0, Sigma0high);


%% ===================================================================== %%
%  ======================== LEARNING SETTINGS ==========================  %
episodes_eval = 150;
steps_eval = 50;
episodes_learn = 50;
steps_learn = 50;
