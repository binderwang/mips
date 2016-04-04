function [label, model, llh] = emgm(X, init, W)
% EMGM Weighted EM for fitting a Gaussian Mixture Model.
%
%    INPUT
%     - X     : [D x N] data matrix
%     - init  : scalar K or 
%               [1 x N] matrix of labels, where 1 <= label(i) <= K, or 
%               [D x K] centers or
%               model (struct)
%     - W     : (optional) [1 x N] weights vector (1 by default)
%
%    OUTPUT
%     - label : [1 x N] vector indicating which Gaussian is associated to
%               each sample
%     - model : struct with means (mu), covariances (Sigma) and component
%               proportions of the mixture model
%     - llh   : log-likelihood of the model
%
% =========================================================================
% ACKNOWLEDGEMENT
% http://www.mathworks.com/matlabcentral/fileexchange/26184-em-algorithm-for-gaussian-mixture-model

if nargin < 3
    W = ones(1,size(X,2));
else
    assert(min(W>=0), 'Weights cannot be negative.');
end

R = initialization(X, init);
[~, label(1,:)] = max(R, [], 2);
R = R(:, unique(label));

tol = 1e-10;
maxiter = 500;
llh = -inf(1, maxiter);
converged = false;
t = 1;

while ~converged && t < maxiter
    t = t + 1;
    model = maximization(X, R, W);
    [R, llh(t)] = expectation(X, model);
    
    [~, label(:)] = max(R, [], 2);
    u = unique(label);   % non-empty components
    if size(R, 2) ~= size(u, 2)
        R = R(:, u);     % remove empty components
    else
        converged = llh(t) - llh(t-1) < tol * abs(llh(t));
    end
end
llh = llh(2:t);

if ~converged
    warning('Not converged in %d steps.\n',maxiter);
end

end


%% Init
function R = initialization(X, init)

[d, n] = size(X);
if isstruct(init)         % initialize with a model
    R = expectation(X, init);
elseif length(init) == 1  % random initialization with k components
    idx = randsample(n,init);
    model.mu = X(:,idx);  % the means are k random points of the dataset
    model.ComponentProportion = init;
    model.Sigma = repmat(nearestSPD(cov(X')),1,1,init);  % the covariances are the covariance of the dataset
    R = expectation(X, model);
elseif size(init,1) == 1 && size(init,2) == n  % initialize with labels
    label = init;
    k = max(label);
    R = full(sparse(1:n,label,1,n,k,n));
elseif size(init,1) == d  % initialize with only centers
    k = size(init,2);
    m = init;
    [~, label] = max(bsxfun(@minus, m' * X, dot(m,m,1)' / 2), [], 1);
    R = full(sparse(1:n,label,1,n,k,n));
else
    error('Initialization is not valid.');
end

end


%% Expectation
function [R, llh] = expectation(X, model)

mu = model.mu;
Sigma = model.Sigma;
w = model.ComponentProportion;

n = size(X,2);
k = size(mu,2);
logRho = zeros(n,k);

for i = 1 : k
    logRho(:,i) = loggausspdf(X, mu(:,i), Sigma(:,:,i));
end

logRho = bsxfun(@plus, logRho, log(w));
T = logsumexp(logRho,2);
llh = sum(T) / n; % loglikelihood
logR = bsxfun(@minus, logRho, T);
R = exp(logR);

end


%% Maximization
function model = maximization(X, R, W)

d = size(X,1);
k = size(R,2);

W = W / sum(W);

w = bsxfun(@times, R, W');
model.ComponentProportion = sum(w,1) / sum(sum(w));

w = bsxfun(@times, w, 1 ./ sum(w,1));
mu = X * w;

Sigma = zeros(d,d,k);
for i = 1 : k
    Xo = bsxfun(@minus, X, mu(:,i));
    Sigma(:,:,i) = bsxfun(@times, Xo, w(:,i)') * Xo';
    Z = (sum(w(:,i))^2 - sum(w(:,i).^2)) / sum(w(:,i));
    Sigma(:,:,i) = Sigma(:,:,i) / Z;
    Sigma(:,:,i) = nearestSPD(Sigma(:,:,i));
end

model.mu = mu;
model.Sigma = Sigma;

end
