function [model, nmse] = quadraticfit2(X1, X2, Y, varargin)
% QUADRATICFIT2 Learns a quadratic model to fit input-output pairs 
% (X1, X2, Y):
% Y = X1'*R1*X1 + X2'*R2*X2 + 2*X1'*Rc*X2 + X1'*r1 + X2'*r2 + r0 
% (R1 is symmetric and negative definite). 
% For the likelihood of the linear model, a multiplicative noise model is 
% used (the higher the absolute value of Y, the higher the variance).
%
%    INPUT
%     - X1          : [d1 x N] matrix, where N is the number of samples
%     - X2          : [d2 x N] matrix, where N is the number of samples
%     - Y           : [1 x N] vector
%     - weights     : (optional) [1 x N] vector of samples weights
%     - standardize : (optional) flag to standardize X1, X2 and Y
%
%    OUTPUT
%     - model       : struct with fields R1, R2, Rc, r1, r2, r0, dim 
%                     (number of parameters of the model) and eval 
%                     (function for generating samples, i.e., 
%                     Y = model.eval(X))
%     - nmse        : normalized mean squared error on the training data

%% Parse and preprocess input
[d1, N] = size(X1);
[d2, N] = size(X2);

options = {'weights', 'standardize'};
defaults = {ones(1,N), 0};
[W, standardize] = internal.stats.parseArgs(options, defaults, varargin{:});

D = d1*(d1+1)/2 + d2*(d2+1)/2 + d1*d2 + d1 + d2 + 1; % Number of parameters of the quadratic model

if standardize
    [X1n, X1mu, X1std] = standardize_data(X1, W);
    [X2n, X2mu, X2std] = standardize_data(X2, W);
    [Yn, Ymu, Ystd] = standardize_data(Y, W);
else
    X1n = X1;
    X2n = X2;
    Yn = Y;
    X1mu = zeros(d1,1);
    X1std = ones(d1,1);
    X2mu = zeros(d2,1);
    X2std = ones(d2,1);
    Ymu = zeros(1,1);
    Ystd = ones(1,1);
end

%% Generate features vector for linear regression
Phi1 = basis_quadratic(d1,X1n); % Linear and quadratic features in X1
Phi2 = basis_quadratic(d2,X2n); % Linear and quadratic features in X2
Phi2(1,:) = []; % The bias is already in Phi1
PhiC = bsxfun(@times, X1n, permute(X2n,[3 2 1])); % Cross features
PhiC = permute(PhiC,[1 3 2]);
PhiC = 2 * reshape(PhiC,[d1*d2,N]);

Phi = [Phi1; Phi2; PhiC];

%% Weight each sample by its inverse of absoltute relative output
absY = abs( Yn - max(Yn) - 1e-2 * (max(Yn) - min(Yn)) ); % Small regularizer for 0 output
weight = 1 ./ absY;
if all(absY == absY(1)), weight(:) = 1; end % If all output are the same
W = W .* weight;

%% Get parameters by linear regression on pairs (Phi, Y) and extract model
params = linear_regression(Phi, Yn, W);
% params = bayesian_linear_regression(Phi, Yn, W);

assert(~any(isnan(params)), 'Model fitting failed.')

% r0 = params(1,:);
idx = 2;
% 
% r1 = params(idx:d1+idx-1);
idx = idx + d1;

R1 = params(idx:idx+d1*(d1+1)/2-1);
idx = idx + d1*(d1+1)/2;
AQuadratic = zeros(d1);
ind = logical(tril(ones(d1)));
AQuadratic(ind) = R1;
R1 = (AQuadratic + AQuadratic') / 2;

% r2 = params(idx:idx+d2-1);
% idx = idx + d2;
% 
% R2 = params(idx:idx+d2*(d2+1)/2-1);
% idx = idx+d2*(d2+1)/2;
% AQuadratic = zeros(d2);
% ind = logical(tril(ones(d2)));
% AQuadratic(ind) = R2;
% R2 = (AQuadratic + AQuadratic') / 2;
% 
% Rc = params(idx:end);
% Rc = reshape(Rc,d1,d2);

%% Enforce R1 to be negative definite
[U, V] = eig(R1);
V(V > 0) = -1e-8;
R1 = U * V * U';

% Re-learn the other components
quadYn = sum( (X1n'*R1)' .* X1n, 1 );
Phi1(2+d1:end,:) = []; % Remove quadratic features in X1
Phi = [Phi1; Phi2; PhiC];
params = linear_regression(Phi, Yn - quadYn, W);

r0 = params(1,:);
idx = 2;

r1 = params(idx:d1+idx-1);
idx = idx + d1;

r2 = params(idx:idx+d2-1);
idx = idx + d2;

R2 = params(idx:idx+d2*(d2+1)/2-1);
idx = idx+d2*(d2+1)/2;
AQuadratic = zeros(d2);
ind = logical(tril(ones(d2)));
AQuadratic(ind) = R2;
R2 = (AQuadratic + AQuadratic') / 2;

Rc = params(idx:end);
Rc = reshape(Rc,d1,d2);

%% De-standardize
X1stdMat = diag((1./X1std));
X2stdMat = diag((1./X2std));
YstdMat = diag(Ystd);
A1 = (X1stdMat * R1 * X1stdMat);
A2 = (X2stdMat * R2 * X2stdMat);
Ac = (X1stdMat * Rc * X2stdMat);
newR1 = YstdMat * A1;
newR2 = YstdMat * A2;
newRc = YstdMat * Ac;
newr1 = -YstdMat * (2*A1*X1mu + 2*Ac*X2mu - X1stdMat*r1);
newr2 = -YstdMat * (2*A2*X2mu + 2*Ac'*X1mu - X2stdMat*r2);
newr0 = YstdMat * (X1mu'*A1*X1mu - r1'*X1stdMat*X1mu + ...
    X2mu'*A2*X2mu - r2'*X2stdMat*X2mu + ...
    2 * X1mu'*Ac*X2mu + ...
    r0) + Ymu;
R1 = newR1;
R2 = newR2;
Rc = newRc;
r1 = newr1;
r2 = newr2;
r0 = newr0;

%% Save model
model.dim = D;
model.R1 = R1;
model.R2 = R2;
model.Rc = Rc;
model.r1 = r1;
model.r2 = r2;
model.r0 = r0;
model.eval = @(X1,X2) sum( (X1'*R1)' .* X1, 1 ) + ...
    2 * sum( (X1'*Rc)' .* X2, 1 ) + ...
    sum( (X2'*R2)' .* X2, 1 ) + ...
    (X1'*r1)' + ...
    (X2'*r2)' + ...
    r0';

nmse = mean( ( Y - model.eval(X1,X2) ).^2 ) / mean( Y.^2 );
