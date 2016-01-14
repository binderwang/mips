function K = kernel(X, Y, type, param)
% KERNEL Computes kernel function for matrices of points.
%
%    INPUT
%     - X     : [N x D] matrix (N samples, D dimension)
%     - Y     : [M x D] matrix (N samples, D dimension)
%     - type  : kernel name
%     - param : kernel parameters
%
%    OUTPUT
%     - K     : [N x M] kernel matrix

switch type
    
    case 'linear'
        K = X*Y';
        
    case 'poly'
        K = X*Y' + 1;
        K = K.^param;
        
    case 'gauss'
        dist = L2_distance(X',Y');
        K = exp(-dist.^2 ./ (2*param.^2));
        
    otherwise
        error('Unknown kernel.')
        
end

end
