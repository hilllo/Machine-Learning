function [J, grad] = costFunctionReg(theta, X, y, lambda)
%COSTFUNCTIONREG Compute cost and gradient for logistic regression with regularization
%   J = COSTFUNCTIONREG(theta, X, y, lambda) computes the cost of using
%   theta as the parameter for regularized logistic regression and the
%   gradient of the cost w.r.t. to the parameters. 

% Initialize some useful values
m = length(y); % number of training examples

% You need to return the following variables correctly 
J = 0;
grad = zeros(size(theta));

% ====================== YOUR CODE HERE ======================
% Instructions: Compute the cost of a particular choice of theta.
%               You should set J to the cost.
%               Compute the partial derivatives and set grad to the partial
%               derivatives of the cost w.r.t. each parameter in theta

z = X * theta; %X:(m,n+1) theta:(n+1,1)
h = sigmoid(z); %h:(m,1)

thetaLen = length(theta);
regTheta = theta(2:thetaLen,1);

%y:(m,1)
%-y .* log(h): each elements multiply, not multiply the matrix
J = 1 / m * sum(-y .* log(h) - (1 - y) .* log(1 - h)) + lambda / (2 * m) * sum (regTheta .^ 2); %number

grad(1,1) = 1 / m * sum ((h - y) .* X(:,1));

for i = 2:thetaLen
    %(h - y) .* X(:,i): each elements multiply, not multiply the matrix
    grad(i,1) = 1 / m * sum ((h - y) .* X(:,i)) + lambda / m * regTheta(i - 1,1);
end




% =============================================================

end
