function g = sigmoid(z)
%SIGMOID Compute sigmoid functoon
%   J = SIGMOID(z) computes the sigmoid of z.

% You need to return the following variables correctly 
g = zeros(size(z));

% ====================== YOUR CODE HERE ======================
% Instructions: Compute the sigmoid of each value of z (z can be a matrix,
%               vector or scalar).

denominator = 1 + exp(-z); %exp(x) means e^x
g = 1 ./ denominator; %For a matrix, your function should perform the sigmoid function on every element.



% =============================================================

end
