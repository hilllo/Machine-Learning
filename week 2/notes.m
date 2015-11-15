a = pi; %';'???????????
disp(sprintf('2 decimals: %0.2f',a))    

A = [1 2; 3 4; 5 6]; 

v = 1:0.1:2 %for( v = 1 ; v < 2 ; v + 0.1)

ones(2,3)   %create a all-one 2*3 matirx 
zeros(1,3)  %create a all-zero 1*3 matirx
rand(3,3)   %create a 3*3 randomly

w = -6 + sqrt(10)*(randn(1,10000));
hist(w) %create a graph

eye(4)  %create a diagnal-one 4*4 matrix



sz = size(A) %return dimension 
size(A,1)   %return the 1st dimension

length([1;2;3;4;5]) %return a vetor's length

pwd     %show current path
%cd 'C:\Users\xiaoshal' %goto a path
%ls      %list directory on your desktop
%load featuresX.dat %load file on current path
load ex1data1.txt
who  %variables in the current scope
whos %details
%clear featureX %delete feature featureX
%v = priceY(1:10)   %v?priceY?10???
%save hello.mat v   %save variable v to file called hello.mat
%clear  %??????

A(3,2)  %row 3, column 2
A(2,:)  %":" means every element along that row/column
A([1 3],:) %all elements on 1st & 3rd rows
A(:,2) = [10;11;12] %assign 10,11,12 to the 2nd column
A = [A, [100;101;102]] %add a 3rd column [100;101;102]. A becomes 3*3 matrix
A(:) %put all elements of A into a single vector
A = [1 2; 3 4; 5 6];
B = [11 12; 13 14; 15 16];
C = [A B] %A becomes a left matrix and B becomes the right of 4*4 new matrix C
C = [A;B] %A becomes a upper matrix and B becomes the lower of 4*4 new matrix C

C = [1 1; 2 2];
A*C
A .* B %mutiple each elements (get 2*2 matrix)
A .^ 2 %each elements square
v = [1;2;3]
1 ./ v %1/each elements
log(v)
abs(v) %???
-v  %negative v
v + ones(length(v) , 1) % each elements + 1,  == v+1

A'  %transform
a = [1 15 2 0.5] %vector
val = max(a)
[val, ind] = max(a) %val is the max element, ind is its column(index)
a < 3 %return a vector show whether the element<3 (1) or not (0)

A = magic(3) %rows and coulmns and diagnoes sum up to the same thing
[r,c] = find (A>=7) %the [ri,ci] element that >= 7

sum(a) %add all elements
prod(a) %times elemnts all
floor(a) %rounds down eg.0.5 = 0
ceil(a) %seal up eg.0.5 = 1

A=magic(3)
max(A,[],1) %find max element of each column of A
max(A,[],2) %find max element of each row of A
A(:) %turns into vector
max(A(:)) %max elements in A, == max(max(A))

A=magic(9)
sum(A,1) %sum each coulmns
sum(A,2) %sum each rows
sum(sum(A.*eye(9))) %add up 1 diagnal elements
flipud(eye(9))  %flip

A = magir(3)
pinv(A) %inverse of A

t = [0:0.01:0.98];
y1 = sin(2*pi*4*t);
plot(t,y1); %draw a function curves of y1 = f(t)
y2 = cos(2*pi*4*t);
hold on; %draw next curve on the same figure
plot(t,y2,'r');
xlabel('time'); %lable axis x
ylabel('value'); %lable axis y
legend('sin','cos'); %named your curve
title('my plot');
cd 'C:\Users\xiaoshal\MOOC\Machine-Learning\week 2'; print -dpng 'myPlot.png' %save figure as graph
close %close the figure
figure(1);plot(t,y1);
figure(2);plot(t,y2); %create diff figure

subplot(1,2,1) %divides plot a 1*2 grid, access 1st element
plot(t,y1)
subplot(1,2,2)
plot(t,y2)
axis([0.5 1 -1 1]) %axis x = [0.5,1] y = [-1,1]
clf; %clear figure

A = magic(5);
imagesc(A) %create a figure, wiv diff color for diff elements
imagesc(A),colorbar, colormap gray; %comma: execute commands at the same time

%FOR
v = zeros(10,1)
for i=1:10
    v(i) = 2^i;
end;
v

indices = 1:10;
for i = indices,
    disp(i); %display
end;
indices

%WHILE
i = 1;
while i<=5,
    v(i) = 100;
    i = i+1;
end;
v

i = 1;
while true,
    v(i) = 999;
    i = i+1;
    if i==6,    %IF
        break;  %break the WHILE
    end;
end;
v

%IF ELSE
v(1) = 2;
if v(1)==1,
    disp('The value is one');
elseif v(1)==2,
    disp('The value is two');
else
    disp('The value is not one or two');
end;

%function: fileName.m (under the path) then function name is fileName()

%eg1. squareThisNumber.m:
%   function y = squareThisNumber(x) %return value
%   y = x^2;
%addpath('path name') %add the path to current path

%eg2. squareAndCubeThisNumber.m:
%   function [y,z] = squareAndCubeThisNumber(x) %return value
%   y = x^2;
%   z = x^3;
%   command: [a,b] = squareAndCubeThisNumber(5);

%eg3. costFunctionJ.m
% function J = costFunctionJ(X,y,theta)
% m = size(X,1);
% predictions = X*theta;
% examples;
% sqrErrors = (predictions - y).^2;
%J = 1/(2*m) * sum(sqrErrors);


