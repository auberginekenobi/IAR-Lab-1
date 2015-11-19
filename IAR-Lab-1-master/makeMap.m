%{
High-level description:
Want to hard-code a map into a .mat file so that when we design a program
for task 3, we can simply load the .mat file from file.
This script will generate a file called map.mat for this purpose.
At this point, I think it will use the pic Barbara took to generate the
map.
Could preprocess the image manually to make the map better / easier to
make.
%}
function makeMap
%loads photo as 915x1693x3 array = height x width x rgb
photo = imread('arena.jpg');
whos photo
% save ('map.mat','var1','var2',...);
end