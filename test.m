function test(s)
s = serial('/dev/ttyS0');
fopen(s);
setCounts(s,0,0);
figure;
global a
a=true;
global x;
x = 0.0;
global y;
y = 0.0;
global angle;

angle = 0.0;
global xlist;
xlist = [];
global ylist;
ylist =  [];
fprintf(s,'D,5,5');
fscanf(s);
odometry(s)
pause(3);
odometry(s)
fprintf(s,'D,0,0');
fscanf(s);
fprintf(s,'D,-5,-5');
fscanf(s);
odometry(s)
pause(3);
odometry(s)
fprintf(s,'D,0,0');
fscanf(s);
fclose(s);
end

function odometry(s)
global angle;
global x;
global y;
counts = readCounts(s);
angle = angle - 0.5*(counts(1) - counts(2))/(5.4)
y = y + 0.5*(counts(1) + counts(2))*cos(angle); 
x = x + 0.5*(counts(1) + counts(2)) *sin(angle); 
global xlist;
global ylist;
xlist = cat(2,xlist,x)
ylist = cat(2,ylist,y)
                                                                                                              
plot(xlist,ylist);
setCounts(s,0,0);
 
end


function setCounts(s,leftCount,rightCount)
fprintf(s, ['G,' num2str(leftCount) ',' num2str(rightCount)]);
fscanf(s);
end


function counts = readCounts(s)
fprintf(s,'H');
countString = fscanf(s);
splitString = regexp(countString,',','split');
counts = cellfun(@str2num,splitString(2:end));
end 

function sensorVals = readIR(s)
fprintf(s,'N');
sensorString = fscanf(s);
splitString = regexp(sensorString,',','split');
sensorVals = cellfun(@str2num,splitString(2:end));
end
