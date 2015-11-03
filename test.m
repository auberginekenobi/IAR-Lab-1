function test()
s = serial('/dev/ttyS0');
fopen(s);
setCounts(s,0,0);
figure;
global countsPrev;
countsPrev = 0;
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



end

function odometry(s)
global angle;
global x;
global y;
global countsPrev;
counts = readCounts(s);
countsCur = counts- countsPrev 
countsPrev = counts;
angle = angle - 0.5*(countsCur(1) - countsCur(2))/(662) %66.2 = 5.4(khepera radius) *122.59259
y = y + 0.5*(countsCur(1) + countsCur(2))*cos(angle); 
x = x + 0.5*(countsCur(1) + countsCur(2))*sin(angle); 
global xlist;
global ylist;
xlist = cat(2,xlist,x)
ylist = cat(2,ylist,y)
                                                                                                              
plot(xlist,ylist);

 
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
