function test()
delete(instrfindall);

s = serial('/dev/ttyS0');
fopen(s);
fprintf(s,'net');
fscanf(s)
% fprintf(s,'list');
% fscanf(s)
% pause(5);
% fprintf(s,'T,116,Z');
% fscanf(s)
% fprintf(s,'T,116,X');
% fscanf(s)
% fprintf(s,'T,116,R,32');
% fscanf(s)
% for i=33:63
%     disp(i);
%     fprintf(s,'T,116,R,%d',i);
%     fscanf(s)
%     fprintf(s,'R,%d',i);
%     fscanf(s)
%     fprintf(s,'T,%d,Z',i);
%     fscanf(s)
% 
% end
% % pause(0.5);
fprintf(s,'D,2,2');
fscanf(s)
pause(2);
fprintf(s,'D,0,0');
fscanf(s)
fclose(s);
end
% for i=1:3
%     fprintf(s,'L,0,1');
%     fscanf(s);
%     fprintf(s,'L,1,1');
%     fscanf(s);
%     pause(.1);
%     fprintf(s,'L,0,0');
%     fscanf(s);
%     fprintf(s,'L,1,0');
%     fscanf(s);
%     pause(.1);
%     i = i+ 1;
% end
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
