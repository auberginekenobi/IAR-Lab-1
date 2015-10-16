
function Main
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
global countsPrev;
countsPrev = 0;
global xlist;
xlist = [];
global ylist;
ylist =  [];
while a
    forward(s)
    wallFollow(s)
end
end

function wallFollow(s)
    sensorVals = readIR(s)
    while sensorVals(1)>70 || sensorVals(3)>130 || sensorVals(5) > 130
        sensorVals = readIR(s)
        if sensorVals(8)<200
            disp('WALL FOLLOWING');
            if sensorVals(1)> 110 || sensorVals(2) > 130 || sensorVals(3)>130 || sensorVals(4) >130 || sensorVals(5) > 110
                disp('TOO CLOSE');
                fprintf(s,'D,1,-1');
                fscanf(s);
            elseif sensorVals(1) < 90  
                disp('TOO FAR AWAY');
                fprintf(s,'D,-1,1');
                fscanf(s);
            else
                disp('Following wall');
                fprintf(s,'D,3,3');
                fscanf(s);
            end
            odometry(s)
            pause(.05)
         else
             %halt(s);
         end
    end
end

function odometry(s)
global angle;
global x;
global y;
global countsPrev;
counts = readCounts(s);
countsCur = counts- countsPrev; 
countsPrev = counts;
angle = angle - (countsCur(1) - countsCur(2))/(662); %66.2 = 5.4(khepera diameter) *122.59259
y = y + 0.5*(countsCur(1) + countsCur(2))*cos(angle); 
x = x + 0.5*(countsCur(1) + countsCur(2))*sin(angle);
global xlist;
global ylist;
xlist = cat(2,xlist,x);
ylist = cat(2,ylist,y);
                                                                                                              
plot(xlist,ylist);

 
end

function forward(s) 
fprintf(s,'D,3,3');
fscanf(s);
sensorVals = readIR(s)
disp('hi, im driving forward');
while (sensorVals(3)<130 && sensorVals(5) < 130 && sensorVals(2) <130)
   sensorVals = readIR(s)
   if sensorVals(8) >200
       %halt(s)
   end
   odometry(s)
   pause(.05)
end
fprintf(s,'D,0,0');
fscanf(s);
end

function halt(s)
    fprintf(s,'D,0,0');
    fscanf(s);
    fclose(s);
    global a
    a=false;
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
