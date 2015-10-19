
function Main
s = serial('/dev/ttyS0');
fopen(s);
setCounts(s,0,0);
figure; 
global runTime;
runTime = 40;
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
global countsCur;
countsCur = 0;
global xlist;
xlist = [];
global ylist;
ylist =  [];
global startTime;
global currentTime;
startTime = clock;
fix(startTime);
currentTime = clock;
while etime(currentTime,startTime) < runTime
    forward(s)
    wallFollow(s)

    currentTime = clock;
end

goTo(s,0,0)
fprintf(s,'D,0,0');
fscanf(s);
fclose(s);
end


function wallFollow(s)
    global startTime;
    global currentTime;
    global runTime;
    sensorVals = readIR(s);
    while sensorVals(1)>70 || sensorVals(3)>130 || sensorVals(5) > 130
        sensorVals = readIR(s)
        currentTime = clock;
        if etime(currentTime,startTime) > runTime
            break
            %halt(s)
        elseif sensorVals(8)<200
            disp('WALL FOLLOWING');
            if sensorVals(1)> 150 || sensorVals(2) > 150 || sensorVals(3)>150 || sensorVals(4) >140 || sensorVals(5) > 110
                disp('TOO CLOSE');
                fprintf(s,'D,1,-1');
                fscanf(s);
            elseif sensorVals(1) < 105  
                disp('TOO FAR AWAY');
                fprintf(s,'D,-1,1');
                fscanf(s);
            else
                disp('Following wall');
                fprintf(s,'D,5,5');
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
global countsCur;
counts = readCounts(s);
countsCur = counts- countsPrev; 
countsPrev = counts;
angle = angle - (countsCur(2)- countsCur(1))/(662); %662 = 5.4(khepera diameter) *122.59259
angle = mod(angle,2*pi);
y = y + 0.5*(countsCur(1) + countsCur(2))*cos(angle);
x = x + 0.5*(countsCur(1) + countsCur(2))*sin(angle);
global xlist;
global ylist;
xlist = cat(2,xlist,x);
ylist = cat(2,ylist,y);
                                                                                                              
plot(xlist,ylist);
title('Khepera Odometry Graph');
xlabel('x');
ylabel('y');

 
end

function forward(s) 
fprintf(s,'D,5,5');
fscanf(s);
sensorVals = readIR(s);
global startTime;
global currentTime;
global runTime;
disp('hi, im driving forward');
while (sensorVals(3)<130 && sensorVals(5) < 130 && sensorVals(2) <130)
   sensorVals = readIR(s);
   odometry(s)
   pause(.05)
   currentTime = clock;
   if etime(currentTime,startTime) > runTime
       break
       %halt(s)
   end
end
fprintf(s,'D,0,0');
fscanf(s);
end

function goTo(s,goalX,goalY)
global angle;
global x;
global y;
global a;
while a == true 
    vecAngle = getBearing(goalX,goalY);
    odometry(s)

    if (angle-vecAngle)<0
        while angle > vecAngle+0.2 || angle < vecAngle-0.2
            fprintf(s,'D,1,-1');
            fscanf(s);
            disp('Turning right to bearing');
            pause(0.1);
            odometry(s)
            vecAngle = getBearing(goalX,goalY);
        end
    else 
        while angle > vecAngle+0.2 || angle < vecAngle-0.2
            fprintf(s,'D,-1,1');
            fscanf(s);
            disp('Turning left to bearing');
            pause(0.1);
            odometry(s)
            vecAngle = getBearing(goalX,goalY);
         end
     end
        odometry(s)
        sensorVals = readIR(s)
        % senses and avoids obstacles
        if sensorVals(1)> 140 || sensorVals(2) > 150 || sensorVals(3)>150
            obstacleFollow(s,0,0) 
        end
    
        if ((goalX-200) > x || x > (goalX +200) || (goalY-200) > y || y > (goalY +200) )
        fprintf(s,'D,3,3');
        fscanf(s);
        disp('Moving forward to goal');
        pause(0.5);
        odometry(s)

        else
        a = false;
        end
    
end
end

function vecAngle = getBearing(goalX,goalY)
global x;
global y;

difX = goalX - x;
difY = goalY - y;

vecMag = sqrt((difX^2) + (difY^2));
vecAngle = asin(abs(difX)/abs(vecMag));
if y <goalY && x > goalX
    vecAngle = pi/2 - vecAngle;
    vecAngle = vecAngle +pi/2;
end
if x <goalX && y>goalY
    vecAngle = pi - vecAngle;
end
if x > goalX
    vecAngle = vecAngle + pi;
end

vecAngle = mod(vecAngle,2*pi);

end

function obstacleFollow(s,goalX,goalY)
    global angle;
    
    vecAngle = getBearing(goalX,goalY);
    sensorVals = readIR(s);
    global currentTime;
    global startTime;
    startTime = clock;
    while (abs(vecAngle - angle) > 0.1 || etime(currentTime,startTime) < 3)
    %while etime(currentTime,startTime) < 3
        if sensorVals(1)> 140 || sensorVals(2) > 150 || sensorVals(3)>150  
            disp('TOO CLOSE');
            fprintf(s,'D,1,-1');
            fscanf(s);
        elseif sensorVals(1) <110
            disp('TOO FAR AWAY');
            fprintf(s,'D,-1,1');
            fscanf(s);
        else
            disp('Following wall');
            fprintf(s,'D,2,3');
            fscanf(s);
        end
        % breaks wallfollow after 20 seconds. good for breaking out of infinite
        % 'obstacles', aka walls.
        
        if etime (currentTime,startTime) > 40
            break;
        end
        
        sensorVals = readIR(s);
        odometry(s)
        vecAngle = getBearing(goalX,goalY);
        pause(.05)
        currentTime = clock;
    end

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