function Main
s = serial('/dev/ttyS0');
fclose(s);
fopen(s);
setCounts(s,0,0);
global foodFlag;
global plotHandle;
global figHandle;
figHandle = figure;
arena = imread('arena-bw.jpg');
global arenaScaled;
arenaScaled = imresize(arena, 10.646);
imshow(arenaScaled);
hold on;

% Setup input window and flag
figure('Position',[50,800,250,250],'MenuBar','none','Name','Food found input','NumberTitle','off');
global fig;
fig = gcf;
set(gcf,'WindowButtonDownFcn',@setFoodFlag); % Mouse click
set(gcf,'KeyPressFcn',@setFoodFlag); % Key press
foodFlag = 0;



global runTime;
runTime = 60;
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
% while etime(currentTime,startTime) < runTime
%     goTo(s,820,4503);
%     currentTime = clock;
% end

% explore(s)
global foodAmount;
global homeFood;
global foodLocsX;
global foodLocsY;

foodAmount = 0;
homeFood =0;
foodLocsX = [];
foodLocsY = [];
% Start main loop
while(1)
    
    
    % Do your normal robot control stuff
    % ...
    explore(s)
       
    % Check for click on food window
    if foodFlag == 1
        
        % Do food finding stuff
        % ...
        disp('Food found!');
        foodAmount = foodAmount + 1;
        foodLocsX = cat(2,foodLocsX,x);
        foodLocsY = cat(2,foodLocsY,y);
        
        foodFlag = 0; % Reset flag
        
    end
    
    drawnow; % Need this to register button presses
    
    goTo(s,0,0)
    fprintf(s,'D,0,0');
    fscanf(s);
    homeFood = homeFood + foodAmount;
    dropFood(s)
    foodAmount = 0;
    
end
goTo(s,0,0)
fprintf(s,'D,0,0');
fscanf(s);
dropFood(s);
fclose(s);
end

function explore(s)
visitsX = [2842,6717,6930,6195,4364,2576,585,-1586,-1980,-3342];
visitsY = [1053,2150,3098,5706,4279,3598,6196,6153,4215,2682];
for i =1:(size(visitsX,2))
    x = visitsX(i);
    y = visitsY(i);
    goTo(s,x,y)
    i = i +1;
end
end

function wallFollow(s)
    global startTime;
    global currentTime;
    global runTime;
    sensorVals = readIR(s);
    while sensorVals(1)>70 || sensorVals(3)>130 || sensorVals(5) > 130
        sensorVals = readIR(s);
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
global figHandle;
global plotHandle;
global angle;
global x;
global y;
global countsPrev;
global countsCur;
global fig;
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
figure(figHandle);
delete(findobj(figHandle,'Color','r'));
plotHandle = plot(9368.5+xlist,7558.7-ylist);
plotHandle.Color = 'r';
title('Khepera Odometry Graph');
xlabel('x');
ylabel('y');
% set(0,'CurrentFigure','Food found input')
figure(fig);

 
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

function foodCheck
global x;
global y;
global foodAmount;
global foodLocsX;
global foodLocsY;
global foodFlag;

% Check for click on food window
if foodFlag == 1
    
    % Do food finding stuff
    disp('Food found!');
    fprintf(s,'D,0,0');
    fscanf(s);
    pause(1);
    foodAmount = foodAmount + 1;
    foodLocsX = cat(2,foodLocsX,x);
    foodLocsY = cat(2,foodLocsY,y);
    
    foodFlag = 0; % Reset flag
    
end

drawnow; % Need this to register button presses
end

function goTo(s,goalX,goalY)
global angle;
global x;
global y;
global a;
while a == true 
    vecAngle = getBearing(goalX,goalY);
    odometry(s)
    if ((goalX-122) > x || x > (goalX +122) || (goalY-122) > y || y > (goalY +122) )
        if (angle-vecAngle)<0
            while angle > vecAngle+0.2 || angle < vecAngle-0.2
                fprintf(s,'D,2,-2');
                fscanf(s);
                disp('Turning right to bearing');
                pause(0.1);
                odometry(s)
                vecAngle = getBearing(goalX,goalY);
                foodCheck;
            end
        else 
            while angle > vecAngle+0.2 || angle < vecAngle-0.2
                fprintf(s,'D,-2,2');
                fscanf(s);
                disp('Turning left to bearing');
                pause(0.1);
                odometry(s)
                vecAngle = getBearing(goalX,goalY);
                foodCheck;
             end
        end
        odometry(s)
        foodCheck;
        sensorVals = readIR(s);
        % senses and avoids obstacles
        if sensorVals(1)> 150 || sensorVals(2) > 150 || sensorVals(3)>150 || sensorVals(4) >140 || sensorVals(5) > 140
            obstacleFollow(s,goalX,goalY)
        end
        fprintf(s,'D,5,5');
        fscanf(s);
        disp('Moving forward to goal');
        pause(0.5);
        odometry(s)
        foodCheck;
        
    else
        a = false;
    end
    
end
a = true;
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
    foodCheck;
    while (abs(vecAngle - angle) > 0.1 || etime(currentTime,startTime) < 3)
    %while etime(currentTime,startTime) < 3
        if sensorVals(1)> 150 || sensorVals(2) > 150 || sensorVals(3)>150 || sensorVals(4) >140 || sensorVals(5) > 140 
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
        foodCheck;
        vecAngle = getBearing(goalX,goalY);
        pause(.05)
        currentTime = clock;
    end

end

function foundfood = pickupFood(s)
    sensorVals=readIR(s);
    %lightVals=readAmbient(s);
    foundfood=true;
    for i=1:8
        if sensorVals(i)<200 %|| lightVals(i)>150 %lightvals is always 500 no matter what, idk why
            foundfood=false;
        end
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

function lightVals = readAmbient(s)
fprintf(s,'O');
sensorString = fscanf(s);
splitString = regexp(sensorString,',','split');
lightVals = cellfun (@str2num,splitString(2:end));
end

function dropFood(s)
    for i=1:3
        fprintf(s,'L,0,1');
        fscanf(s);
        fprintf(s,'L,1,1');
        fscanf(s);
        pause(.1);
        fprintf(s,'L,0,0');
        fscanf(s);
        fprintf(s,'L,1,0');
        fscanf(s);
        pause(.1);
        i = i+ 1;
    end
end



function setFoodFlag(~,~)
foodFlag = 1;
end