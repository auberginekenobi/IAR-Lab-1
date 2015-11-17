function Main
s = serial('/dev/ttyS0');
fclose(s);
fopen(s);
setCounts(s,0,0);
global runTime startTime currentTime;
global a x y angle countsPrev countsCur xlist ylist; 
global corners foodFlag plotHandle figHandle arenaCleared RG1 fig;

load('corners.mat','cornerlist');
corners = cornerlist;
figHandle = figure;
arena = imread('arena-bw.jpg');
arenaScaled = imresize(arena, 10.646);
thresh = graythresh(arenaScaled);
BW = im2bw(arenaScaled, thresh);
arenaCleared = bwareaopen(BW, 100000);
imshow(arenaCleared);
[label, num] = bwlabel(arenaCleared);
RG1 = regionprops(label,'Centroid');
hold on;
% Setup input window and flag
figure('Position',[50,800,250,250],'MenuBar','none','Name','Food found input','NumberTitle','off');
fig = gcf;
set(gcf,'WindowButtonDownFcn',@setFoodFlag); % Mouse click
set(gcf,'KeyPressFcn',@setFoodFlag); % Key press
foodFlag = 0;

runTime = 60;
a=true;
x = 0.0;
y = 0.0;
angle = 0.0;
countsPrev = 0;     
countsCur = 0;
xlist = [];
ylist =  [];

startTime = clock;
fix(startTime);
currentTime = clock;
% while etime(currentTime,startTime) < runTime
%     goTo(s,820,4503);
%     currentTime = clock;
% end

% explore(s)
global foodAmount homeFood foodLocsX foodLocsY;

foodAmount = 0;
homeFood =0;
foodLocsX = [];
foodLocsY = [];
% Start main loop
explore(s);
dropFood(s);
while etime(currentTime,startTime) < runTime-20
    
    
    % Do your normal robot control stuff
    % ...
    forage(s)
       
    % Check for click on food window
    foodCheck(s);

    dropFood(s);

    
    drawnow; % Need this to register button presses
    
end
goTo(s,0,0)
fprintf(s,'D,0,0');
fscanf(s);
dropFood(s);
fclose(s);
end

function explore(s)
global currentTime;
currentTime = clock;
visitsX = [2842,6717,6930,6195,4364,2576,585,-1586,-1980,-3342];
visitsY = [1053,2150,3098,5706,4279,3598,6196,6153,4215,2682];
for i =1:(size(visitsX,2))
    x = visitsX(i);
    y = visitsY(i);
    goTo(s,x,y)
    i = i +1;
    currentTime = clock;
end
end

function forage(s)
global foodLocsX foodLocsY currentTime;
currentTime = clock;
for i =1:(size(foodLocsX,2))
    x = foodLocsX(i);
    y = foodLocsY(i);
    goTo(s,x,y)
    i = i +1;
    currentTime = clock;
end
end


% function wallFollow(s)
%     global startTime;
%     global currentTime;
%     global runTime;
%     sensorVals = readIR(s);
%     while sensorVals(1)>70 || sensorVals(3)>130 || sensorVals(5) > 130
%         sensorVals = readIR(s);
%         currentTime = clock;
%         if etime(currentTime,startTime) > runTime
%             break
%             %halt(s)
%         elseif sensorVals(8)<200
%             disp('WALL FOLLOWING');
%             if sensorVals(1)> 150 || sensorVals(2) > 150 || sensorVals(3)>150 || sensorVals(4) >140 || sensorVals(5) > 110
%                 disp('TOO CLOSE');
%                 fprintf(s,'D,1,-1');
%                 fscanf(s);
%             elseif sensorVals(1) < 105  
%                 disp('TOO FAR AWAY');
%                 fprintf(s,'D,-1,1');
%                 fscanf(s);
%             else
%                 disp('Following wall');
%                 fprintf(s,'D,5,5');
%                 fscanf(s);
%             end
%             odometry(s)
%             pause(.05)
%          else
%              %halt(s);
%          end
%     end
% end

function odometry(s)
global figHandle plotHandle angle x y countsPrev countsCur fig;
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
figure(fig);

 
end

function forward(s) 
fprintf(s,'D,5,5');
fscanf(s);
sensorVals = readIR(s);
global startTime currentTime runTime;
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

function foodCheck(s)
global x y foodAmount foodLocsX foodLocsY foodFlag;

% Check for click on food window
if foodFlag == 1
    
    % Do food finding stuff
    disp('Food found!');
    fprintf(s,'D,0,0');
    fscanf(s);
    pause(1);
    foodAmount = foodAmount + 1;
    check = true;
    for i=1:size(foodLocsX,2)
        xdif = x - foodLocsX(i);
        ydif = y -foodLocsY(i);
        distance = euclidean(xdif,ydif);
        if distance <500
            
            check = false;
        end
    end
    if check == true
        disp('Adding food location');
        foodLocsX = cat(2,foodLocsX,x);
        foodLocsY = cat(2,foodLocsY,y);
    end
    foodFlag = 0; % Reset flag
    
end

drawnow; % Need this to register button presses
end

function goTo(s,goalX,goalY)
global angle x y a;
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
                foodCheck(s);
            end
        else 
            while angle > vecAngle+0.2 || angle < vecAngle-0.2
                fprintf(s,'D,-2,2');
                fscanf(s);
                disp('Turning left to bearing');
                pause(0.1);
                odometry(s)
                vecAngle = getBearing(goalX,goalY);
                foodCheck(s);
             end
        end
        odometry(s)
        foodCheck(s);
        sensorVals = readIR(s);
        % senses and avoids obstacles
        if sensorVals(1)> 160 || sensorVals(2) > 160 || sensorVals(3)>160 || sensorVals(4) >160 || sensorVals(5) > 160
            obstacleFollow2(s,goalX,goalY)
        end
        fprintf(s,'D,5,5');
        fscanf(s);
        disp('Moving forward to goal');
        pause(0.5);
        odometry(s)
        foodCheck(s);
        
    else
        a = false;
    end
    
end
a = true;
end

function vecAngle = getBearing(goalX,goalY)
global x y;

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

function obstacleFollow2(s,goalX,goalY)
global angle x y RG1;
mindistance = 400000;
obstacleid = 0;
disp('entering obstaclefollow');
odometry(s)

vecAngle = getBearing(goalX,goalY)
global corners;
% find which object we've encountered
% for i=1:size(corners,1)
%     distance = euclidean(x-corners(i,1),y-corners(i,2));
%     if distance < mindistance
%         mindistance = distance;
%         obstacleid = corners(i,3);
%     end

% end
% obtain a submatrix of those corners

% subX = [];
% subY = [];
% for i=1:size(corners,1)
%     if cornersensorVals = readIR(s);
% s(i,3) == obstacleid
%         subX = cat(2,subX,corners(i,1));
%         subY = cat(2,subY,corners(i,2));
%     end

% end
% use submatrix to calculate which direction to turn
gradient = ((goalY-y)/(goalX-x));
c = y - gradient*x;
fun=@(t) gradient*t + c;

id = 0;
centroids = cat(1, RG1.Centroid);
centroids = centroids/10;
centroids(:,1) = (10.646*centroids(:,1))-9368.5;
centroids(:,2) = (10.646*centroids(:,2)) -7558.7;
centroids(:,2) = -centroids(:,2);
x
y
for i=1:size(centroids,1)
    distance = euclidean(x-centroids(i,1),y-centroids(i,2))
    if distance < mindistance
        mindistance = distance
        nearCent = centroids(i,:);
        id = i;
    end
end
nearCent
id
disp('done obst calculations');
odometry(s)
% decide to turn left or right
vecObstacle = getBearing(nearCent(1),nearCent(2))

difBearing = vecObstacle - vecAngle
   
if (difBearing <= pi && difBearing >= 0 && id~=1) || (~(difBearing <= pi && difBearing >= 0) && id == 1) 
    %wallFollow(s,-1)
    direction = -1;
else 
    direction = 1;
    %wallFollow(s,1)
end
% wallFollow until our angle with each corner exceeds a certain amount

while ~(difBearing > pi/2 && difBearing < 3*pi/2)
    sensorVals = readIR(s);
    odometry(s)
    if direction ==-1
        if sensorVals(1)> 165 || sensorVals(2) > 165 || sensorVals(3)>140 || sensorVals(4) >140 || sensorVals(5) > 165
            disp('TOO CLOSE');
            fprintf(s,'D,1,-1');
            fscanf(s);
        elseif sensorVals(1) <100
            disp('TOO FAR AWAY');
            fprintf(s,'D,-1,1');
            fscanf(s);
        else
            disp('Following wall');
            fprintf(s,'D,2,3');
            fscanf(s);
        end
    else
        if sensorVals(1)> 165 || sensorVals(2) > 165 || sensorVals(3)>140 || sensorVals(4) >140 || sensorVals(5) > 165
            disp('TOO CLOSE');
            fprintf(s,'D,-1,1');
            fscanf(s);
        elseif sensorVals(6) <100
            disp('TOO FAR AWAY');
            fprintf(s,'D,1,-1');
            fscanf(s);
        else
            disp('Following wall');
            fprintf(s,'D,3,2');
            fscanf(s);
        end

    end
    odometry(s)
    pause(0.25);
    vecAngle = getBearing(goalX,goalY)
    vecObstacle = getBearing(nearCent(1),nearCent(2))
    difBearing = vecObstacle - vecAngle
   

end
disp('---around the obstacle---');
odometry(s)
goTo(s,goalX,goalY);
end

function distance = euclidean(x,y)
    distance = sqrt(x^2+y^2);
end

% function wallFollow(s,d,difBearing,goalX,goalY,nearCent)
%     %     right = 1, left = -1
% sensorVals = readIR(s);
% while ~(difBearing > pi/2 && difBearing < 3*pi/2)
%     if d==1
%         if sensorVals(1)> 140 || sensorVals(2) > 140 || sensorVals(3)>140 || sensorVals(4) >140 || sensorVals(5) > 140
%             %disp('TOO CLOSE');
%             fprintf(s,'D,1,-1');
%             fscanf(s);
%         elseif sensorVals(1) <100
%             %disp('TOO FAR AWAY');
%             fprintf(s,'D,-1,1');
%             fscanf(s);
%         else
%             %disp('Following wall');
%             fprintf(s,'D,2,3');
%             fscanf(s);
%         end
%     else
%         if sensorVals(1)> 140 || sensorVals(2) > 140 || sensorVals(3)>140 || sensorVals(4) >140 || sensorVals(5) > 140
%             %disp('TOO CLOSE');
%             fprintf(s,'D,-1,1');
%             fscanf(s);
%         elseif sensorVals(6) <100
%             %disp('TOO FAR AWAY');
%             fprintf(s,'D,1,-1');
%             fscanf(s);
%         else
%             %disp('Following wall');
%             fprintf(s,'D,3,2');
%             fscanf(s);
%         end
%     end
%     odometry(s)
%     sensorVals = readIR(s);
%     vecAngle = getBearing(goalX,goalY);
%     vecObstacle = getBearing(nearCent(1),nearCent(2));
%     difBearing = vecAngle - vecObstacle;
% end
% end


% function obstacleFollow(s,goalX,goalY)
%     global angle currentTime startTime;
%     odometry(s);
%     vecAngle = getBearing(goalX,goalY);
%     sensorVals = readIR(s);
% 
%     startTime = clock;
%     foodCheck(s);
%     while (abs(vecAngle - angle) > 0.1 || etime(currentTime,startTime) < 3)
%     %while etime(currentTime,startTime) < 3
%         if sensorVals(1)> 150 || sensorVals(2) > 150 || sensorVals(3)>150 || sensorVals(4) >140 || sensorVals(5) > 140 
%             disp('TOO CLOSE');
%             fprintf(s,'D,1,-1');
%             fscanf(s);
%         elseif sensorVals(1) <110
%             disp('TOO FAR AWAY');
%             fprintf(s,'D,-1,1');
%             fscanf(s);
%         else
%             disp('Following wall');
%             fprintf(s,'D,2,3');
%             fscanf(s);
%         end
%         % breaks wallfollow after 20 seconds. good for breaking out of infinite
%         % 'obstacles', aka walls.
%         
%         if etime (currentTime,startTime) > 40
%             break;
%         end
%         
%         sensorVals = readIR(s);
%         odometry(s)
%         foodCheck(s);
%         vecAngle = getBearing(goalX,goalY);
%         pause(.05)
%         currentTime = clock;
%     end
% 
% end

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
global foodAmount homeFood;
goTo(s,0,0);
fprintf(s,'D,0,0');
fscanf(s);
homeFood = homeFood + foodAmount;
disp(homeFood);
foodAmount = 0;
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
global foodFlag;
foodFlag = 1;
end