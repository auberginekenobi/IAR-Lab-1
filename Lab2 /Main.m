
function Main
s = serial('/dev/ttyS0');
fopen(s);
setCounts(s,0,0);
global a
a=true;
global x;
x = 0.0;
global y;
y = 0.0;
global angle;
angle = 0.0;
while a
    forward(s)
    wallFollow(s)
end
end

function wallFollow(s)
    sensorVals = readIR(s);
    while sensorVals(1)>70 || sensorVals(3)>180
        sensorVals = readIR(s);
        if sensorVals(8)<200
            disp('WALL FOLLOWING');
            if sensorVals(1)> 170 || sensorVals(3)>170
                disp('TOO CLOSE');
                fprintf(s,'D,2,-1');
                fscanf(s);
            elseif sensorVals(1) < 120  
                disp('TOO FAR AWAY');
                fprintf(s,'D,-1,2');
                fscanf(s);
            else
                disp('Following wall');
                fprintf(s,'D,5,5');
                fscanf(s);
            end
            odometry(s)
            pause(.1)
         else
             halt(s);
         end
    end
end

function odometry(s)
global angle;
global x;
global y;
counts = readCounts(s);
angle = angle - 0.5*(counts(1) - counts(2))/(2R);
x = x + 0.5*(counts(1) + counts(2))*cos(angle); 
y = y + 0.5*(counts(1) + counts(2)) *sin(angle); 
x   `
setCounts(s,0,0);
 
end

function forward(s) 
fprintf(s,'D,10,10');
fscanf(s);
sensorVals = readIR(s);
disp('hi, im driving forward');
while (sensorVals(3)<155)
   sensorVals = readIR(s);
   if sensorVals(8) >200
       halt(s)
   end
   pause(.1)
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
