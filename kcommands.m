
function s = openConnection
s = serial('/dev/ttyS0');
fopen(s);
end

function closeConnection(s)
stop(s);
fclose(s);
end

function setSpeeds(s,leftSpeed,rightSpeed)
fprintf(s,['D,' num2str(leftSpeed) ',' num2str(rightSpeed)]);
fscanf(s);
end

function go(s,speed)
setSpeeds(s,speed,speed);
end

function stop(s)
go(s,0);
end

function turn(s,leftSpeed,rightSpeed)
setSpeeds(s,leftSpeed,rightSpeed);
end

function sensorVals = readIR(s)
fprintf(s,'N');
sensorString = fscanf(s);
splitString = regexp(sensorString,',','split');
sensorVals = cellfun(@str2num,splitString(2:end));
end

function sensorVals = readAmbient(s)
fprintf(s,'O');
sensorString = fscanf(s);
splitString = regexp(sensorString,',','split');
sensorVals = cellfun(@str2num,splitString(2:end));
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