function Main
s = serial('/dev/ttyS0');
fopen(s);
forward(s)
turn(s)
forward(s)
fclose(s);
end

function forward(s)
fprintf(s,'D,10,10');
fscanf(s);
sensorVals = readIR(s)
while (sensorVals(3)<130)
   sensorVals = readIR(s)
end
fprintf(s,'D,0,0');
fscanf(s);
end

function turn(s)
fprintf(s,'D,1,-1');
pause(2);
end


function sensorVals = readIR(s)
fprintf(s,'N');
sensorString = fscanf(s);
splitString = regexp(sensorString,',','split');
sensorVals = cellfun(@str2num,splitString(2:end));
end
