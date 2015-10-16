function tcalib
s = serial('/dev/ttyS0');
fopen(s);
setCounts(s,0,0);
counts = readCounts(s)
while counts(1) < 1000
    fprintf(s,'D,1,-1');
    fscanf(s);
    counts = readCounts(s)
end
fprintf(s,'D,0,0');
fscanf(s);
fclose(s);
end


function turn(s)
counts = readCounts(s)
%fprintf(s,'D,1,-1');
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
