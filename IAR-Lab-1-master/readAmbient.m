%{
This is in its own separate function because I was tinkering with using
functions from different .m files in our new main3.
%}
function lightVals = readAmbient(s)
fprintf(s,'O');
sensorString = fscanf(s);
splitString = regexp(sensorString,',','split');
lightVals = cellfun (@str2num,splitString(2:end));
end