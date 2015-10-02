
import kcommands.*
s = serial('/dev/ttyS0')
fopen(s)
%a = fscanf(s)
%while a ~= ''
    %a=fscanf(s);
%end
fprintf(s,'L,0,2')
fscanf(s)
pause(5);
fprintf(s,'L,0,2')
fscanf(s)
fclose(s)
