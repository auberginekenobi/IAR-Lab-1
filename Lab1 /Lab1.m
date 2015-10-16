               
s = serial('/dev/ttyS0');
fopen(s);
fprintf(s,'D,0,0')
fscanf(s)
fprintf(s,'L,0,2')
fscanf(s)
pause(2);
fprintf(s,'L,0,2')
fscanf(s)
fclose(s)
                   