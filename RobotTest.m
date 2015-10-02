
import kcommands.*
s = serial('/dev/ttyS0')
fopen(s)
fprintf(s, 'N')
fscanf(s)
fclose(s)

