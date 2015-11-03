%{
main file for task 3.
Note: we should use an event listener for the food pickup.
%}
function main3()
    s = serial('/dev/ttyS0');
    fopen(s);
    readAmbient(s)
    fclose(s);
end