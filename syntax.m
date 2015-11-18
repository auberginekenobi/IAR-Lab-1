% teting matlab syntax and other stuff
function syntax
global x y;
x=0;
y=0;
addfour();
addfour();
x
y
end

function addfour
global x y;
x = x + 4;
y = y+4;
end