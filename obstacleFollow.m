function obstacleFollow(s)
    sensorVals = readIR(s);
    global currentTime;
    global startTime;
    startTime = clock;
    while etime(currentTime,startTime) < 3
        if sensorVals(1)> 140 || sensorVals(2) > 150 || sensorVals(3)>150  
            disp('TOO CLOSE');
            fprintf(s,'D,1,-1');
            fscanf(s);
        else
            disp('Following wall');
            fprintf(s,'D,3,3');
            fscanf(s);
        end
        sensorVals = readIR(s);
        odometry(s)
        pause(.05)
        currentTime = clock;
    end

end