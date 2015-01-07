% system sampling frequency
sampleRate = 50000;

% maximum jerk
jerkAmount = 640000;

% fraction of (de/a)cceleration time where jerk is positive
jerkUpFrac = 1/3;

% velocity target in steps per second
veloTarget = 3200;

% move in number of steps
moveSteps = 3200;

% ------------------

% compute time required to reach set velocity (acceleration time)
accelTime = sqrt (veloTarget / jerkAmount / jerkPosFrac / (1.0 - jerkPosFrac));

disp (sprintf ('Time required to reach set velocity is %f seconds.', accelTime));

% compute amount of time where jerk is positive, zero, and negative
% based on jerkPosFrac and the acceleration time computed above

jerkUpTime = jerkUpFrac * accelTime;
jerkDownTime = jerkUpTime;
jerkZeroTime = accelTime - jerkUpTime - jerkDownTime;

disp (sprintf ('Acceleration Plan (seconds): Jerk Up: %f, Zero: %f, Down: %f.', ...
	jerkUpTime, jerkZeroTime, jerkDownTime));

jerkUpSamples = sampleRate * jerkUpTime;            % TODO: round this to nearest integer
jerkDownSamples = sampleRate * jerkDownTime;        % TODO: round this to nearest integer
jerkZeroSamples = sampleRate * jerkZeroTime;        % TODO: round this to nearest integer

disp (sprintf ('Acceleration Plan (samples): Jerk Up: %d, Zero: %d, Down: %d.', ...
    round(jerkUpSamples), round(jerkZeroSamples), round(jerkDownSamples)));

% compute number of steps moved in each of the first three segments of the curve
[p1,v1,a1] = pvajt ( 0,  0,  0, +jerkAmount, jerkUpSamples / sampleRate);
[p2,v2,a2] = pvajt (p1, v1, a1,           0, jerkZeroSamples / sampleRate);
[p3,v3,a3] = pvajt (p2, v2, a2, -jerkAmount, jerkDownSamples / sampleRate);

disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f', p1, v1, a1)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f', p2, v2, a2)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f', p3, v3, a3)); 

% computer number of steps moved at constant velocity
moveSeg4 = moveSteps - 2 * p3;

% compute time required to move those steps
timeSeg4 = moveSeg4 / v3;
samplesSeg4 = timeSeg4 * sampleRate;

disp (sprintf ('Need to move %f steps in segment four.', moveSeg4));
disp (sprintf ('This will take %f seconds (%d samples).', timeSeg4, round(samplesSeg4)));

% compute number of steps moved in segment four
[p4,v4,a4] = pvajt (p3, v3, a3, 0, samplesSeg4 / sampleRate);

disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f', p4, v4, a4)); 

% compute number of steps moved in each of the last three segments of the curve
[p5,v5,a5] = pvajt (p4, v4, a4, -jerkAmount, jerkUpSamples / sampleRate);
[p6,v6,a6] = pvajt (p5, v5, a5,           0, jerkZeroSamples / sampleRate);
[p7,v7,a7] = pvajt (p6, v6, a6, +jerkAmount, jerkDownSamples / sampleRate);

disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f', p5, v5, a5)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f', p6, v6, a6)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f', p7, v7, a7)); 

% ------------------

% compute jerk per sample period
jerkAmountQ = jerkAmount / sampleRate / sampleRate / sampleRate;

% compute run time in samples
runTimeSamples = jerkUpSamples + jerkZeroSamples + jerkDownSamples + ...
    samplesSeg4 + jerkUpSamples + jerkZeroSamples + jerkDownSamples;

% initial conditions
time = 0;
jerk = 0;
accel = 0;
speed = 0;
position = 0;

% record all variables versus time to create a plot
time_t = [0];
jerk_t = [0];
accel_t = [0];
speed_t = [0];
position_t = [0];

% used for displaying debug information
lastSegment = 0;
segment = 0;

for sample=1:(runTimeSamples+1)

    % calculate current time
    time = sample / sampleRate;
    
    % calculate jerk during the time period
    if (time < jerkUpTime)
        jerk = jerkAmountQ;
        segment = 0;
    elseif (time < (jerkUpTime + jerkZeroTime))
        jerk = 0;
        segment = 1;
    elseif (time < (jerkUpTime + jerkZeroTime + jerkDownTime))
        jerk = -jerkAmountQ;
        segment = 2;
    elseif (time < (runTime - (jerkUpTime + jerkZeroTime + jerkDownTime)))
        jerk = 0;
        segment = 3;
    elseif (time < (runTime - (jerkUpTime + jerkZeroTime)))
        jerk = -jerkAmountQ;
        segment = 4;
    elseif (time < (runTime - jerkUpTime))
        jerk = 0;
        segment = 5;
    elseif (time < runTime)
        jerk = jerkAmountQ;
        segment = 6;
    else 
        jerk = 0;
        segment = 7;
    end
    
    % display the position at the end of each segment
    if (segment ~= lastSegment)
        s = sprintf ('At the end of segment %d, the position is %f.', segment, position); 
        disp (s);
        lastSegment = segment;
    end
    
    % integrate jerk into acceleration
    accel = accel + jerk;
    
    % integrate acceleration into speed
    speed = speed + accel;
    
    % integrate speed into position
    position = position + speed;
    
    % record variables for plotting
    time_t = [time_t, time];
	jerk_t = [jerk_t, jerk*sampleRate*sampleRate*sampleRate];
	accel_t = [accel_t, accel*sampleRate*sampleRate];
	speed_t = [speed_t, speed*sampleRate];
	position_t = [position_t, position];   
end

figure (1);
subplot (4,1,1);
plot (time_t,jerk_t);
ylabel ('jerk');
subplot (4,1,2);
plot (time_t,accel_t);
ylabel ('acceleration');
subplot (4,1,3);
plot (time_t,speed_t);
ylabel ('velocity');
subplot (4,1,4);
plot (time_t,position_t);
ylabel ('position');
xlabel ('time');


% convert jerkAmount to jerk per sample period and quantize to 40 bits
% jerkAmountQ = round (jerkAmount / sampleRate^3 * 2^40);

