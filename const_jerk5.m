% system sampling frequency
sampleRate = 50000;

% maximum jerk
jerkAmount = 640000;

% fraction of (de/a)cceleration time where jerk is positive
jerkUpFrac = 1/3;

% velocity target in steps per second
veloTarget = 3200;
veloTarget = 49500;
veloTarget = 50000;

% move in number of steps
moveSteps = 3200;
moveSteps = 10000;
moveSteps = 32007;
moveSteps = 84000;

% ------------------

% convert jerkAmount to jerk per sample period 
% quanitze to 64 total bits with 40 fractional bits
jerkAmountInt64 = int64 (jerkAmount / sampleRate^3 * 2^40);
veloTargetInt64 = int64 (veloTarget / sampleRate * 2^40);
moveStepsInt64 = int64(bitshift (int64(moveSteps), 40));
accelSamplesInt64 = int64 (floor ( ...
    sqrt (double(veloTargetInt64) / double(jerkAmountInt64) / jerkUpFrac / (1.0 - jerkUpFrac))));

seg1Samples = int64 (floor (jerkUpFrac * double (accelSamplesInt64)));
seg3Samples = int64 (seg1Samples);
seg2Samples = int64 (accelSamplesInt64 - seg1Samples - seg3Samples);
seg5Samples = seg3Samples;
seg6Samples = seg2Samples;
seg7Samples = seg1Samples;

% calculate position after acceleration segments complete 
[p1,v1,a1] = pvajt ( 0,  0,  0, +jerkAmountInt64, seg1Samples);
[p2,v2,a2] = pvajt (p1, v1, a1,                0, seg2Samples);
[p3,v3,a3] = pvajt (p2, v2, a2, -jerkAmountInt64, seg3Samples);

disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f (at end of segment 1)', ...
    double(p1) / 2^40, double(v1) / 2^40 * sampleRate, double(a1) / 2^40 * sampleRate^2)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f (at end of segment 2)', ...
    double(p2) / 2^40, double(v2) / 2^40 * sampleRate, double(a2) / 2^40 * sampleRate^2)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f (at end of segment 3)', ...
    double(p3) / 2^40, double(v3) / 2^40 * sampleRate, double(a3) / 2^40 * sampleRate^2)); 

% calculate position after deceleration segments complete 
% without the constant velocty segment included
[pf,vf,af] = pvajt (p3, v3, a3, -jerkAmountInt64, seg5Samples);
[pf,vf,af] = pvajt (pf, vf, af,                0, seg6Samples);
[pf,vf,af] = pvajt (pf, vf, af, +jerkAmountInt64, seg7Samples);

disp (sprintf ('pf: %10.2f, vf: %10.2f, af: %10.2f (at end of move excluding segment 4)', ...
    double(pf) / 2^40, double(vf) / 2^40 * sampleRate, double(af) / 2^40 * sampleRate^2)); 

seg4Move = moveStepsInt64 - pf;
seg4Samples = int64(ceil(double(seg4Move) / double(v3)));

disp (sprintf ('Need to move %10.2f steps in segment 4 in %d samples', ...
    double(seg4Move) / 2^40, seg4Samples));

[p4,v4,a4] = pvajt (p3, v3, a3, 0, seg4Samples);

disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f (at end of segment 4)', ...
    double(p4) / 2^40, double(v4) / 2^40 * sampleRate, double(a4) / 2^40 * sampleRate^2)); 

[p5,v5,a5] = pvajt (p4, v4, a4, -jerkAmountInt64, seg5Samples);
[p6,v6,a6] = pvajt (p5, v5, a5,                0, seg6Samples);
[p7,v7,a7] = pvajt (p6, v6, a6, +jerkAmountInt64, seg7Samples);

disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f (at end of segment 5)', ...
    double(p5) / 2^40, double(v5) / 2^40 * sampleRate, double(a5) / 2^40 * sampleRate^2)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f (at end of segment 6)', ...
    double(p6) / 2^40, double(v6) / 2^40 * sampleRate, double(a6) / 2^40 * sampleRate^2)); 
disp (sprintf ('pt: %10.2f, vt: %10.2f, at: %10.2f (at end of segment 7)', ...
    double(p7) / 2^40, double(v7) / 2^40 * sampleRate, double(a7) / 2^40 * sampleRate^2)); 

% ------------------

% compute run time in samples
runTimeSamples = seg1Samples + seg2Samples + seg3Samples + ...
    seg4Samples + seg5Samples + seg6Samples + seg7Samples;

% initial conditions
time = 0;
jerk = int64(0);
accel = int64(0);
speed = int64(0);
position = int64(0);

% record all variables versus time to create a plot
time_t = [0];
jerk_t = [0];
accel_t = [0];
speed_t = [0];
position_t = [0];

% used for displaying debug information
lastSegment = 0;
segment = 0;

for sample=1:runTimeSamples+1

    % calculate current time
    time = double(sample) / double(sampleRate);
    
    % calculate jerk during the sample period
    if (sample <= seg1Samples)
        jerk = jerkAmountInt64;
        segment = 0;
    elseif (sample <= (seg1Samples + seg2Samples))
        jerk = 0;
        segment = 1;
    elseif (sample <= (seg1Samples + seg2Samples + seg3Samples))
        jerk = -jerkAmountInt64;
        segment = 2;
    elseif (sample <= (seg1Samples + seg2Samples + seg3Samples + seg4Samples))
        jerk = 0;
        segment = 3;
    elseif (sample <= (seg1Samples + seg2Samples + seg3Samples + seg4Samples + ...
			seg5Samples))
        jerk = -jerkAmountInt64;
        segment = 4;
    elseif (sample <= (seg1Samples + seg2Samples + seg3Samples + seg4Samples + ...
			seg5Samples + seg6Samples))
        jerk = 0;
        segment = 5;
    elseif (sample <= (seg1Samples + seg2Samples + seg3Samples + seg4Samples + ...
			seg5Samples + seg6Samples + seg7Samples))
        jerk = jerkAmountInt64;
        segment = 6;
    else 
        jerk = 0;
        segment = 7;
    end
    
    % display the position at the end of each segment
    if (segment ~= lastSegment)
        s = sprintf ('At the end of segment %d, the position is %d.', segment, int64(bitshift(position,-40))); 
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
	jerk_t = [jerk_t, (double(jerk)/2^40*sampleRate^3)];
	accel_t = [accel_t, double(accel)/2^40*sampleRate^2];
	speed_t = [speed_t, double(speed)/2^40*sampleRate];
	position_t = [position_t, double(position)/2^40];   
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

