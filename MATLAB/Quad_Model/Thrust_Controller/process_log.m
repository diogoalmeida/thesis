%% Reads a log from the APM board and creates a matrix with motor values

fid = fopen('motor_test.txt');

tline = fgetl(fid);

M = [];

while ischar(tline)
    if tline(1:3) == 'MOT'
        temp = textscan(tline,'%s%n%n%n%n%s','delimiter',',');
        M(end+1,:) = [temp{2:5}];
    end
    tline = fgetl(fid);
end
fclose(fid);