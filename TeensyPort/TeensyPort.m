% The purpose of this MATLAB script is to offer streamlined interaction
% with the ADCS and FC teensy during testing. The port will prompt you for
% test code characters and timeout values when requested by the teensy,
% alerts will be displayed, and all data will be saved to the specified
% file.

% Start Configuration Variables

% The serial port should be connected to the teensy. This port can be found
% by running "ls /dev/tty.*" in terminal or looking at the port being used
% by the Arduino IDE "Tools -> Port -> /dev/tty. ..."
Serial = serial('/dev/tty.');

File = fopen('/adcs/data/test_08_06_18_00.txt', 'w');

% End Configuration Variables

set(Serial, 'BaudRate', 9600);
fopen(Serial);

% Test timeout value
tmov = 0;

% Loop until the close command of '0' is written
while(1)
   
    % Wait for available data
    while(~Serial.BytesAvailable)
    end
    
    % Test timeout occurs
    if(tmov > 0 && toc > tmov)
        fprintf(Serial, '0');
    end
    
    % Read in and process data
    line = fgets(Serial);
    switch(line(1))
       
        % Alert message
        case '!'
            fprintf('ALERT: %s', line(2:length(line)));
            fprintf(File, '%s', line);
            
        % Test identifier
        case '@'
            fprintf('\nStarting test %s\n', line(2));
            fprintf(File, '%s', line);
            tic;
            
        % Data line
        case '#'
            fprintf(File, '%s', line);
            
        % Requesting character code
        case '$'
            fprintf('\nCharacter test code requested\n');
            code = input('Code: ', 's');
            tmov = input('Tmov: ');
            fprintf(Serial, "%s", code);
            
        % TeensyPort close command
        case '0'
            fprintf('Exiting TeensyPort\n');
            break;
            
        % Unknown command
        otherwise
            fprintf('Unknown command reveived from Teensy\n');
            
    end
    
end