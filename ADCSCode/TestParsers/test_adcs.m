% Adjustable variables ----------------------------------------------------

% Match this string to the name of the Teensy's serial port. On a mac this
% can be determined by running "ls /dev/tty.*" in terminal or looking at
% the port being used by the Arduino IDE "Tools -> Port".
SerialPort = '/dev/tty.';

% This is the filename where the testing data will be stored.
CSVFileName = 'data/rev1/t_08_31_18_00.txt';

% End of Adjustable Variables ---------------------------------------------

% Initialize serial object to communicate with the Teensy
Serial = serial(SerialPort);
set(Serial, 'BaudRate', 9600);
fopen(Serial);

% Initialize data file to store test data
File = fopen(CSVFileName, 'w');

% Loop to run more tests until the end charactar '0' is input
flag = 0;
while(~flag)
  
    % Read in test charactar - only the first charactar is used
    n = input('\nTest charactar: ', 's');
    t = input('Test time (seconds): ');
    fprintf('\n');
    flag = (n(1) == '0');
    
    % Write command over Serial
    fprintf("%s\n", n);
    
    if(~flag)
        while(next_line(File))
        end
    end
    
end

% Clean up resources

clear SerialPort
clear CSVDirectory
clear flag
clear line

fclose(Serial);
fclose(File);
delete(Serial);
delete(File);
clear Serial;
clear File

function should_end = next_line()
% Parses the teensy lines and display/writes the appropriate data to the
% command window and/or output files. If the test has completed true is
% returned.

    should_end = 0;

    switch line(1)
        
        % Alert line recieved
        case '!'
            fprintf('ALERT: %s\n', line(2:length(line)));
            
        % Test identifier recieved
        case '@'
            fprintf('Beginning test %s\n', line(2));
            
        % Data line recieved
        case '#'
            % TODO : Write data lines to the appropriate output file
            
        % End of a test without a timeout
        case '$'
            should_end = 1;
            
        % Undefined behavior
        otherwise
            fprintf('Undefined message type recieved');
        
    end
end
