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

% Counters for the number of times each test has been run
TestCounter = [
    'g', 0;
    's', 0
];

% Loop to run more tests until the end charactar '0' is input
flag = 0;
while(~flag)
  
    % Read in test charactar - only the first charactar is used
    n = input('\nTest charactar: ', 's');
    t = input('Test time (seconds): ');
    fprintf('\n');
    flag = (n(1) == '0');
    
    % Retrieve index of test id
    i = find(TestCounter(:,1) == n(1));
    
    if(isempty(i))
        
        % Invalid test ID
        fprintf('Not a valid test ID. Please try again.');
        
    else
        
        % Write test code to 
        fprintf(Serial, '%s', i(1));
        
        line = fgetl(Serial);
        parse_line(line);
        
        % Increment test counter
        TestCounter(i(1),2) = TestCounter(i(1),2) + 1;
        
    end
    
end

% Clean up resources

clear SerialPort
clear CSVDirectory
clear flag
clear line

fclose(Serial);
delete(Serial);
clear Serial;

function handle_test(code, count)



end

function handle_line(line)
% Parses the teensy lines and display/writes the appropriate data to the
% command window and/or output files.

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
            
        % Undefined behavior
        otherwise
            fprintf('Undefined message type recieved');
        
    end
end
