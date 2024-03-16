% Create serial port object
s = serialport("COM6", 115200);

% Set up serial port communication
configureTerminator(s, "LF");
r_vetor = zeros(100);
u_vetor = zeros(100);
lux_vetor = zeros(100);

% Read and plot data in real-time
startTime = datetime('now');
t = 1;
while t < 100
    % Read data from serial port
    data = readline(s);  % Read data from serial port
    disp(data);          % Display data in command window
    
    % Split the data string into individual values
    parts = split(data, ',');
    
    % Extract each value into its own variable
    for i = 1:length(parts)
        % Split each part into key-value pair
        keyValue = split(parts(i), ':');
        
        % Extract key and value
        key = strtrim(keyValue(1));
        value = str2double(strtrim(keyValue(2)));  % Convert value to double
        
        % Store the value in the appropriate variable
        switch key
            case "u"
                u = value;
            case "r"
                r = value;
            case "lux"
                lux = value;
        end
    end

    u_vetor(t) = u;
    r_vetor(t) = r;
    lux_vetor(t) = lux;
    
end