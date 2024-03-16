% Create serial port object
s = serialport("COM6", 115200);

% Initialize figure and axes
figure;
ax = axes;

% Initialize animated line objects
lineObj1 = animatedline(ax, 'Color', 'r');
lineObj2 = animatedline(ax, 'Color', 'g');


% Set axes properties
xlabel('Time');
ylabel('Value');
legend('r', 'lux');
ylim([0,25]);
grid on;

% Set up serial port communication
configureTerminator(s, "LF");

% Read and plot data in real-time
startTime = datetime('now');
while true
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
            case "r"
                r = value;
            case "lux"
                lux = value;
        end
    end
    
    % Update plot
    addpoints(lineObj1, seconds(datetime('now') - startTime), r);
    addpoints(lineObj2, seconds(datetime('now') - startTime), lux);

    drawnow limitrate;  % Update plot
end