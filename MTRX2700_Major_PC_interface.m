% Group 8
% James Cooper, Thejan Elankayer, Rishi Rangarajan,
% Naida Rasheed, David Young

% Program Function:
% PC interface for a object detection and area evaluation program
% running on a HCS12 microcontroller.

clc;
clear;
clf;
clf reset;

s = serial('COM6'); % Create a serial port object with the specified COM port
fopen(s); % Connect the serial port object


live_plotting = 1;

% Initialise PC or board mode
while 1
    string = input('Press Enter to send PC mode request...','s');
    if strcmp(string, '') == 1       % If pc mode selected
        fprintf(s, ' ');   % Send newline to board
        break;
    end
end


while 1
    
    % Check for data  
    while 1
        if s.BytesAvailable ~= 0
            command = fscanf(s, '%c\n');
            break;
        end
    end
       
    if strcmp(command, 'a') == 1    % Data request    
        % Wait for message
        while 1
            if s.BytesAvailable ~= 0
                message = fscanf(s, '%s\n');
                message = strrep(message, '_', ' ');
                fprintf('\n%s\n', message);
                break;
            end
        end
        string = input('>>> ','s');
        fprintf(s, string);
        
    elseif strcmp(command, 'l') == 1    % LIDAR test
        while 1
            if s.BytesAvailable ~= 0
                distance = fscanf(s, '%f');
                fprintf('Distance: %5.0f\n', distance);               
            end
        end
        
%     elseif strcmp(command, 's') == 1    % Servo test
%         while 1
%             string = input('>>> ','s');
%             fprintf(s, string);
%         end

    elseif strcmp(command, 'g') == 1    % Gyroscope test
        while 1
            if s.BytesAvailable ~= 0
                g_pan = fscanf(s, '%d\n');
                g_tilt = fscanf(s, '%d\n');
                fprintf('Pan: %d      Tilt: %d\n', g_pan, g_tilt);               
            end
        end
        
    elseif strcmp(command, 't') == 1    % Accelerometer test
        while 1
            if s.BytesAvailable ~= 0
                a_tilt = fscanf(s, '%d\n');
                fprintf('Tilt: %d\n', a_tilt);               
            end
        end
        
    elseif strcmp(command, 'm') == 1    % Magnetometer test
        while 1
            if s.BytesAvailable ~= 0
                m_pan = fscanf(s, '%d\n');
                fprintf('Pan: %d\n', m_pan);               
            end
        end

    elseif strcmp(command, 'd') == 1    % Plot data
        scan_mode = fscanf(s, '%s\n');
        resolution = fscanf(s, '%f');
        fprintf('Resolution: %.1f\n', resolution);
        
        % Arrays to store polar coordinates
        pan_array = [];
        tilt_array = [];
        r_array = [];
        
        % Arrays to store rectangular coordinates
        x_array = [];
        y_array = [];
        z_array = [];
        
        %%% Data %%%
        figure(1);
        s1 = subplot(2,2,1);
        title('1. Data between 0.8-1.2m');
        hold on; % Retain plots
        axis vis3d; % Freeze aspect ratio
        axis equal;
        grid on;
        grid minor;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view(-30,25);
        
        while 1
            if s.BytesAvailable ~= 0        % If there is data in the buffer
                
                pan = fscanf(s,'%f');       % Read the data from the buffer
                if abs(pan + 1) < 10^-16    % If a -1 is received
                    disp('Data termination string received.');
                    break;
                end
                tilt = fscanf(s,'%f');
                if abs(tilt + 1) < 10^-16    % If a -1 is received
                    disp('Data termination string received.');
                    fclose(s);
                    break;
                end
                r = fscanf(s,'%f');
                if abs(r + 1) < 10^-16    % If a -1 is received
                    disp('Data termination string received.');
                    fclose(s);
                    break;
                end
                
                pan_array(end+1) = pan;
                tilt_array(end+1) = tilt;
                r_array(end+1) = r;
                
                x = r * cos(deg2rad(tilt-90)) * sin(deg2rad(pan-90));
                y = r * cos(deg2rad(pan-90)) * sin(deg2rad(tilt-90));
                z = r * cos(deg2rad(pan-90)) * cos(deg2rad(tilt-90));
                
                
                fprintf('Pan: %6.1f   Tilt: %6.1f   Distance: %4.0f      x: %6.1f   y: %6.1f   z: %4.0f\n', pan,tilt,r, x,y,z);
                
                if z >= 800 && z <= 1200
                    x_array(end+1) = x;
                    y_array(end+1) = y;
                    z_array(end+1) = z;
                    
                    scatter3(x, y, z, 50, y, 'filled', 'MarkerEdgeColor','k');
                    if live_plotting == 1
                        drawnow; % Plot instantly
                    end
                end
                
            end
        end
        
        %%% Median Filter %%%
        s2 = subplot(2,2,2);
        title('2. Median Filter');
        hold on; % Retain plots
        axis vis3d; % Freeze aspect ratio
        axis equal;
        grid on;
        grid minor;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view(-30,25);
        
        median_z = median(z_array);
        fprintf('\nMedian z: %.2f\n', median_z);
        if strcmp(scan_mode, 'rectangle') == 1
            [x_f, y_f, z_f] = filter_data(x_array, y_array, z_array, median_z-40, median_z+40);
        else
            [x_f, y_f, z_f] = filter_data(x_array, y_array, z_array, median_z-40, median_z+40);
        end
        scatter3(x_f, y_f, z_f, 50, y_f, 'filled', 'MarkerEdgeColor','k');
        
        
        %%% Projection %%%
        s3 = subplot(2,2,3);
        title('3. Projection');
        hold on; % Retain plots
        axis vis3d; % Freeze aspect ratio
        axis equal;
        grid on;
        grid minor;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view(-30,25);
        
        [x_p, y_p, z_p] = projection(x_f, y_f, z_f);
        
        if strcmp(scan_mode, 'full') == 1
            % Filter x outliers
            out = or(x_p > mean(x_p)+1.3*iqr(x_p), x_p < mean(x_p)-1.3*iqr(x_p));
            scatter3(x_p(out), y_p(out), z_p(out), 'r*');
            [y_p, z_p, x_p] = filter_data(y_p, z_p, x_p, mean(x_p)-1.25*iqr(x_p), mean(x_p)+1.25*iqr(x_p));
            
            % Filter stand
            [x_p, y_p, z_p] = filter_stand(x_p, y_p, z_p);
        end
        
        scatter3(x_p, y_p, z_p, 50, y_p, 'filled', 'MarkerEdgeColor','k');
        
        
        %%% Find Area and Plot Shape %%%
        
        s4 = subplot(2,2,4);
        title('4. Shape Approximation');
        hold on; % Retain plots
        axis vis3d; % Freeze aspect ratio
        axis equal;
        grid on;
        grid minor;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view(-30,25);

        if strcmp(scan_mode, 'rectangle') == 1
            [width, height, area] = rectangle_area(x_p, y_p, z_p, resolution);
            fprintf('Rectangle Width: %.2f cm\n', width);
            fprintf('Rectangle Height: %.2f cm\n', height);
            fprintf('Rectangle Area: %.2f cm^2\n', area);
            
        elseif strcmp(scan_mode, 'full') == 1
            area = get_area(x_p, y_p, z_p, resolution);
            fprintf('Shape Area: %.2f cm^2\n', area);
        end
        
        
        % Link the 4 subplots to rotate together
        hlink = linkprop([s1,s2,s3,s4],{'CameraPosition','CameraUpVector'}); 
        %rotate3d on;
        
    end
      
end





function [width, height, area] = rectangle_area(x, y, z, resolution)
    
    % Get the indices of the min and max points in the x and y directions
    [i, left_index] = min(x);
    [i, right_index] = max(x);
    [i, bottom_index] = min(y);
    [i, top_index] = max(y);
        
    data = [x; y; z];    
    edges = [data(:,left_index) data(:,right_index) data(:,bottom_index) data(:,top_index)];
    
    new_x = edges(1,:);
    new_y = edges(2,:);
    new_z = edges(3,:);

    % Convert rectangular coordinates to polar coordinates
    pan = rad2deg(atan(new_x ./ new_z));
    tilt = rad2deg(atan(new_y ./ new_z));
    r = new_z ./ (cos(deg2rad(pan)) .* cos(deg2rad(tilt)));
        
    % Get the edges and convert them to rectangular coordinates
    left_edge = r(1) * cos(deg2rad(tilt(1))) * sin(deg2rad(pan(1) - 0.5*resolution));
    right_edge = r(2) * cos(deg2rad(tilt(2))) * sin(deg2rad(pan(2) + 0.5*resolution));
    bottom_edge = r(3) * cos(deg2rad(pan(3))) * sin(deg2rad(tilt(3) - 0.5*resolution));
    top_edge = r(4) * cos(deg2rad(pan(4))) * sin(deg2rad(tilt(4) + 0.5*resolution));
    
    % Plot rectangle shape
    rect = polyshape([left_edge left_edge right_edge right_edge],[bottom_edge top_edge top_edge bottom_edge]);
    plot(rect,'FaceColor','blue');
    
    width = (right_edge - left_edge)/10;
    height = (top_edge - bottom_edge)/10;    
    area = width * height;
    
end

function [x_f, y_f, z_f] = filter_data(x, y, z, min_z, max_z)       
    filtered = and(z >= min_z, z <= max_z);
    x_f = x(filtered);
    y_f = y(filtered);
    z_f = z(filtered);    
end

function [x_p, y_p, z_p] = projection(x, y, z)
    
    % Convert rectangular coordinates to polar coordinates with a new r
    % value 
    pan = rad2deg(atan(x./z));
    tilt = rad2deg(atan(y./z));
    r = median(z) ./ (cos(deg2rad(pan)) .* cos(deg2rad(tilt)));
    
    % Convert back to rectangular coordinates
    x_p = r .* cos(deg2rad(tilt)) .* sin(deg2rad(pan));
    y_p = r .* cos(deg2rad(pan)) .* sin(deg2rad(tilt));        
    z_p = r .* cos(deg2rad(pan)) .* cos(deg2rad(tilt));
    
end

function area = get_area(x, y, z, resolution)

    % Convert rectangular coordinates to polar coordinates
    pan = rad2deg(atan(x./z));
    tilt = rad2deg(atan(y./z));
    r = z ./ (cos(deg2rad(pan)) .* cos(deg2rad(tilt)));
    
    % Get the edges and convert them to rectangular coordinates
    left_edges = r .* cos(deg2rad(tilt)) .* sin(deg2rad(pan - 0.5*resolution));
    right_edges = r .* cos(deg2rad(tilt)) .* sin(deg2rad(pan + 0.5*resolution));
    bottom_edges = r .* cos(deg2rad(pan)) .* sin(deg2rad(tilt - 0.5*resolution));
    top_edges = r .* cos(deg2rad(pan)) .* sin(deg2rad(tilt + 0.5*resolution));
    
    % Plot rectangle shape
    subplot(2,2,4);
    hold on;
    for n = 1:length(x)
        rect = polyshape([left_edges(n) left_edges(n) right_edges(n) right_edges(n)],[bottom_edges(n) top_edges(n) top_edges(n) bottom_edges(n)]);
        plot(rect,'FaceColor','red');
    end
    
    % Get the difference between the edges to find the dimensions of each
    % rectangle approximation
    widths = right_edges - left_edges;
    heights = top_edges - bottom_edges;
    
    areas = widths .* heights;  % Calculate the area for each rectangle
    
    area = sum(areas);  % Sum all the rectangle approximations to get the total area
    
    area = area / 100;  % Convert mm^2 to cm^2
    
end


function [x, y, z] = filter_stand(x, y, z)
    
    values = uniquetol(y, 1e-10);
    frequencies = [];
    disp(frequencies);
    
    % Get the frequency of each value
    for n = values
        count = 0;
        for value = y
            if abs(n - value) < 1e-10
                count = count + 1;
            end
        end
        frequencies(end+1) = count;
    end
    
    median_freq = median(frequencies);
    disp(median_freq);
    
    remove_values = [];
    for n = 1:length(frequencies)        
        if frequencies(n) <= median_freq / 5
            remove_values(end+1) = values(n);
        else
            break;  % Stop if condition not satisfied
        end        
    end
    
    disp(remove_values);
    for index = length(y):-1:1
        for value = remove_values
            if abs(y(index)- value) < 1e-10
                x(index) = [];
                y(index) = [];
                z(index) = [];
                break;
            end
        end
    end

end