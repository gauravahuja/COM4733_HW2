%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 2
%
% Team number: 15
% Team leader: Gaurav Ahuja (ga2371)
% Team members: Sarah Panda (sp3206), Nikita Ushakov (nyu2000)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% main function

function  hw2_team_15(serPort)
    % Variable Declaration
    
    Initial_Distance = DistanceSensorRoomba(serPort);
    total_distance = 0;
    robotRadius= 0.125;
    state = 0; %Initial State when robot has not touched the wall for the first time
    pause_time = 0.01; %Pause time for all commands
    pause_time_2 = 0.01; %Pause time for main while loop
    strip_width = 10;
    
    p = properties(serPort);
    denominator = 22;
    if size(p,1) == 0
        %If we are inside a simulator increase the pause time of main while
        %loop
        denominator = 18;
        pause_time_2 = 0.1;
    end
    
    fwdSpeed= 0.1; %Forward speed of Create
    angSpeed = 0.8; %Angular speed of Create
    
    %Start Moving Forward
    SetFwdVelAngVelCreate(serPort, fwdSpeed, 0); 
    pause(pause_time);
    
    %parameters to track location
    coords_target = [0 4 0];
    coords_old = [0 0 0];      %create a vector with x, y, angle coordinates
    coords_new = [0 0 0];
    turn_angle_old = 0;        %setup an angle to be measure according to axis of the initial position in degrees
    turn_angle_new = 0;
    %distance = DistanceSensorRoomba(serPort);    %distance var to keep track of every movement
    %turn_angle_new = AngleSensorRoomba(serPort);
    total_distance = 0;
    stop_range = 0.2;
    minimum_stop_distance = 1;
    
    fprintf('Start state machine now\n');
    
    f = figure;
    plot(coords_new(1),coords_new(2),'gx');
    plot(coords_target(1), coords_target(2), 'r*');
    hold on;
    
    while true
        [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        WallSensor = WallSensorReadRoomba(serPort);
        
        bumped = BumpRight||BumpLeft||BumpFront;
        fprintf('[Sensors] w=%d, r=%d, l=%d, f=%d, state=%d\n', WallSensor, BumpRight, BumpLeft, BumpFront, state);

        figure(f);
        plot(coords_new(1), coords_new(2), 'x');
        hold on;
        
        if state == 0
            %In State 0, We assume that the robot will hit the obstacle
            %head on, after that it switches to State 1, where it
            %circumnavigates the obstacle
            fprintf('State 0, Following path\n');
            if(bumped)
                fprintf('Hit the Obstacle\n');
                state = 1;
                total_distance = 0;

                SetFwdVelAngVelCreate(serPort, 0, 0);

                distance = DistanceSensorRoomba(serPort);
                turn_angle_new = AngleSensorRoomba(serPort);

                coords_new = coords_old + [(distance*sin(coords_new(3) + turn_angle_new)) (distance * cos(coords_old(3) + turn_angle_new)) (turn_angle_new)];
                coords_old = coords_new;
                
            end

        elseif (state == 2)
            fprintf('State 2, Rotating to align with path\n');
            current_angle = coords_new(3);
            m = -1;
            if (current_angle < 0)
                m = 1;
            end
            
            SetFwdVelAngVelCreate(serPort, 0, m*angSpeed);
            
            a2 = AngleSensorRoomba(serPort);
            angle = coords_old(3) + coords_new(3) + a2;
            fprintf('Angle = %f\n', angle);
            while ((abs(angle) > 0.1))
                a2 = AngleSensorRoomba(serPort);
                angle = angle + a2;
                fprintf('Angle = %f\n', angle);
                pause(pause_time);
            end
            coords_new(3) = 0;
            coords_old(3) = angle;
            SetFwdVelAngVelCreate(serPort, fwdSpeed, 0);

            state = 0;

        elseif (state == 1)
            fprintf('State 1\n');
            distance = DistanceSensorRoomba(serPort);
        
            turn_angle_new = AngleSensorRoomba(serPort);
            coords_new = coords_old + [(distance*sin(coords_new(3) + turn_angle_new)) (distance * cos(coords_old(3) + turn_angle_new)) (turn_angle_new)];
            display(coords_new);
            coords_old = coords_new;
            total_distance = total_distance + distance;
            
            if(total_distance > 30 && abs(coords_new(1)) < strip_width)
                state = 2;         
            end
            
            if (WallSensor==1 && BumpFront==1)
                %This state handles the case when the robot is following
                %the wall and another wall comes infront of it. Now it
                %rotates CW. As soon as it has rotated PI rad CW it starts
                %checking the Wall Sensor, if Wall sensor is turned on it
                %stops rotating and follows the new wall.
                fprintf('Wall at right and front, Rotating CW > PI and checking for Wall Sensor\n');
                
                SetFwdVelAngVelCreate(serPort, 0, -angSpeed);
                pause(pause_time);
                
                angle = 0;
                
                while true
                    a2 = AngleSensorRoomba(serPort);
                    coords_old(3) = coords_old(3) + a2;
                    
                    angle = angle + abs(a2);
                    pause(pause_time);
                    fprintf('Angle: %f\n', angle);

                    if (angle > pi)
                        WallSensor = WallSensorReadRoomba(serPort);
                        fprintf('Wall Sensor = %d\n', WallSensor);
                        if WallSensor == 1
                            break;
                        end
                        pause(pause_time);
                    end
                end
                SetFwdVelAngVelCreate(serPort, fwdSpeed, 0);

            elseif (WallSensor==1 && BumpRight)
                %This state occurs when the robot is just touching the
                %wall, so it rotates CCW untill right bump sensor is turned
                %off
                
                fprintf('Dragging on wall\n');
                fprintf('Rotating ccw untill right bumper is off\n')
                SetFwdVelAngVelCreate(serPort, 0, angSpeed);
                while(BumpRight == 1)
                    [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                    pause(pause_time);
                end
                SetFwdVelAngVelCreate(serPort, fwdSpeed, 0);
                
            elseif(WallSensor==0 && BumpRight)
                %This state occurs when the Robot is too close to the wall
                %being followed that the Wall sensor gets turned off. It
                %now rotates CCW untill the right bump sensor gets turned
                %off.
                fprintf('Right Bump is on\n');
                fprintf('Rotating CCW\n');
                SetFwdVelAngVelCreate(serPort, 0, angSpeed);
                while BumpRight==1
                    [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                    pause(pause_time);
                end
                SetFwdVelAngVelCreate(serPort, fwdSpeed, 0);

            elseif(WallSensor==0 && ~bumped)
                %This state occurs when the Robot gets drifted away from
                %the wall, or it reaches a cornor. In this case the robot
                %needs to know what happened. The Wall sensor can go off
                %when the robot drifted away from the wall or got too close
                %such that the the wall shadows the IR transmitter. For the
                %case when robot drifted away from the wall, rotating CW
                %should fix the problem, so the robot first tries that. If
                %the wall sensor still does not get turned on, it rotates
                %CCW hoping it could be the other possibility. If the Wall
                %sensor still does not turn on, the robot assumes hight
                %probability that it is at the cornor and it takes on an
                %aggressive strategy of rotating CW and moving forward
                %untill it bumps. We also found a way to handle cornors
                %which is much more elegant than our current strategy. It uses
                %along a radius. But due to time constraints we could not test
                %it.
                fprintf('WallSensor = 0 and Not bumped. Lost the wall. Retracing to get the wall back\n');            

                fprintf('Rotating CW pi/6\n');
                
                SetFwdVelAngVelCreate(serPort, 0, -angSpeed);
                angle = 0;

                while(abs(angle) < pi/6 && WallSensor == 0)
                    WallSensor = WallSensorReadRoomba(serPort);
                    a2 = AngleSensorRoomba(serPort);
                    coords_old(3) = coords_old(3) + a2;
                    
                    angle = angle + abs(a2);
                    pause(pause_time);
                end
                
                if WallSensor == 1
                    fprintf('Found the Wall\n')                
                else %WallSensor == 0
                    fprintf('Rotating CCW 2*pi/6\n');
                    angle = 0;                    
                    SetFwdVelAngVelCreate(serPort, 0, angSpeed);
                    while(angle <2*pi/6 && WallSensor == 0)
                        WallSensor = WallSensorReadRoomba(serPort);
                        a2 = AngleSensorRoomba(serPort);
                        coords_old(3) = coords_old(3) + a2;
                    
                        angle = angle + abs(a2);
                        pause(pause_time);
                    end
                    
                    if WallSensor == 1
                        fprintf('Found the Wall\n');
                    else % WallSensor == 0
                        fprintf('Coming back to original orientation\n')
                        SetFwdVelAngVelCreate(serPort, 0, -angSpeed);
                        angle = 0;
                        while angle < pi/6
                            a2 = AngleSensorRoomba(serPort);
                            coords_old(3) = coords_old(3) + a2;
                    
                            angle = angle + abs(a2);
                            pause(pause_time);
                        end
                        SetFwdVelAngVelCreate(serPort, 0, 0);
                        WallSensor = WallSensorReadRoomba(serPort);

                        if WallSensor == 0
                            %Confirmed that it is at cornor
                            fprintf('Wall sensor still not found\n')
                            fprintf('Aggressive strategy\n');
                            WallSensor = WallSensorReadRoomba(serPort);
                            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                            bumped = BumpRight||BumpLeft||BumpFront;
                            while (WallSensor==0 && ~bumped)

                                fprintf('Rotating random angle CW\n');

                                SetFwdVelAngVelCreate(serPort, 0, -angSpeed);
                                angle = 0;
                                random_addition = pi/denominator;
                                while angle < random_addition
                                    a2 = AngleSensorRoomba(serPort);
                                    coords_old(3) = coords_old(3) + a2;
                    
                                    angle = angle + abs(a2);
                                    pause(pause_time);
                                end
                                
                                
                                fprintf('Moving fwd\n');
                                t1 = tic;
                                SetFwdVelAngVelCreate(serPort, fwdSpeed, 0);
                                while (tic - t1 < 200000 && WallSensor == 0 && ~bumped)
                                    WallSensor = WallSensorReadRoomba(serPort);
                                    [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                                    bumped = BumpRight||BumpLeft||BumpFront;
                                    pause(pause_time);
                                end
                                
                                distance = DistanceSensorRoomba(serPort);
        
                                turn_angle_new = AngleSensorRoomba(serPort);
                                coords_new = coords_old + [(distance*sin(coords_old(3) + turn_angle_new)) (distance * cos(coords_old(3)+ turn_angle_new)) (turn_angle_new)];
                                coords_old = coords_new;
                                total_distance = total_distance + distance;
                                
                                figure(f);
                                plot(coords_new(1), coords_new(2), 'rx');
                                hold on;


                                
                            end%while (WallSensor==0 && ~bumped)
                        end%if WallSensor == 0
                    end%if WallSensor == 1
                end%if WallSensor == 1
                SetFwdVelAngVelCreate(serPort, fwdSpeed, 0);

            elseif(WallSensor==1 && ~bumped)
                %Following the wall properly
                fprintf('Keep Going\n');
            
            elseif((WallSensor==0 && BumpFront==1) || BumpLeft==1)
                %This state handles the rare case when the robot rotates
                %too much in the agressive strategy. Here it rotates CCW 90
                %degrees if Front bump sensor was on, and CCW 180 degrees
                %if the left bump sensor was on.
                if BumpFront==1
                    fprintf('Rotating CCW max 90\n')
                   
                    angle = 0;
                    SetFwdVelAngVelCreate(serPort, 0, angSpeed);
                    pause(pause_time);
                    while (WallSensor==0 && angle < pi/2)
                        a2 = AngleSensorRoomba(serPort);
                        coords_old(3) = coords_old(3) + a2;
                    
                        angle = angle + abs(a2);
                        pause(pause_time);
                        WallSensor = WallSensorReadRoomba(serPort);
                        pause(pause_time);
                    end
                else
                    %Bump Left
                    fprintf('Rotating CCW max 180\n')
                    angle = 0;
                    SetFwdVelAngVelCreate(serPort, 0, angSpeed);
                    pause(pause_time);
                    while (WallSensor==0 && angle < pi)
                        a2 = AngleSensorRoomba(serPort);
                        angle = angle + abs(a2);
                        WallSensor = WallSensorReadRoomba(serPort);
                        pause(pause_time);
                    end
                end%if BumpFront==1
                
                SetFwdVelAngVelCreate(serPort, fwdSpeed, 0);
            end%elseif((WallSensor==0 && BumpFront==1) || BumpLeft==1)
        end%if(State == 0)
        pause(pause_time_2);
    end%While(true)
    
    SetFwdVelAngVelCreate(serPort, 0, 0);
    pause(0.1)
    % Stop the Robot
end


    
