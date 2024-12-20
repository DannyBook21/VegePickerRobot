%% Define the LinearUR5 and NewRobot models with the new 8th link
UR5 = LinearUR5();
Robot = NewRobot();
light('Position', [0, 0, -20])
%% Define the Item objects
item1 = Item('potato.ply', 1);  % First item for LinearUR5
item2 = Item('knife.ply', 2);  % Second item for NewRobot
base1 = Item('holygrail.ply', 3);
half1 = Item('potatohalf.ply', 4);
half2 = Item('potatohalf.ply', 5);
hand = Item('hand.ply', 6);
med = Item('medkit.ply', 7);
keyboard = Item('keyboard.ply', 8);
board = Item('board.ply', 9);
%% Define new XYZ coordinates for each robot (in meters)
new_x = 0; new_y = 0; new_z = 0.25;  % For LinearUR5
x2 = -0.5; y2 = 0.25; z2 = 0.25;    % For NewRobot

%% Define a custom joint configuration for both robots
q_deg = [-0.8, 0, 45, -45, 0, 90, 0, 0];  % For LinearUR5
q_deg2 = [0.1, -90, -135, -45, -90, 90, 180, 0];  % For NewRobot

%% Convert only the rotational joints to radians
q = q_deg;  
q(2:end-1) = deg2rad(q_deg(2:end-1));  % Convert joints 2-7 to radians

q2 = q_deg2;
q2(2:end-1) = deg2rad(q_deg2(2:end-1));

%% Create a homogeneous transformation matrix for the desired base position
new_base_transform = transl(new_x, new_y, new_z) * trotz(pi);
new_base_transform2 = transl(x2, y2, z2) * trotz(pi/2);

%% Create new instances of robots with the updated base transformations
UR5 = LinearUR5(new_base_transform);
Robot = NewRobot(new_base_transform2);

%% Set animation delay to 0 for faster animations
UR5.model.delay = 0;
Robot.model.delay = 0;

%% Define 8 waypoints as target positions in 3D space for each robot
waypoints1 = [
    0.60, 0.75, 0.04;   % 1st waypoint (item1 pickup)
    0.75, 0.75, 0.6;
    0.10, 0.75, 0.6;
    0.10, 0.75, 0.29;   
    0.25, 0.5, 0.6;
    0.75, 0.75, 0.6;
    0.75, 0.75, 0.25;
    0.75, 0.75, 0.25;  % Final target
];

waypoints2 = [
    -0.45, -0.12, 0.275;  % 1st waypoint (item2 pickup)
    -0.45, -0.12, 0.6;
    -0.45, -0.12, 0.6;
    0, 0.75, 0.6;
    0, 0.75, 0.27;
    0, 0.75, 0.6;
    -0.45, -0.12, 0.6;
    -0.45, -0.12, 0.275;  % Final target
];
waypoints3 = [
    -0.45, -0.12, 0.6;  
    0.10, 0.75, 0.6;
    0.10, 0.78, 0.29;
    0.05, 1.15, 0.29;
    0.10, 0.72, 0.29;
    0.05, 1.09, 0.29;
    0.10, 0.75, 0.6;
    -0.45, -0.12, 0.6;
    -0.45, -0.12, 0.275;  % Final target
];

%% Define RMRC parameters
dt = 0.03;  
lambda = 2;  
maxSteps = 120;  
max_joint_velocity = 0.5;  
stop_threshold = 1e-2;  
collision_threshold = 0.1;  % Collision distance threshold for avoiding hand object

%% Set initial item positions at their respective first waypoints
item1.model.base = transl(0.60,0.75,0.04)*trotz(pi/2);  
item1.PlotAndLoadPly(0);

item2.model.base = transl(-0.45, -0.12, 0.275);  
item2.PlotAndLoadPly(0);

base1.model.base = transl(1.1,-0.35,-0.10)*trotz(pi);  
base1.PlotAndLoadPly(0);

half1.model.base = transl(0,0,0)*trotz(pi/2);
half1.PlotAndLoadPly(0);
half2.model.base = transl(0,0,0);
half2.PlotAndLoadPly(0);

hand.model.base = transl(0.35, 0.75, 0.6)*trotz(pi/2)*troty(pi/2);
hand.PlotAndLoadPly(0);

med.model.base = transl(1.75, 0.5, -0.1);
med.PlotAndLoadPly(0);
board.model.base = transl(-0.2, 1.15, 0.25)*trotz(pi/2)*trotz(deg2rad(270));
board.PlotAndLoadPly(0);
keyboard.model.base = transl(-0.45, 1.3, 0.25);
keyboard.PlotAndLoadPly(0);

%% Set up the Central Camera for the visual servoing
pStar = [512; 512];
P = [0.60;0.75;0.04];

cam = CentralCamera('focal', 0.05, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name', 'UR5camera');

Tc0 = UR5.model.fkine(q).T;
UR5.model.animate(q);
drawnow
cam.T = Tc0;

%% Camera view and plotting
cam.plot_camera('label', 'scale', 0.05);
cam.clf()
cam.plot(pStar, '*');  % Create the camera view
cam.hold(true);
cam.plot(P, 'pose', Tc0, 'o');  % Create the camera view

pause(2);
cam.hold(true);
cam.plot(P);  % Show initial view

%% Set the camera parameters for visual servoing
lambda_vs = 2.0;  % Visual servoing gain
max_iter = 250;  % Maximum iterations
error_threshold = 1;  % Pixel error threshold to stop the motion

%% Define prismatic joint limits
prismatic_joint_limit1 = [-0.9, -0.01];  % For LinearUR5
prismatic_joint_limit2 = [0.10, 0.8];    % For NewRobot

%% RMRC loop for reducing the error between pStar and P using the camera
ksteps = 0;
while ksteps < max_iter
    ksteps = ksteps + 1;

    % Get the current end-effector transformation
    Tc_current = UR5.model.fkine(q);
    cam.T = Tc_current;

    % Compute the view of the camera
    uv = cam.plot(P);

    % Compute image plane error
    e = pStar - uv;  % Feature error (difference between desired and current points)
    error_norm = norm(e);

    % Stop if the error is below the threshold
    if error_norm < error_threshold
        disp('Error below threshold, stopping motion.');
        break;
    end

    % Update the depth dynamically based on the current camera to target distance
    depth = norm(transl(Tc_current) - 0.04);

    % Compute the desired image plane velocity
    v_camera = lambda_vs * e;

    % Get the camera Jacobian in the current frame
    J_cam = cam.visjac_p(uv, depth);

    % Compute the end-effector velocity in the task space
    v_ee = pinv(J_cam) * v_camera(:);

    % Get the Jacobian at the current joint configuration
    J_ee = UR5.model.jacob0(q);

    % Apply damping to the pseudo-inverse of the Jacobian
    lambda_damping = 0.05;
    J_damped = J_ee' * inv(J_ee * J_ee' + lambda_damping * eye(6));

    % Calculate joint velocities
    q_dot = J_damped * v_ee;

    % Limit joint velocities
    q_dot = max(min(q_dot, max_joint_velocity), -max_joint_velocity);

    % Update joint angles
    q_new = q + (q_dot' * dt);

    % Enforce prismatic joint limits
    q_new(1) = min(max(q_new(1), prismatic_joint_limit1(1)), prismatic_joint_limit1(2));

    % Update the joint angles
    q = q_new;

    % Animate the robot
    UR5.model.animate(q);
    Robot.model.animate(q2);
    workspace = [-2 2 -1 2 -0.5 1];
    axis(workspace);
    grid off;
    set(gca, 'Color', 'none');

    % Allow GUI events to process
    pause(dt/50);
end

%% Final message for the RMRC loop
if ksteps == max_iter
    disp('Maximum iterations reached, stopping motion.');
else
    disp('Camera error minimized. Visual servoing complete.');
end

%% Initialize flags for item pickups and releases
isItem1Picked = false;
isItem2Picked = false;
isItem1Cut = false;

%% Get the current end-effector positions and orientations
T_current1 = UR5.model.fkine(q);
T_current2 = Robot.model.fkine(q2);
current_pos1 = transl(T_current1);
current_pos2 = transl(T_current2);
current_rot1 = t2r(T_current1);  
current_rot2 = t2r(T_current2);  

%% Loop through each of the 8 waypoints for the first RMRC loop
for wp = 1:size(waypoints1, 1)
    % Define target positions for the current waypoint
    target_pos1 = waypoints1(wp, :);
    target_pos2 = waypoints2(wp, :);

    % Define target transformation matrices with a 180-degree rotation
    T_target1 = transl(target_pos1) * trotx(pi);
    T_target2 = transl(target_pos2) * trotx(pi);
    target_rot1 = t2r(T_target1);
    target_rot2 = t2r(T_target2);

    % RMRC loop for the current waypoint
    for step = 1:maxSteps
        % Calculate position and orientation errors
        pos_error1 = target_pos1 - current_pos1;
        rot_error_matrix1 = target_rot1 * current_rot1';
        rot_error1 = tr2rpy(rot_error_matrix1);

        pos_error2 = target_pos2 - current_pos2;
        rot_error_matrix2 = target_rot2 * current_rot2';
        rot_error2 = tr2rpy(rot_error_matrix2);

        % Concatenate position and orientation errors
        error1 = [pos_error1, rot_error1];
        error2 = [pos_error2, rot_error2];

        % Calculate desired end-effector velocities
        v_desired1 = lambda * error1;
        v_desired2 = lambda * error2;

        % Collision avoidance check for LinearUR5 to avoid hand object
        hand_pos = transl(hand.model.fkine(0));
        distance_to_hand = norm(current_pos1 - hand_pos);

        if distance_to_hand < collision_threshold
            disp('Collision detected with hand! Adjusting trajectory...');
            v_desired1(1:2) = v_desired1(1:2) * 0.3;  % Slow down the linear velocity
            avoidance_direction_z = (current_pos1(3) - hand_pos(3)) / distance_to_hand;
            v_desired1(1:3) = v_desired1(1:3) + 0.5 * avoidance_direction_z;
        end

        % Get the Jacobians at the current joint configurations
        J1 = UR5.model.jacob0(q);
        J2 = Robot.model.jacob0(q2);

        % Apply damping to the pseudo-inverses of the Jacobians
        lambda_damping = 0.01;
        J_damped1 = J1' * inv(J1 * J1' + lambda_damping * eye(6));
        J_damped2 = J2' * inv(J2 * J2' + lambda_damping * eye(6));

        % Calculate joint velocities
        q_dot1 = J_damped1 * v_desired1';
        q_dot2 = J_damped2 * v_desired2';

        % Limit joint velocities
        q_dot1 = max(min(q_dot1, max_joint_velocity), -max_joint_velocity);
        q_dot2 = max(min(q_dot2, max_joint_velocity), -max_joint_velocity);

        % Update joint angles
        q_new1 = q + (q_dot1' * dt);
        q_new2 = q2 + (q_dot2' * dt);

        % Enforce prismatic joint limits
        q_new1(1) = min(max(q_new1(1), prismatic_joint_limit1(1)), prismatic_joint_limit1(2));
        q_new2(1) = min(max(q_new2(1), prismatic_joint_limit2(1)), prismatic_joint_limit2(2));

        % Update the joint angles
        q = q_new1;
        q2 = q_new2;

        % Update the current end-effector positions and orientations
        T_current1 = UR5.model.fkine(q);
        T_current2 = Robot.model.fkine(q2);
        current_pos1 = transl(T_current1);
        current_pos2 = transl(T_current2);
        current_rot1 = t2r(T_current1);
        current_rot2 = t2r(T_current2);

        % Animate both robots
        UR5.model.animate(q);
        Robot.model.animate(q2);
        workspace = [-2 2 -1 2 -0.5 1];
        axis(workspace);
        grid off;
        set(gca, 'Color', 'none');

        % Check for item pickups and releases
        if wp == 2 && ~isItem1Picked
            item1_pos = transl(item1.model.fkine(0));
            distance1 = norm(current_pos1 - item1_pos);
            if distance1 < 0.1
                disp('Item 1 picked up!');
                isItem1Picked = true;
            end
        end

        % Make item1 follow the end-effector of LinearUR5
        if isItem1Picked && wp <= 5
            item1.model.base = T_current1;
            item1.PlotAndLoadPly(0);
        end

        % Release item1 at the 5th waypoint
        if wp == 5 && isItem1Picked
            disp('Item 1 released!');
            isItem1Picked = false;
        end

        % Handle item cutting at waypoint 6
        if wp == 6 && ~isItem1Cut
            disp('Item Cut!');
            half1.model.base = transl(0.10, 0.72, 0.29)*trotz(pi);
            half2.model.base = transl(0.10, 0.78, 0.29);
            half1.PlotAndLoadPly(0);
            half2.PlotAndLoadPly(0);
            item1.model.base = transl(0,0,0);
            item1.PlotAndLoadPly(0);
            isItem1Cut = true;
        end

        % Check if item2 is picked up at the first waypoint
        if wp == 2 && ~isItem2Picked
            item2_pos = transl(item2.model.fkine(0));
            distance2 = norm(current_pos2 - item2_pos);
            if distance2 < 0.05
                disp('Item 2 picked up!');
                isItem2Picked = true;
            end
        end

        % Make item2 follow the end-effector of NewRobot
        if isItem2Picked && wp <= 8
            item2.model.base = T_current2;
            item2.PlotAndLoadPly(0);
        end

        % Allow GUI events to process
        pause(dt/15);
    end
end

%% Final message for the first RMRC loop
disp('All 8 waypoints reached by both robots. RMRC trajectory complete.');

%% Define flags for half1 and half2 pickups and releases
isHalf1Picked = false;
isHalf2Picked = false;

%% Get the initial positions and orientations for the second RMRC loop
T_current3 = Robot.model.fkine(q2);
current_pos3 = transl(T_current3);
current_rot3 = t2r(T_current3);

%% Loop through each of the 9 waypoints for the second RMRC loop
for wp = 1:size(waypoints3, 1)
    % Define the target position for the current waypoint
    target_pos3 = waypoints3(wp, :);

    % Define the target transformation matrix with a 180-degree rotation
    T_target3 = transl(target_pos3) * trotx(pi);
    target_rot3 = t2r(T_target3);

    % RMRC loop for the current waypoint
    for step = 1:maxSteps
        % Calculate position and orientation errors
        pos_error3 = target_pos3 - current_pos3;
        rot_error_matrix3 = target_rot3 * current_rot3';
        rot_error3 = tr2rpy(rot_error_matrix3);

        % Concatenate position and orientation errors
        error3 = [pos_error3, rot_error3];

        % Calculate desired end-effector velocity
        v_desired3 = lambda * error3;

        % Get the Jacobian at the current joint configuration
        J3 = Robot.model.jacob0(q2);

        % Apply damping to the pseudo-inverse of the Jacobian
        J_damped3 = J3' * inv(J3 * J3' + lambda_damping * eye(6));

        % Calculate joint velocities
        q_dot3 = J_damped3 * v_desired3';

        % Limit joint velocities
        q_dot3 = max(min(q_dot3, max_joint_velocity), -max_joint_velocity);

        % Update joint angles
        q_new3 = q2 + (q_dot3' * dt);

        % Enforce prismatic joint limits
        q_new3(1) = min(max(q_new3(1), prismatic_joint_limit2(1)), prismatic_joint_limit2(2));

        % Update the joint angles
        q2 = q_new3;

        % Update the current end-effector position and orientation
        T_current3 = Robot.model.fkine(q2);
        current_pos3 = transl(T_current3);
        current_rot3 = t2r(T_current3);

        % Animate the robot
        Robot.model.animate(q2);
        workspace = [-2 2 -1 2 -0.5 1];
        axis(workspace);
        grid off;
        set(gca, 'Color', 'none');

        % Handle half pickups and releases
        if wp == 4 && ~isHalf2Picked
            half2_pos = transl(half2.model.fkine(0));
            distance_half2 = norm(current_pos3 - half2_pos);
            if distance_half2 < 0.05
                disp('Half 2 picked up!');
                isHalf2Picked = true;
            end
        end

        % Make half1 follow the end-effector from waypoint 4 to waypoint 5
        if isHalf2Picked && wp <= 5
            half2.model.base = T_current3;
            half2.PlotAndLoadPly(0);
        end

        % Release half1 at waypoint 5
        if wp == 5 && isHalf2Picked
            disp('Half 2 released!');
            isHalf2Picked = false;
        end

        % Pick up half2 at waypoint 6
        if wp == 6 && ~isHalf1Picked
            half1_pos = transl(half1.model.fkine(0));
            distance_half1 = norm(current_pos3 - half1_pos);
            if distance_half1 < 0.05
                disp('Half 1 picked up!');
                isHalf1Picked = true;
            end
        end

        % Make half2 follow the end-effector from waypoint 6 to waypoint 7
        if isHalf1Picked && wp <= 7
            half1.model.base = T_current3;
            half1.PlotAndLoadPly(0);
        end

        % Release half2 at waypoint 7
        if wp == 7 && isHalf1Picked
            disp('Half 1 released!');
            isHalf1Picked = false;
        end

        % Allow GUI events to process
        pause(dt/5);
    end

    disp(['Waypoint ' num2str(wp) ' reached for the second RMRC loop.']);
end

%% Final message for the second RMRC loop
disp('All 9 waypoints reached for the second RMRC loop.');
