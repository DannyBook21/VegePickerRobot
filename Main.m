% Define the LinearUR5 robot model with the new 8th link
UR5 = LinearUR5();
Robot = NewRobot();


% Define the Item objects
item1 = Item('potato.ply', 1);  % First item for LinearUR5
item2 = Item('knife.ply', 2);  % Second item for NewRobot
base1 = Item('holygrail.ply', 3);

% Define new XYZ coordinates (in meters)
new_x = 0; new_y = 0; new_z = 0.25;  % For LinearUR5
x2 = -0.5; y2 = 0.25; z2 = 0.25;    % For NewRobot

% Define a custom joint configuration
q_deg = [-0.8, 0, 45, -90, -45, 90, 0, 0];  % For LinearUR5
q_deg2 = [0.1, -90, -135, -45, -90, 90, 180, 0];  % For NewRobot

% Convert only the rotational joints to radians
q = q_deg;  
q(2:end-1) = deg2rad(q_deg(2:end-1));  % Convert joints 2-7 to radians

q2 = q_deg2;
q2(2:end-1) = deg2rad(q_deg2(2:end-1));

% Create a homogeneous transformation matrix for the desired base position
new_base_transform = transl(new_x, new_y, new_z) * trotz(pi);
new_base_transform2 = transl(x2, y2, z2) * trotz(pi/2);

% Create new instances of robots with the updated base transformations

UR5 = LinearUR5(new_base_transform);
Robot = NewRobot(new_base_transform2);

% Set animation delay to 0 for faster animations
UR5.model.delay = 0;
Robot.model.delay = 0;



% Define 8 waypoints as target positions in 3D space for each robot
waypoints1 = [
    0.75, 0.75, 0.04;   % 1st waypoint (item1 pickup)
    0.75, 0.75, 0.6;
    0.15, 0.75, 0.6;
    0.15, 0.75, 0.29;   
    0.25, 0.5, 0.6;
    0.75, 0.75, 0.6;
    0.75, 0.75, 0.25;
    0.75, 0.75, 0.25;  % Final target
];

waypoints2 = [
    -0.45, -0.12, 0.25;  % 1st waypoint (item2 pickup)
    -0.45, -0.12, 0.6;
    -0.45, -0.12, 0.6;
    0, 0.75, 0.6;
    0, 0.75, 0.27;
    0, 0.75, 0.6;
    -0.45, -0.12, 0.6;
    -0.45, -0.12, 0.25;  % Final target
];

% Define RMRC parameters
dt = 0.03;  
lambda = 2;  
maxSteps = 90;  
max_joint_velocity = 0.5;  
stop_threshold = 1e-4;  

% Set initial item positions at their respective first waypoints
item1.model.base = transl(waypoints1(1, :));  
item1.PlotAndLoadPly(0);

item2.model.base = transl(waypoints2(1, :));  
item2.PlotAndLoadPly(0);

base1.model.base = transl(1.1,-0.375,-0.10)*trotz(pi);  
base1.PlotAndLoadPly(0);



% Flags for item pickups and releases
isItem1Picked = false;
isItem2Picked = false;

% Get the current end-effector positions and orientations
T_current1 = UR5.model.fkine(q);
T_current2 = Robot.model.fkine(q2);
current_pos1 = transl(T_current1);
current_pos2 = transl(T_current2);
current_rot1 = t2r(T_current1);  
current_rot2 = t2r(T_current2);  

% Define prismatic joint limits
prismatic_joint_limit1 = [-0.8, -0.01];  % For LinearUR5
prismatic_joint_limit2 = [0.10, 0.8];    % For NewRobot

% Loop through each of the 8 waypoints
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
        % Calculate position errors
        pos_error1 = target_pos1 - current_pos1;
        pos_error2 = target_pos2 - current_pos2;

        % Calculate orientation errors
        rot_error_matrix1 = target_rot1 * current_rot1';
        rot_error_matrix2 = target_rot2 * current_rot2';
        rot_error1 = tr2rpy(rot_error_matrix1);
        rot_error2 = tr2rpy(rot_error_matrix2);

        % Concatenate position and orientation errors
        error1 = [pos_error1, rot_error1];
        error2 = [pos_error2, rot_error2];

        % Calculate desired end-effector velocities
        v_desired1 = lambda * error1;
        v_desired2 = lambda * error2;

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
        

        % Check if item1 is picked up at the first waypoint
        if wp == 1 && ~isItem1Picked
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

        % Check if item2 is picked up at the first waypoint
        if wp == 1 && ~isItem2Picked
            item2_pos = transl(item2.model.fkine(0));
            distance2 = norm(current_pos2 - item2_pos);

            if distance2 < 0.1
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
        pause(dt/10);  % Adjust to control visualization speed
    end
end

% Final message
disp('All 8 waypoints reached by both robots. RMRC trajectory complete.');
