%% Visual Servoing with Dynamic Depth Adjustment
% Define the LinearUR5 robot model with the new 8th link
UR5 = LinearUR5();
Robot = NewRobot();

% Define the Item objects
item1 = Item('potato.ply', 1);  % First item for LinearUR5
item2 = Item('knife.ply', 2);  % Second item for NewRobot
base1 = Item('holygrail.ply', 3);
half1 = Item('potatohalf.ply', 4);
half2 = Item('potatohalf.ply', 5);
hand = Item('hand.ply', 6);
% Define new XYZ coordinates (in meters)
new_x = 0; new_y = 0; new_z = 0.25;  % For LinearUR5
x2 = -0.5; y2 = 0.25; z2 = 0.25;    % For NewRobot

% Define a custom joint configuration
q_deg = [-0.8, 0, 45, -45, 0, 90, 0, 0];  % For LinearUR5
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
    0.75, 0.85, 0.04;   % 1st waypoint (item1 pickup)
    0.75, 0.75, 0.6;
    0.10, 0.75, 0.6;
    0.10, 0.75, 0.29;   
    0.25, 0.5, 0.6;
    0.75, 0.75, 0.6;
    0.75, 0.75, 0.25;
    0.75, 0.75, 0.25;  % Final target
];

% Set the camera parameters
cam = CentralCamera('focal', 0.05, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');

% Set initial item positions at their respective first waypoints
item1.model.base = transl(0.75,0.85,0.04);  
item1.PlotAndLoadPly(0);

% Set up the initial camera position and joint configuration
Tc0 = UR5.model.fkine(q).T;
UR5.model.animate(q);
cam.T = Tc0;

% Display the initial camera setup
cam.plot_camera('label', 'scale', 0.15);
cam.clf()

% Define the desired feature points in the image plane
pStar = [512; 512];

% Depth will be dynamically updated based on distance between camera and target
lambda_vs = 1.0;  % Reduce visual servoing gain for more stability
max_iter = 500; % Maximum iterations to avoid infinite loop
error_threshold = 0.1;  % Pixel error threshold to stop the motion

% RMRC loop for reducing the error between pStar and P using the camera
ksteps = 0;
while ksteps < max_iter
    ksteps = ksteps + 1;
    
    % Get the current end-effector transformation
    Tc_current = UR5.model.fkine(q);
    cam.T = Tc_current;
    
    % Compute the current view of the camera
    P = [0.75; 0.85; 0.04];  % Define the initial target point in 3D space
    uv = cam.plot(P);
    
    % Compute image plane error
    e = pStar - uv;   % Feature error (difference between desired and current points)
    error_norm = norm(e);
    
    % Stop if the error is below the threshold
    if error_norm < error_threshold
        disp('Error below threshold, stopping motion.');
        break;
    end
    
    % Update the depth dynamically based on the current camera to target distance
    depth = norm(transl(Tc_current) - P);
    
    % Compute the desired image plane velocity
    v_camera = lambda_vs * e;
      
    % Get the camera Jacobian in the current frame
    J_cam = cam.visjac_p(uv, depth);
    
    % Compute the end-effector velocity in the task space
    v_ee = pinv(J_cam) * v_camera(:);  % Ensure v_camera is treated as a column vector for matrix multiplication
   
    % Get the Jacobian at the current joint configuration
    J_ee = UR5.model.jacob0(q);
    
    % Apply damping to the pseudo-inverse of the Jacobian
    lambda_damping = 0.05;  % Adjust damping for better convergence
    J_damped = J_ee' * inv(J_ee * J_ee' + lambda_damping * eye(6));
    
    % Calculate joint velocities
    q_dot = J_damped * v_ee;
   
    % Limit joint velocities
    max_joint_velocity = 0.5;
    q_dot = max(min(q_dot, max_joint_velocity), -max_joint_velocity);
    
    % Update joint angles
    dt = 0.03;
    q = q + (q_dot' * dt);
    
    % Animate the robot
    UR5.model.animate(q);
    
    % Allow GUI events to process
    pause(dt/50);
end

% Final message
if ksteps == max_iter
    disp('Maximum iterations reached, stopping motion.');
else
    disp('Camera error minimized. Visual servoing complete.');
end
