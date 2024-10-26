function LabWithCustomGUI_ESTOP_NewRobot()
    %% LabWithCustomGUI_ESTOP_NewRobot - NewRobot with Custom GUI and E-STOP Button
    % This function creates a NewRobot model and a custom GUI with sliders
    % to adjust each joint angle. Separate E-STOP Engage and Resume buttons are added.

    % Create the NewRobot model
    baseTr = eye(4); % Adjust base transformation if needed
    myRobot = NewRobot(baseTr);

    % Get the number of joints
    numJoints = length(myRobot.model.links);

    % Limit to a maximum of 7 joints in the GUI
    displayJoints = min(7, numJoints);

    % Set initial joint angles to zero
    q = zeros(1, numJoints);

    % Plot the robot in a new figure
    robotFig = figure('Name', 'NewRobot Visualization', 'NumberTitle', 'off');
    myRobot.model.plot(q, 'workspace', [-1 1 -1 1 -0.5 1.5]);

    % Create the custom GUI for joint control with separate E-STOP buttons
    CreateJointControlGUI(myRobot);

    %% Function Definitions

    % Function to create the GUI with sliders for each joint and separate E-STOP buttons
    function CreateJointControlGUI(robotModel)
        % Create a figure for the GUI
        guiFig = figure('Name', 'Robot Joint Control', 'NumberTitle', 'off', ...
            'Position', [100, 100, 400, 150 + 50*displayJoints], ...
            'CloseRequestFcn', @closeGUI); % Close function

        % Initialize E-STOP state
        estopState = true;  % E-STOP is engaged by default

        % Create E-STOP Engage button
        engageButton = uicontrol('Parent', guiFig, 'Style', 'pushbutton', ...
            'Position', [50, 60 + displayJoints*50, 120, 40], ...
            'String', 'E-STOP ENGAGE', ...
            'FontSize', 12, 'BackgroundColor', 'red', 'ForegroundColor', 'white', ...
            'Callback', @engageESTOP);

        % Create E-STOP Resume button
        resumeButton = uicontrol('Parent', guiFig, 'Style', 'pushbutton', ...
            'Position', [230, 60 + displayJoints*50, 120, 40], ...
            'String', 'E-STOP RESUME', ...
            'FontSize', 12, 'BackgroundColor', 'green', 'ForegroundColor', 'white', ...
            'Callback', @resumeESTOP, 'Enable', 'off');

        % Create sliders and text boxes for each joint up to the 7th joint
        for i = 1:displayJoints
            % Get joint limits
            qlim = robotModel.model.links(i).qlim;
            if isempty(qlim) || qlim(1) >= qlim(2)
                % Set default limits if qlim is empty or invalid
                qlim = [-pi, pi];
            end

            % Create label for the slider
            uicontrol('Parent', guiFig, 'Style', 'text', ...
                'Position', [20, 50 + (displayJoints - i)*50, 60, 20], ...
                'String', ['Joint ' num2str(i)]);

            % Create slider
            uicontrol('Parent', guiFig, 'Style', 'slider', ...
                'Min', qlim(1), 'Max', qlim(2), 'Value', max(min(q(i), qlim(2)), qlim(1)), ...
                'Position', [90, 50 + (displayJoints - i)*50, 200, 20], ...
                'Tag', ['slider' num2str(i)], 'Enable', 'off', ...
                'Callback', @sliderCallback);

            % Create text box to display joint angle
            uicontrol('Parent', guiFig, 'Style', 'edit', ...
                'Position', [300, 50 + (displayJoints - i)*50, 80, 20], ...
                'Tag', ['edit' num2str(i)], 'Enable', 'off', ...
                'String', num2str(q(i)), ...
                'Callback', @editCallback);
        end

        % Engage E-STOP callback function
        function engageESTOP(~, ~)
            estopState = true;
            set(engageButton, 'Enable', 'off');
            set(resumeButton, 'Enable', 'on');
            % Disable sliders and text boxes
            setControlsEnabled('off');
        end

        % Resume E-STOP callback function
        function resumeESTOP(~, ~)
            estopState = false;
            set(engageButton, 'Enable', 'on');
            set(resumeButton, 'Enable', 'off');
            % Enable sliders and text boxes
            setControlsEnabled('on');
        end

        % Helper function to enable/disable sliders and edit boxes
        function setControlsEnabled(enableState)
            for i = 1:displayJoints
                slider = findobj('Tag', ['slider' num2str(i)]);
                editBox = findobj('Tag', ['edit' num2str(i)]);
                if ishandle(slider) && ishandle(editBox) % Check if handles are valid
                    set(slider, 'Enable', enableState);
                    set(editBox, 'Enable', enableState);
                end
            end
        end

        % Slider callback function
        function sliderCallback(~, ~)
            if estopState || ~isvalid(guiFig) || ~isvalid(robotFig)
                return; % Do not update if E-STOP is engaged or either figure is closed
            end

            % Update joint angles and edit boxes
            for i = 1:displayJoints
                slider = findobj('Tag', ['slider' num2str(i)]);
                if ishandle(slider) % Check if handle is valid
                    q(i) = get(slider, 'Value');
                    editBox = findobj('Tag', ['edit' num2str(i)]);
                    if ishandle(editBox)
                        set(editBox, 'String', num2str(q(i)));
                    end
                end
            end

            % Update robot animation only if the figure is valid
            if isvalid(robotFig) && isvalid(robotModel.model)
                robotModel.model.animate(q);
                drawnow();
            end
        end

        % Edit box callback function
        function editCallback(~, ~)
            if estopState || ~isvalid(guiFig) || ~isvalid(robotFig)
                return; % Do not update if E-STOP is engaged or either figure is closed
            end

            % Update joint angles and sliders
            for i = 1:displayJoints
                editBox = findobj('Tag', ['edit' num2str(i)]);
                slider = findobj('Tag', ['slider' num2str(i)]);
                if ishandle(editBox) && ishandle(slider) % Check if handles are valid
                    qVal = str2double(get(editBox, 'String'));
                    qlim = robotModel.model.links(i).qlim;
                    qVal = max(min(qVal, qlim(2)), qlim(1));
                    q(i) = qVal;
                    set(slider, 'Value', qVal);
                    set(editBox, 'String', num2str(qVal));
                end
            end

            % Update robot animation only if the figure is valid
            if isvalid(robotFig) && isvalid(robotModel.model)
                robotModel.model.animate(q);
                drawnow();
            end
        end

        % Close GUI function to prevent crashes after closing
        function closeGUI(~, ~)
            delete(guiFig);
            if isvalid(robotFig)
                close(robotFig);
            end
        end
    end
end
