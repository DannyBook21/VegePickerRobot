classdef Item < handle
    % This class represents an item that can be visualized, moved, and checked for collisions.
    % The item is visualized using a .ply or .stl file and a simple single-link robot model (SerialLink).
    
    properties
        model;          % SerialLink model representing the item's position and orientation
        fileName;       % Path to the .ply or .stl file used for visualizing the item
        itemNumber;     % A unique identifier or number for each item instance
        itemHandle;     % Handle to the plotted item for updating its position in the plot
        faceData;       % Stores face data from the file
        vertexData;     % Stores vertex data from the file
        vertexColors;   % Stores color data from the file
    end

    methods
        % Constructor: Initializes the item object with its file and a unique item number
        function self = Item(fileName, itemNumber)
            % Save the file name and item number to the object's properties
            self.fileName = fileName;
            self.itemNumber = itemNumber;

            % Create a simple robot link (revolute) to represent the item in space
            L = Link('d', 0, 'a', 0, 'alpha', 0);  % Revolute link with no offset
            
            % Create a SerialLink model for the item using a single revolute link
            self.model = SerialLink(L, 'name', ['Item' num2str(itemNumber)]);  % Name includes the item number
            self.model.plotopt = {'nojoints', 'nobase', 'noname', 'noarrow'};  % Disable unnecessary visuals for the item
            
            % Initialize the itemHandle to empty, which will be used for the plot later
            self.itemHandle = [];

            % Load the file data once
            self.LoadFileData();
        end

        % Method to load file data once and store it in properties
        function LoadFileData(self)
            [~, ~, ext] = fileparts(self.fileName);  % Get the file extension

            switch lower(ext)
                case '.ply'
                    self.LoadPlyData();  % Load PLY data
                case '.stl'
                    self.LoadStlData();  % Load STL data
                otherwise
                    warning('Unsupported file format. Please use .ply or .stl files.');
            end
        end

        % Method to load .ply data and store it in properties
        function LoadPlyData(self)
            if exist(self.fileName, 'file')
                [self.faceData, self.vertexData, plyData] = plyread(self.fileName, 'tri');
                % Extract color data from the PLY file
                if isfield(plyData.vertex, 'red')
                    self.vertexColors = [plyData.vertex.red, plyData.vertex.green, plyData.vertex.blue] / 255;
                else
                    self.vertexColors = repmat([0.5, 0.5, 0.5], size(self.vertexData, 1), 1);  % Default grey
                end
            else
                warning('PLY file not found.');
            end
        end

        % Method to load .stl data and store it in properties
        function LoadStlData(self)
            if exist(self.fileName, 'file')
                try
                    % Try to use built-in stlread if available
                    fv = stlread(self.fileName);
                    self.vertexData = fv.Points;           % Extract vertices
                    self.faceData = fv.ConnectivityList;   % Extract faces
                catch
                    % Use importGeometry as a fallback
                    model = createpde();
                    importGeometry(model, self.fileName);
                    self.vertexData = model.Geometry.Vertices;
                    self.faceData = model.Geometry.Elements';
                end

                % Set default grey color for STL files
                self.vertexColors = repmat([0.5, 0.5, 0.5], size(self.vertexData, 1), 1);
            else
                warning('STL file not found.');
            end
        end

        % Method to plot the item and update its position
        function PlotAndLoadPly(self, q)
            % Get the transformation matrix for the item using forward kinematics (fkine)
            tr = self.model.fkine(q).T;  % Calculate the 4x4 transformation matrix

            % Convert the vertex data to homogeneous coordinates (add a 4th dimension with value 1)
            numVertices = size(self.vertexData, 1);
            vertexDataHomogeneous = [self.vertexData, ones(numVertices, 1)]';  % Create a 4xN matrix

            % Apply the transformation matrix to the vertex data (transforms the item in space)
            transformedVertexData = tr * vertexDataHomogeneous;  % Apply the transformation
            transformedVertexData = transformedVertexData(1:3, :)';  % Convert back to Nx3 (remove the 4th row)

            % If the item hasn't been plotted yet, plot it
            if isempty(self.itemHandle)
                hold on;  % Hold the plot to add multiple objects
                self.itemHandle = trisurf(self.faceData, transformedVertexData(:,1), transformedVertexData(:,2), transformedVertexData(:,3), ...
                    'FaceVertexCData', self.vertexColors, ...  % Apply color from the file
                    'EdgeColor', 'none', 'FaceColor', 'interp');  % Smooth shading without edges
                axis equal;  % Ensure that the plot scales equally in all directions
                hold off;  % Release the plot hold
            else
                % Update the vertex positions of the plotted item to reflect its new position
                self.itemHandle.Vertices = transformedVertexData;
            end

            % Update the plot immediately
            drawnow;
        end

        % Method to check for collisions with another object
        function isCollision = CheckCollision(self, otherPosition, threshold)
            % Get the current position of this item using forward kinematics
            currentPosition = self.model.fkine(0).t;  % Get the current translation vector (position)

            % Calculate the distance between this item and the other object
            distance = norm(currentPosition - otherPosition);  % Compute Euclidean distance between the two points

            % Check if the distance is below the specified threshold for a collision
            if distance < threshold
                isCollision = true;  % A collision is detected
                disp(['Collision detected with Item ' num2str(self.itemNumber)]);  % Display a message
            else
                isCollision = false;  % No collision
            end
        end
    end
end
