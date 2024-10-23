classdef NewRobot < RobotBaseClass
    %% NewRobot 4kg payload robot model with linear rail
    % This class defines a NewRobot robot model with an additional prismatic link for a linear rail
   

    properties(Access = public)   
        plyFileNameStem = 'NewRobot';  % Base name for the .ply files to load the 3D models of the robot
    end
    
    methods
        %% Constructor
        % Initializes the robot model and sets the base transformation and tool options if provided
        function self = NewRobot(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end
          

        %% CreateModel
        % This function defines the kinematic structure of the robot using Denavit-Hartenberg (DH) parameters.
       
        function CreateModel(self)
           
            link(1) = Link([pi 0 0 pi/2 1]);  % Prismatic link
            link(1).qlim = [0.10 0.8];          
            link(2) = Link('d',0.15,'a',0,'alpha',pi/2,'qlim',deg2rad([-360, 360]), 'offset',0);           
            link(3) = Link('d',0,'a',-0.25,'alpha',0,'qlim', deg2rad([0, -180]), 'offset',0);            
            link(4) = Link('d',0,'a',-0.215,'alpha',0,'qlim', deg2rad([-170, 170]), 'offset', 0);                    
            link(5) = Link('d',0.15,'a',0,'alpha',pi/2,'qlim',deg2rad([-360, 360]),'offset', 0);            
            link(6) = Link('d',0.11,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360, 360]), 'offset',0);                       
            link(7) = Link('d',0.1,'a',0,'alpha',0,'qlim',deg2rad([0, 0]), 'offset', 0);        
      
            link(8) = Link('d',0.1,'a',0,'alpha',deg2rad(0),'qlim',deg2rad([0, 0]), 'offset', 0); 
            
            % Create the full SerialLink model by combining all the links into the robot model
            self.model = SerialLink(link,'name',self.name);
            

        end  
    end
end
