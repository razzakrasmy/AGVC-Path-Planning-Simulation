%% Your information 
disp("   Student name : Abdul Razzak Rasmy");
disp("   Student ID : 222060441");
disp("   Project code SEN771 T1 2023");

pause(1);
%% Field
fig = figure;  % create a new figure
set(fig, 'WindowState', 'maximized');  % maximize the figure
Area=[0 0 120 90];
rectangle('Position',Area)
axis([-20 140 -20 110]) ;
xlabel("x(m)");
ylabel("y(m)");
title('َAGVC simulation field')
hold on
%% Mesh
MeshSize=0.5; % 0.5 meter
hold on
% plot horizontal mesh lines
for i=0:MeshSize:Area(4)
    %plot([0 Area(3)], [i i],'r');
end
% plot veritcal mesh lines
for j=0:MeshSize:Area(3)
   % plot([j j],[0 Area(4)],'r');
end
disp('Grids are shown');
pause(1)
%% cell cntre
FieldX=[0:MeshSize:Area(3)];
FieldY=[0:MeshSize:Area(4)];
CellsX=FieldX(1:end-1)+MeshSize; %fieldx(1 :end-1) - means from 1st element until one before last we are adding half of mess size to those elements
CellsY=FieldY(1:end-1)+MeshSize;



%% field with obstacle and target point
rectangle('Position',Area)
axis([-20 140 -20 110 ])
hold on
CheckCells=zeros(size(CellsX,2),size(CellsY,2)); %This matrix will keep record of cells that the robot has checked ( previously visited)
CheckObstacles=zeros(size(CheckCells));   % This matrix will show cells that are filled with obstacles




%% Obtacles dimentions
%get information from user or generate it randomly
% ObsWidth=input("entre and randum value between 0-8m (fractions is acceptable): ");
% generate random number
ObsWidth=round(8*rand()); %to  create a random integer number between 0-8 meters rand(), creates a uniform random number between 0-1
ObsLen=2*ObsWidth;

%% Obstacle locations
O(:,1)=[30,70]';
O(:,2)=[30, 45]';
O(:,3)=[30, 20]';
O(:,4)=[45,60]';
O(:,5)=[45,30]';
O(:,6)=[60,80]' ;
O(:,7)=[60,65]';
O(:,8)=[60,25]';
O(:,9)=[60,10]';
O(:,10)=[75,69]';
O(:,11)=[75,30]';
O(:,12)=[90,70]';
O(:,13)=[90,45]';
O(:,14)=[90,20]';
O(:,15)=[109,45]';
plot(O(1,:),O(2,:),'om');% O is an array with 15 positions.X and y pos are given as in rows.X is row 1 and row 2 are y coordinates

%% plot obstacles
for i=1:15
    if(i<6 || i>9)   % Thin rectangles
        rectangle('Position',[O(1, i)-ObsWidth/2 O(2,i)-ObsWidth, ObsWidth,ObsLen],'FaceColor',[1 0 0],'EdgeColor','r');%this function makes a rectangle in the form of [starting point- x,y,width,length/height]
        CheckObstacles((O(1,i)-ObsWidth/2)*2:(O(1,i) + ObsWidth/2)*2,(O(2,i)-ObsWidth)*2:(O(2,i)+ObsWidth)*2)=1; %making the cells which obstacles occupy as 1 to make sure our robot doesn't move to these cells

    else          % Fat rectangles
        rectangle('Position',[O(1,i)-ObsWidth O(2,i)-ObsWidth/2, ObsLen, ObsWidth],'FaceColor',[1 0 0],'EdgeColor','r');
        CheckObstacles((O(1,i)-ObsWidth)*2:(O(1,i) + ObsWidth)*2,(O(2,i)-ObsWidth/2)*2:(O(2,i)+ObsWidth/2)*2)=1;
    end

end


%% Targets
Target(:,1)=[119.5,54.5]';
Target(:,2)=[82.5, 15.5]';
Target(:,3)=[82.5, 75.5]';
Target(:,4)=[60.5, 54.5]';
plot(Target(1,:),Target(2,:),'og','LineWidth',1);


Target_2x(:,1)=[239,109]';   %targets position is 2x cell position now as we split the cell to two for one unit position.0.5m mesh making cells to increase.
Target_2x(:,2)=[165, 31]';
Target_2x(:,3)=[165, 151]';
Target_2x(:,4)=[121, 109]';

%%  Robot position
RobotStartCell=[1,91]';
CheckCells=CheckObstacles;


RobotPose= RobotStartCell;
Roh=plot(RobotPose(1)/2,RobotPose(2)/2,'ko')
%% A* Algoritm for multiple targets
PathInd=1;
Path(PathInd).Cell=RobotPose; %% mapping cells at which robot moves in.

TargetSequence=[4 2 3 1]; %order in which targets are reached,order which has the lowest path length
TargetFound(size(TargetSequence))=false; %initializing target founf to be false .
CheckCells((RobotPose(1)),(RobotPose(2)))=1; %checking out cells to 1 to add to the cells which robot has moved



TargetIndex=1;
Selected_TargetCell=Target_2x(:,TargetSequence(TargetIndex));
while(sum(~TargetFound))
    AvailableNeighburQueue=[];
    NextCellFound=false;
    % check cell in the left side
    if (RobotPose(1)>1)
        CellToCheck=RobotPose-[1 0]';
        if (CheckCells(CellToCheck(1),CellToCheck(2))==0)
            AvailableNeighburQueue=[AvailableNeighburQueue CellToCheck];
            %NextCellFound=true;
        end
    end

    % Check the cell in down
    if (RobotPose(2)>1)
        CellToCheck=RobotPose-[0 1]';
        if (CheckCells(CellToCheck(1),CellToCheck(2))==0)
            AvailableNeighburQueue=[AvailableNeighburQueue CellToCheck];
            %NextCellFound=true;
        end
    end
    % check cell in the right side
    if (RobotPose(1)<size(CellsX,2))
        CellToCheck=RobotPose+[1 0]';
        if (CheckCells(CellToCheck(1),CellToCheck(2))==0)
            AvailableNeighburQueue=[AvailableNeighburQueue CellToCheck];
            %NextCellFound=true;
        end
    end
    % check cell in the up
    if (RobotPose(2)<size(CellsY,2))
        CellToCheck=RobotPose+[0 1]';
        if (CheckCells(CellToCheck(1),CellToCheck(2))==0)
            AvailableNeighburQueue=[AvailableNeighburQueue CellToCheck];
            %NextCellFound=true;
        end
    end

    Roh=plot(CellsX(RobotPose(1)),CellsY(RobotPose(2)),'ko');

    %checks if the AvailableNeighburQueue is empty, if it's the case it doesn't
    %have a cell to move from its current location .hence progrm will
    %terminate
    if (isempty(AvailableNeighburQueue))
        if (sum(find(CheckCells-1))==0)
            disp("Target not found - All cells checked- Program terminated")
            break;
        else
            disp("No cell available, back track the path")
            Path(PathInd).Cell=[];
            PathInd=PathInd-1;

            RobotPose=Path(PathInd).Cell;
            Roh=plot(CellsX(RobotPose(1)),CellsY(RobotPose(2)),'yo');
            pause(0.1);
            delete(Roh)
        end
    else

        DistanceVec=[];
        DisToStart=[];



        for i=1:size(AvailableNeighburQueue,2)
            DistanceVec(i)=norm(AvailableNeighburQueue(:,i)-Selected_TargetCell);
            DisToStart(i)=norm(AvailableNeighburQueue(:,i)-RobotStartCell);
        end
        DistanceVec=DistanceVec+0.5*DisToStart; %length of distance from start and distance of its target.
        [mindis, indx]=min(DistanceVec);

        RobotPose=AvailableNeighburQueue(:,indx);
        PathInd=PathInd+1;
        Path(PathInd).Cell=RobotPose;
        CheckCells(RobotPose(1),RobotPose(2))=1;
        pause(0.1);
        delete(Roh)
        plot(CellsX(RobotPose(1)),CellsY(RobotPose(2)),'k.');
        if (norm(RobotPose-Selected_TargetCell)==0) %checks if the rbot has reached its target cell

            TargetFound(TargetIndex)=true;

            if (TargetIndex==1)      %checking tpo find out which target has been found by looking at the target index and then incrementing the target index to move to next target
                disp('1st Target Found')

                TargetIndex=2;
                Selected_TargetCell=Target_2x(:,TargetSequence(TargetIndex)); %next target is set as the target index has incremented
            elseif (TargetIndex==2)
                disp('2nd Target Found')

                TargetIndex=3;
                Selected_TargetCell=Target_2x(:,TargetSequence(TargetIndex));

            elseif (TargetIndex==3)
                disp('3rd Target Found')

                TargetIndex=4;
                Selected_TargetCell=Target_2x(:,TargetSequence(TargetIndex));
            else
                disp('4th Target Found')

            end
            fprintf(" Path length %i \n",PathInd);%displays the path length to reach each target
        end
    end
end
if(sum(TargetFound)==4) %if this statement is true it means the robot has reached all its target.
    disp("All Targets Found")

end

% Backtracking the robot to the start cell
while ~(RobotPose(1) == RobotStartCell(1) && RobotPose(2) == RobotStartCell(2))
    Roh = plot(CellsX(RobotPose(1)), CellsY(RobotPose(2)), 'bo'); %the robot is given a differnet colour(blue) when backtracking for visualisation
    pause(0.1);
    delete(Roh);

    % Finding the previous cell until robot reaches its starting cell
    RobotPose = Path(PathInd).Cell;
    PathInd = PathInd - 1;


end

disp('Robot backtracked to its starting point');  %mission accomplished message displayed


disp('------------------------------------------');
pause(1)




