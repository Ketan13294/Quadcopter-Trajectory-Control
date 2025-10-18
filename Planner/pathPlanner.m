%% create an occupancy map from an example map
function waypts = pathPlanner(start,goal,plt)
    map = load("myMap3D.mat").map3D;
    map2 = load("myMap3D_inflate.mat").map3D;
    map2D = load("exampleMaps.mat").simpleMap;
    %map resolution 5 cells/meter
    map2D = occupancyMap(map2D,4);
    
    %% create a state space
    ss = stateSpaceSE3([0,6.5;0,6.75;2,2;Inf,Inf;Inf,Inf;Inf,Inf;Inf,Inf]);
    
    %% create state validator
    sv = validatorOccupancyMap3D(ss,Map=map2,ValidationDistance=0.01);
    
    %% Create prm object
    planner = plannerRRTStar(ss,sv);
    planner.ContinueAfterGoalReached = true;
    planner.MaxConnectionDistance = 1.5;
    planner.MaxIterations = 2000;
    planner.GoalReachedFcn = @test_function;
    planner.GoalBias = 0.4;
    
    %% Plan path with default setting. Using rng seed for repeatibility
    rng(100,"twister");
    [pthObj, solnInfo] = plan(planner,start,goal);
    if (~solnInfo.IsPathFound)
        disp("No Path Found by the RRT, terminating!")
        return
    end
    
    %% Visualize results
    waypts = pthObj.States;
    nwaypts = pthObj.NumStates;

    % Calculate the distance between waypoints
    distance = zeros(1,nwaypts);
    distance(1) = 0.05;
    for i = 1:nwaypts-1
        distance(i+1) = norm(waypts(i+1,1:3) - waypts(i,1:3));
    end

    % Assume a UAV speed of 3 m/s and calculate time taken to reach each waypoint
    UAVspeed = 1.0;
    timepoints = cumsum(distance/UAVspeed);
    nSamples = 100;

    waypts = minsnappolytraj(waypts',timepoints,nSamples,MinSegmentTime=0.01,MaxSegmentTime=10,TimeAllocation=true,TimeWeight=10)';
    %%
    if plt ==1
        figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]); % full screen
        show(map)
        hold on
        % Tree points
        plot3(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), solnInfo.TreeData(:,3),'-','Color',[1 1 1],'LineWidth',0.5); % tree expansion
        % Minimum Jerk Trajectory
        plot3(waypts(1:end,1),waypts(1:end,2),waypts(1:end,3),'k-','LineWidth',2)
        plot3(goal(1),goal(2),goal(3),"g*")
        plot3(start(1),start(2),start(3),"r*")
        hold off
        view(-30.0,45.0);
    end
    waypts = waypts(:,1:3);
end

function isReached = test_function(planner,currentState,goalState)
    if norm(currentState(1:2)-goalState(1:2)) < 0.01
        isReached = 1;
    else
        isReached = 0;
    end
end


