x = [0, 0, 0, 0, 0]; % Initial state [x, y, theta, v, w]
goal = [1, 1]; % Goal position [x, y]
obstacles = [0.5, 0.5, 0.2; -0.5, -0.5, 0.2]; % Obstacle positions and radii
r = 0.1; % Robot radius
vmax = 0.5; % Max linear velocity
wmax = pi/2; % Max angular velocity
dt = 0.1; % Time step



for t = 1:100
  % Get velocities using DWA algorithm
  
xnext = @(x, v, w) [x(1)+v*dt*cos(x(3)), x(2)+v*dt*sin(x(3)), x(3)+w*dt, v, w];
  
  % Define cost function
  cost = @(x, goal) norm([x(1)-goal(1), x(2)-goal(2)]);
  
  % Define dynamic window
  vs = linspace(0, vmax, 10);
  ws = linspace(-wmax, wmax, 21);
  
  % Evaluate trajectories within dynamic window
  best_cost = inf;
  for v = vs
    for w = ws
      % Simulate trajectory
      xt = x;
      cost_t = 0;
      for t = 1:10
        xt = xnext(xt, v, w);
        cost_t = cost_t + cost(xt, goal);
        
        % Check for collision with obstacles
        for i = 1:size(obstacles, 1)
          d = norm(xt(1:2)-obstacles(i, 1:2));
          if d < r+obstacles(i, 3)
            cost_t = inf;
            break
          end
        end
      end
      
      % Update best trajectory
      if cost_t < best_cost
        best_cost = cost_t;
        best_v = v;
        best_w = w;
      end
    end
  end
  
  % Return best velocities
  v = best_v;
  w = best_w;

  % Update state using motion model
  x = [x(1)+v*dt*cos(x(3)), x(2)+v*dt*sin(x(3)), x(3)+w*dt, v, w];
  
  % Plot robot and obstacles
  plot(x);
  plot(obstacles);
  
  % Check if goal is reached
  if norm(x(1:2)-goal) < 0.1
    break
  end
  
  % Pause for visualization
  pause(0.1);
end


function [v, w] = dwa_navigation(x, goal, obstacles)
  % DWA Navigation algorithm
  % Inputs:
  %   x: current state [x, y, theta, v, w]
  %   goal: goal position [x, y]
  %   obstacles: array of obstacle positions [x, y, radius]
  % Outputs:
  %   v: selected linear velocity
  %   w: selected angular velocity
  
  % Define robot parameters
  r = 0.1; % Robot radius
  vmax = 0.5; % Max linear velocity
  wmax = pi/2; % Max angular velocity
  dt = 0.1; % Time step
  
  % Define motion model
  xnext = @(x, v, w) [x(1)+v*dt*cos(x(3)), x(2)+v*dt*sin(x(3)), x(3)+w*dt, v, w];
  
  % Define cost function
  cost = @(x, goal) norm([x(1)-goal(1), x(2)-goal(2)]);
  
  % Define dynamic window
  vs = linspace(0, vmax, 10);
  ws = linspace(-wmax, wmax, 21);
  
  % Evaluate trajectories within dynamic window
  best_cost = inf;
  for v = vs
    for w = ws
      % Simulate trajectory
      xt = x;
      cost_t = 0;
      for t = 1:10
        xt = xnext(xt, v, w);
        cost_t = cost_t + cost(xt, goal);
        
        % Check for collision with obstacles
        for i = 1:size(obstacles, 1)
          d = norm(xt(1:2)-obstacles(i, 1:2));
          if d < r+obstacles(i, 3)
            cost_t = inf;
            break
          end
        end
      end
      
      % Update best trajectory
      if cost_t < best_cost
        best_cost = cost_t;
        best_v = v;
        best_w = w;
      end
    end
  end
  
  % Return best velocities
  v = best_v;
  w = best_w;
end