% Multi-Robot Trajectory Optimization with Local and Global Attention Dynamics and Critical Attention Threshold
clear; clc; close all;

%% Parameters
n_agents = 4;             % Number of agents (robots)
n_obstacles = 2;          % Number of dynamic obstacles
timestep = 0.2;           % Time step (seconds)
total_time = 40;          % Total simulation time (seconds)
iterations = total_time / timestep; % Number of simulation steps

% Local and global attention radii
local_attention_radius = 2.0;   % Radius for local attention (avoiding obstacles and nearby agents)
global_attention_radius = 4.0;  % Radius for global attention (cooperative behavior)

% Opinion update weights
alpha_local = 0.7;      % Weight for local attention behavior (avoiding obstacles/agents)
alpha_global = 0.5;     % Weight for global attention behavior (global cooperation)

% Critical attention threshold (opinions only change when attention exceeds this value)
critical_attention_threshold = 0.5;

% Safe distances
safe_distance_agents = 0.5;     % Minimum distance between agents to avoid collision
safe_distance_obstacles = 0.5;  % Safe distance from obstacles

% Opinion boundaries (to constrain opinions between -1 and 1)
opinion_min = -1;
opinion_max = 1;

% Robot and obstacle initialization
robot_positions = [1, 1; 1, 8; 8, 1; 8, 8];  % Initial positions of robots
goal_positions = [8, 8; 8, 1; 1, 8; 1, 1];   % Goal positions of robots

% Initialize opinions (all agents start with opinions at zero)
opinion_positions = zeros(n_agents, 1);  % All opinions start at 0 (equilibrium)

% Obstacles: [initial position, velocity function]
obstacle_positions = [3, 5; 7, 5];    % Initial positions of dynamic obstacles

% Obstacle motion: [type, parameters]
% Obstacle 1 - Sinusoidal motion along x-axis
% Obstacle 2 - Circular motion around center (5,5)
obstacle_motion = {@(t) [3 + 2*sin(0.5*t), 5]; @(t) [5 + 2*cos(0.2*t), 5 + 2*sin(0.2*t)]};

%% Simulation parameters
robot_traj = zeros(n_agents, 2, iterations); % Store robot positions
obstacle_traj = zeros(n_obstacles, 2, iterations); % Store obstacle positions
opinion_traj = zeros(n_agents, iterations); % Store opinions (X-axis opinions only for steering)
opinion_evolution = zeros(n_agents, iterations); % Store opinion evolution (X-direction)

% Local and global attention tracking
local_attention_values = zeros(n_agents, iterations);
global_attention_values = zeros(n_agents, iterations);

% Define markers and colors for each agent
agent_markers = {'o', 's', 'd', 'p'};  % Markers: circle, square, diamond, pentagon
agent_colors = {'b', 'g', 'r', 'm'};   % Colors: blue, green, red, magenta

%% Simulation Loop
for t = 1:iterations
    % Update obstacle positions based on motion
    obstacle_positions = update_obstacles(t*timestep, obstacle_motion);
    obstacle_traj(:,:,t) = obstacle_positions; % Store obstacle positions

    % Local and global attention dynamics
    local_neighbors = get_neighbors(robot_positions, local_attention_radius);   % Local attention (obstacle/agent avoidance)
    global_neighbors = get_neighbors(robot_positions, global_attention_radius); % Global attention (cooperation)

    % Opinion dynamics: Update agent opinions with local and global attention
    [opinion_positions, local_attention, global_attention] = update_opinions_with_attention(opinion_positions, robot_positions, goal_positions, local_neighbors, global_neighbors, alpha_local, alpha_global, opinion_min, opinion_max, critical_attention_threshold);
    opinion_traj(:,t) = opinion_positions; % Store opinions for plotting

    % Track evolution of opinions (X-opinions influence steering)
    opinion_evolution(:,t) = opinion_positions;
    
    % Track local and global attention values over time
    local_attention_values(:,t) = local_attention;
    global_attention_values(:,t) = global_attention;

    % Handle collisions with dynamic obstacles and inter-agent collision avoidance
    robot_positions = handle_collisions(robot_positions, obstacle_positions, safe_distance_obstacles);
    robot_positions = avoid_agent_collisions(robot_positions, safe_distance_agents);  % Inter-agent collision avoidance

    % Move towards goal and adjust steering based on opinion
    for i = 1:n_agents
        direction_to_goal = goal_positions(i,:) - robot_positions(i,:);
        
        % Adjust movement based on the opinion: steers more based on opinion value
        direction_to_goal(1) = direction_to_goal(1) + opinion_positions(i) * 2;  % Steer strongly left/right based on opinion
        
        % Update the robot's position based on its goal-seeking and opinion
        robot_positions(i,:) = robot_positions(i,:) + 0.05 * direction_to_goal * timestep; % Basic step towards goal with steering
    end
    
    % Store robot positions for plotting
    robot_traj(:,:,t) = robot_positions;

    %% Visualization and Animation
    clf; % Clear previous plot
    
    % Subplot 1: Main simulation plot
    subplot(3,1,1);
    hold on;
    axis([0 10 0 10]);
    xlabel('X'); ylabel('Y');
    title(sprintf('Simulation Time: %.1f seconds', t*timestep));

    % Plot robot actual positions with different colors/markers for each agent
    for i = 1:n_agents
        scatter(robot_positions(i,1), robot_positions(i,2), 80, agent_colors{i}, agent_markers{i}, 'filled', 'DisplayName', sprintf('Agent %d', i));
    end

    % Plot robot opinions (we'll show opinions as X-axis offsets)
    for i = 1:n_agents
        scatter(robot_positions(i,1) + opinion_positions(i), robot_positions(i,2), 80, agent_colors{i}, agent_markers{i});
    end
    
    % Plot goal positions
    scatter(goal_positions(:,1), goal_positions(:,2), 80, 'k', 'x', 'LineWidth', 2, 'DisplayName', 'Goals');

    % Plot dynamic obstacles
    scatter(obstacle_positions(:,1), obstacle_positions(:,2), 100, 'r', 'filled', 'DisplayName', 'Obstacles');
    
    % Plot trajectories of actual positions
    for i = 1:n_agents
        plot(squeeze(robot_traj(i,1,1:t)), squeeze(robot_traj(i,2,1:t)), agent_colors{i}, 'LineWidth', 1.5);
    end
    
    % Plot obstacles trajectories
    for j = 1:n_obstacles
        plot(squeeze(obstacle_traj(j,1,1:t)), squeeze(obstacle_traj(j,2,1:t)), 'r--');
    end

    % Add legend to clarify which agent is which
    legend('Location', 'northeastoutside');

    % Subplot 2: Plot opinion evolution over time (X-opinions)
    subplot(3,1,2);
    hold on;
    xlabel('Time (iterations)'); ylabel('Opinion (X-direction)');
    title('Opinion Evolution Over Time');
    
    % Plot the evolution of each agent's X-opinion
    for i = 1:n_agents
        plot(1:t, opinion_evolution(i,1:t), 'LineWidth', 1.5, 'Color', agent_colors{i}, 'DisplayName', sprintf('Agent %d', i));
    end
    legend('Location', 'eastoutside');
    xlim([0, iterations]);
    ylim([-1, 1]); % Opinions are constrained between -1 and 1

    % Subplot 3: Local and Global Attention over time
    subplot(3,1,3);
    hold on;
    xlabel('Time (iterations)');
    title('Local and Global Attention Over Time');
    
    % Plot Local and Global attention evolution for each agent
    for i = 1:n_agents
        plot(1:t, local_attention_values(i,1:t), '--', 'LineWidth', 1.5, 'Color', agent_colors{i}, 'DisplayName', sprintf('Local - Agent %d', i));
        plot(1:t, global_attention_values(i,1:t), '-', 'LineWidth', 1.5, 'Color', agent_colors{i}, 'DisplayName', sprintf('Global - Agent %d', i));
    end
    legend('show');
    xlim([0, iterations]);
    ylim([0, 1]);  % Attention values confined between 0 and 1

    % Pause to simulate real time
    pause(timestep);
end

%% Function Definitions

% Function to update robot opinions based on local and global attention dynamics
function [updated_opinion, local_attention, global_attention] = update_opinions_with_attention(opinion_positions, robot_positions, goal_positions, local_neighbors, global_neighbors, alpha_local, alpha_global, opinion_min, opinion_max, critical_attention_threshold)
    updated_opinion = opinion_positions;
    local_attention = zeros(size(opinion_positions));
    global_attention = zeros(size(opinion_positions));
    
    for i = 1:size(opinion_positions,1)
        % Local attention: Reciprocal of distance, normalized and constrained to [0,1]
        local_attention(i) = compute_attention(robot_positions(i,:), local_neighbors{i});
        if local_attention(i) > critical_attention_threshold
            % Only update opinion if local attention exceeds threshold
            updated_opinion(i) = opinion_positions(i) + alpha_local * local_attention(i);
        end
        
        % Global attention: Reciprocal of distance for global behavior
        global_attention(i) = compute_attention(robot_positions(i,:), global_neighbors{i});
        if global_attention(i) > critical_attention_threshold
            % Only update opinion if global attention exceeds threshold
            updated_opinion(i) = updated_opinion(i) + alpha_global * global_attention(i);
        end
        
        % Add a bias towards the goal (goal-seeking behavior)
        direction_to_goal = goal_positions(i,:) - robot_positions(i,:);
        updated_opinion(i) = updated_opinion(i) + alpha_global * sign(direction_to_goal(1));
        
        % Clip opinion values to be within -1 and 1
        updated_opinion(i) = max(opinion_min, min(updated_opinion(i), opinion_max));
    end
end

% Function to compute attention based on reciprocal of distance, normalized to [0,1]
function attention = compute_attention(agent_position, neighbors)
    if isempty(neighbors)
        attention = 0;
    else
        distances = sqrt(sum((neighbors - agent_position).^2, 2));
        % Compute attention as 1/distance, normalized to [0,1]
        attention = min(1, mean(1 ./ (distances + 1e-6)));  % Add a small value to avoid division by zero
    end
end

% Function to get neighbors based on communication radius
function neighbors = get_neighbors(robot_positions, attention_radius)
    neighbors = cell(size(robot_positions,1), 1);
    for i = 1:size(robot_positions,1)
        for j = 1:size(robot_positions,1)
            if i ~= j && norm(robot_positions(i,:) - robot_positions(j,:)) <= attention_radius
                neighbors{i} = [neighbors{i}; robot_positions(j,:)];
            end
        end
    end
end

% Function to avoid collisions with dynamic obstacles
function robot_positions = handle_collisions(robot_positions, obstacle_positions, safe_distance)
    for i = 1:size(robot_positions,1)
        for j = 1:size(obstacle_positions,1)
            dist = norm(robot_positions(i,:) - obstacle_positions(j,:));
            if dist < safe_distance
                % Move the robot away from the obstacle
                direction = (robot_positions(i,:) - obstacle_positions(j,:)) / dist;
                robot_positions(i,:) = robot_positions(i,:) + direction * (safe_distance - dist);
            end
        end
    end
end

% Function to avoid collisions between agents
function robot_positions = avoid_agent_collisions(robot_positions, safe_distance)
    for i = 1:size(robot_positions,1)
        for j = 1:size(robot_positions,1)
            if i ~= j
                dist = norm(robot_positions(i,:) - robot_positions(j,:));
                if dist < safe_distance
                    % Move the robot away from the other robot
                    direction = (robot_positions(i,:) - robot_positions(j,:)) / dist;
                    robot_positions(i,:) = robot_positions(i,:) + direction * (safe_distance - dist);
                end
            end
        end
    end
end

% Function to update obstacle positions based on motion type
function obstacle_positions = update_obstacles(t, obstacle_motion)
    obstacle_positions = zeros(length(obstacle_motion), 2);
    for i = 1:length(obstacle_motion)
        obstacle_positions(i, :) = obstacle_motion{i}(t);
    end
end
