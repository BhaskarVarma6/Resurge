% Multi-Robot Trajectory Optimization with CBF Only (Increased Speed and Time)
clear; clc; close all;

%% Parameters
n_agents = 4;             % Number of agents (robots)
n_obstacles = 2;          % Number of dynamic obstacles
timestep = 0.1;           % Time step (seconds)
total_time = 50;          % Total simulation time (seconds) increased to 40 seconds
iterations = total_time / timestep; % Number of simulation steps

% Control Barrier Function Parameters
cbf_safe_distance_agents = 0.6;   % Safe distance to avoid collisions between agents
cbf_safe_distance_obstacles = 0.6; % Safe distance to avoid collisions with obstacles

% Robot and obstacle initialization
robot_positions = [1, 1; 1, 8; 8, 1; 8, 8];  % Initial positions of robots
goal_positions = [8, 8; 8, 1; 1, 8; 1, 1];   % Goal positions of robots

% Obstacles: [initial position, velocity function]
obstacle_positions = [3, 5; 7, 5];    % Initial positions of dynamic obstacles

% Obstacle motion: [type, parameters]
% Obstacle 1 - Sinusoidal motion along x-axis
% Obstacle 2 - Circular motion around center (5,5)
obstacle_motion = {@(t) [3 + 2*sin(0.5*t), 5]; @(t) [5 + 2*cos(0.2*t), 5 + 2*sin(0.2*t)]};

% Simulation tracking parameters
robot_traj = zeros(n_agents, 2, iterations); % Store robot positions
obstacle_traj = zeros(n_obstacles, 2, iterations); % Store obstacle positions
cbf_violations = zeros(n_agents, iterations); % Track CBF violations (for plotting)

% Define markers and colors for each agent
agent_markers = {'o', 's', 'd', 'p'};  % Markers: circle, square, diamond, pentagon
agent_colors = {'b', 'g', 'r', 'm'};   % Colors: blue, green, red, magenta

%% Simulation Loop
for t = 1:iterations
    % Update obstacle positions based on motion
    obstacle_positions = update_obstacles(t * timestep, obstacle_motion);
    obstacle_traj(:,:,t) = obstacle_positions; % Store obstacle positions

    % CBF - Handle collisions with dynamic obstacles and inter-agent collision avoidance
    [robot_positions, cbf_violation_flags] = apply_cbf(robot_positions, obstacle_positions, cbf_safe_distance_agents, cbf_safe_distance_obstacles);
    cbf_violations(:,t) = cbf_violation_flags; % Track CBF activations (violations)

    % Move towards goal with increased speed
    for i = 1:n_agents
        direction_to_goal = goal_positions(i,:) - robot_positions(i,:);

        % Normalize direction to have consistent speed
        direction_norm = norm(direction_to_goal);
        if direction_norm > 0
            direction_to_goal = direction_to_goal / direction_norm;
        end

        % Increase speed: Update the robot's position based on its goal-seeking direction
        robot_positions(i,:) = robot_positions(i,:) + 0.2 * direction_to_goal * timestep; % Increased speed by 2x
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
    title(sprintf('CBF Collision Avoidance - Time: %.1f seconds', t * timestep));

    % Plot robot actual positions with different colors/markers for each agent
    for i = 1:n_agents
        scatter(robot_positions(i,1), robot_positions(i,2), 80, agent_colors{i}, agent_markers{i}, 'filled');
    end

    % Plot goal positions
    scatter(goal_positions(:,1), goal_positions(:,2), 80, 'k', 'x', 'LineWidth', 2);

    % Plot dynamic obstacles
    scatter(obstacle_positions(:,1), obstacle_positions(:,2), 100, 'r', 'filled');

    % Plot trajectories of actual positions
    for i = 1:n_agents
        plot(squeeze(robot_traj(i,1,1:t)), squeeze(robot_traj(i,2,1:t)), agent_colors{i}, 'LineWidth', 1.5);
    end

    % Plot obstacles trajectories
    for j = 1:n_obstacles
        plot(squeeze(obstacle_traj(j,1,1:t)), squeeze(obstacle_traj(j,2,1:t)), 'r--');
    end

    legend('show');
    legend({'Agent 1', 'Agent 2', 'Agent 3', 'Agent 4', 'Goal', 'Obstacles'}, 'Location', 'northeastoutside');

    % Subplot 2: Plot CBF activations over time
    subplot(3,1,2);
    hold on;
    xlabel('Time (iterations)'); ylabel('CBF Activation');
    title('CBF Activations (Collision Avoidance) Over Time');
    
    % Plot the CBF violations (activation of CBF) for each agent over time
    for i = 1:n_agents
        plot(1:t, cbf_violations(i,1:t), 'LineWidth', 1.5, 'Color', agent_colors{i});
    end
    legend('show');
    xlim([0, iterations]);
    ylim([0, 1]); % CBF activation is binary (either 0 or 1)

    % Pause to simulate real time
    pause(timestep);
end

%% Helper Function Definitions

% Function to apply Control Barrier Function for collision avoidance
function [updated_positions, cbf_violations] = apply_cbf(robot_positions, obstacle_positions, safe_distance_agents, safe_distance_obstacles)
    updated_positions = robot_positions;
    n_agents = size(robot_positions,1);
    cbf_violations = zeros(n_agents, 1);  % To track which robots violate CBF

    % Avoid agent collisions using CBF
    for i = 1:n_agents
        for j = 1:n_agents
            if i ~= j
                dist = norm(robot_positions(i,:) - robot_positions(j,:));
                if dist < safe_distance_agents
                    % Move robot i away from robot j to maintain safe distance
                    direction = (robot_positions(i,:) - robot_positions(j,:)) / dist;
                    updated_positions(i,:) = updated_positions(i,:) + direction * (safe_distance_agents - dist);
                    cbf_violations(i) = 1;  % Mark CBF activation
                end
            end
        end
    end

    % Avoid obstacle collisions using CBF
    for i = 1:n_agents
        for j = 1:size(obstacle_positions,1)
            dist = norm(robot_positions(i,:) - obstacle_positions(j,:));
            if dist < safe_distance_obstacles
                % Move robot i away from obstacle j to maintain safe distance
                direction = (robot_positions(i,:) - obstacle_positions(j,:)) / dist;
                updated_positions(i,:) = updated_positions(i,:) + direction * (safe_distance_obstacles - dist);
                cbf_violations(i) = 1;  % Mark CBF activation
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
