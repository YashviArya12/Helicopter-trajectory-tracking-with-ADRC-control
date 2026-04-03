
% Data from Simulink
t = out.tout;

pitch_actual = out.pitch.Data;
yaw_actual   = out.yaw.Data;

% Desired trajectory
pitch_desired = deg2rad(-20 * square((2*pi/20) * t));
yaw_desired   = deg2rad(-20 * square((2*pi/20) * t));

% Create XY path
step_size = 0.08;

x_des = zeros(length(t),1);
y_des = zeros(length(t),1);

x_act = zeros(length(t),1);
y_act = zeros(length(t),1);

for k = 2:length(t)

    x_des(k) = x_des(k-1) + step_size*cos(yaw_desired(k));
    y_des(k) = y_des(k-1) + step_size*sin(pitch_desired(k));

    x_act(k) = x_act(k-1) + step_size*cos(yaw_actual(k));
    y_act(k) = y_act(k-1) + step_size*sin(pitch_actual(k));

end

% Figure
figure('Color','w','Position',[50 50 1700 850])

for k = 1:5:length(t)

    clf


    % LARGE LEFT PLOT : TRAJECTORY + HELICOPTER

    ax1 = axes('Position',[0.05 0.10 0.58 0.82]);
    hold(ax1,'on')
    grid(ax1,'on')
    axis(ax1,'equal')

    % Desired trajectory
    h1 = plot(ax1,x_des(1:k),y_des(1:k), ...
        'b--','LineWidth',3);

    % Actual trajectory
    h2 = plot(ax1,x_act(1:k),y_act(1:k), ...
        'r','LineWidth',3);

    % Current desired position
    h3 = plot(ax1,x_des(k),y_des(k), ...
        'bo','MarkerFaceColor','b','MarkerSize',10);

    % Current actual position
    h4 = plot(ax1,x_act(k),y_act(k), ...
        'ro','MarkerFaceColor','r','MarkerSize',12);

    %% Bigger and clearer helicopter

    yaw = yaw_actual(k);

    % Main body
    body = [ -0.45 -0.18;
              0.45 -0.18;
              0.45  0.18;
             -0.45  0.18 ];

    % Nose
    nose = [0.45 -0.18;
            0.72  0;
            0.45  0.18];

    % Tail boom
    tail = [-0.45 0;
            -1.10 0];

    % Main rotor
    rotor1 = [-0.65 0;
               0.65 0];

    rotor2 = [0 -0.65;
              0  0.65];

    % Tail rotor
    tail_rotor = [1.10 -0.12;
                  1.10  0.12];

    % Rotation matrix
    R = [cos(yaw) -sin(yaw);
         sin(yaw)  cos(yaw)];

    body_r       = (R*body')';
    nose_r       = (R*nose')';
    tail_r       = (R*tail')';
    rotor1_r     = (R*rotor1')';
    rotor2_r     = (R*rotor2')';
    tail_rotor_r = (R*tail_rotor')';

    % Shift to helicopter position
    helicopter_parts = {body_r,nose_r,tail_r,rotor1_r,rotor2_r,tail_rotor_r};

    for i = 1:length(helicopter_parts)
        helicopter_parts{i}(:,1) = helicopter_parts{i}(:,1) + x_act(k);
        helicopter_parts{i}(:,2) = helicopter_parts{i}(:,2) + y_act(k);
    end

    body_r       = helicopter_parts{1};
    nose_r       = helicopter_parts{2};
    tail_r       = helicopter_parts{3};
    rotor1_r     = helicopter_parts{4};
    rotor2_r     = helicopter_parts{5};
    tail_rotor_r = helicopter_parts{6};

    % Draw helicopter
    h5 = fill(body_r(:,1),body_r(:,2),[0.2 0.6 0.9], ...
        'EdgeColor','b','LineWidth',2);

    fill(nose_r(:,1),nose_r(:,2),[0.1 0.4 0.8], ...
        'EdgeColor','b','LineWidth',2);

    h6 = plot(tail_r(:,1),tail_r(:,2), ...
        'k','LineWidth',2);

    h7 = plot(rotor1_r(:,1),rotor1_r(:,2), ...
        'g','LineWidth',2);

    plot(rotor2_r(:,1),rotor2_r(:,2), ...
        'g','LineWidth',2);

    plot(tail_rotor_r(:,1),tail_rotor_r(:,2), ...
        'g','LineWidth',2);

    xlabel(ax1,'X Position')
    ylabel(ax1,'Y Position')

    title(ax1,['Helicopter Desired vs Actual Trajectory    Time = ', ...
        num2str(t(k),'%.1f'),' s'], ...
        'FontSize',16,'FontWeight','bold')

    xlim(ax1,[min([x_des; x_act])-2 , max([x_des; x_act])+2])
    ylim(ax1,[min([y_des; y_act])-2 , max([y_des; y_act])+2])

    legend(ax1,[h1 h2 h3 h4 h5 h6 h7], ...
        {'Desired Trajectory', ...
         'Actual Trajectory', ...
         'Desired Position', ...
         'Actual Position', ...
         'Helicopter Body', ...
         'Tail Boom', ...
         'Main Rotor'}, ...
         'Location','northeastoutside', ...
         'FontSize',10)

    % PITCH GRAPH

    ax2 = axes('Position',[0.70 0.57 0.27 0.33]);
    hold(ax2,'on')
    grid(ax2,'on')

    p1 = plot(ax2,t(1:k),rad2deg(pitch_desired(1:k)), ...
        'b--','LineWidth',3);

    p2 = plot(ax2,t(1:k),rad2deg(pitch_actual(1:k)), ...
        'b','LineWidth',3);

    xlabel(ax2,'Time [s]')
    ylabel(ax2,'Pitch Angle [deg]')

    title(ax2,'Pitch Tracking','FontSize',14,'FontWeight','bold')

    xlim(ax2,[0 max(t)])
    ylim(ax2,[-30 30])

    legend(ax2,[p1 p2], ...
        {'Desired Pitch','Actual Pitch'}, ...
        'Location','northeast', ...
        'FontSize',10)

   
    % Yaw Graph

    ax3 = axes('Position',[0.70 0.12 0.27 0.33]);
    hold(ax3,'on')
    grid(ax3,'on')

    y1 = plot(ax3,t(1:k),rad2deg(yaw_desired(1:k)), ...
        'r--','LineWidth',3);

    y2 = plot(ax3,t(1:k),rad2deg(yaw_actual(1:k)), ...
        'r','LineWidth',3);

    xlabel(ax3,'Time [s]')
    ylabel(ax3,'Yaw Angle [deg]')

    title(ax3,'Yaw Tracking','FontSize',14,'FontWeight','bold')

    xlim(ax3,[0 max(t)])
    ylim(ax3,[-30 30])

    legend(ax3,[y1 y2], ...
        {'Desired Yaw','Actual Yaw'}, ...
        'Location','northeast', ...
        'FontSize',10)

    drawnow

    if k + 5 <= length(t)
        pause(t(k+5) - t(k))
    end

end
