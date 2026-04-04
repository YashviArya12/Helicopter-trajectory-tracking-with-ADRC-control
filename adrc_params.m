%% Pitch ADRC parameters

% Plant parameter (rough estimate)
b_tetha = 11.575;          % estimated input gain (can be wrong)

% ESO bandwidth
wo_pitch = 30;           % observer bandwidth (rad/s)

% ESO gains (standard ADRC rule)
beta1_p = 3*wo_pitch;
beta2_p = 3*wo_pitch^2;
beta3_p = wo_pitch^3;

% PD controller gains
Kp_p = 27;
Kd_p = 9;

%% Yaw ADRC parameters

% Plant parameter (rough estimate)
b_psi = 23.148;

% ESO bandwidth
wo_yaw = 45;

% ESO gains
beta1_y = 3*wo_yaw;
beta2_y = 3*wo_yaw^2;
beta3_y = wo_yaw^3;

% PD controller gains
Kp_y = 64;
Kd_y = 16;



