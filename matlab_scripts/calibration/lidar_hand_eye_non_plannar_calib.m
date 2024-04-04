clc; close all;
addpath('./yaml');
addpath('./kabsch');

%% parameter setting
b_odom_obtain = false;
b_abs_rel_odom_check = false;
b_plot_odom = false;
b_noise = false;
str_initializer = 'quaternion';

rng(2);

%% obtain odom
disp('Read odom from txt ...');        
txt_file = 'kitti_traj_front.txt';
lidar_front_data = odom_obtain_from_txt(txt_file, 'Transformation of front lidar', 1, b_plot_odom, b_noise);
txt_file = 'kitti_traj_left.txt';
lidar_left_data = odom_obtain_from_txt(txt_file, 'Transformation of left lidar', 2, b_plot_odom, b_noise);
txt_file = 'kitti_traj_right.txt';
lidar_right_data = odom_obtain_from_txt(txt_file, 'Transformation of right lidar', 3, b_plot_odom, b_noise);

figure;
plot_sensor_odom(lidar_front_data, 'Trajectory in front lidar', 3, 1);
plot_sensor_odom(lidar_left_data, 'Trajectory in left lidar', 3, 2);
plot_sensor_odom(lidar_right_data, 'Trajectory in right lidar', 3, 3);

if b_abs_rel_odom_check
    disp(lidar_front_data.tformAbs_lidar(:,:,end));
    tform = eye(4);
    for frame = 1:length(lidar_front_data.tformRel)
        tform = tform * lidar_front_data.tformRel(:,:,frame);
    end
    disp(tform); disp('checking if equal ?');
end

disp('Finish odom acquirement !');

%% lidar_1 -> lidar_3
disp('lidar_1 to lidar_3 ...');
l = min(lidar_front_data.odom_range, lidar_right_data.odom_range);
% X = Tform_gt_lidar_1_lidar_3;
X = eye(4,4);
n = 5;
lidar_front_data.tformRel_val = zeros(4, 4, 0); 
lidar_right_data.tformRel_val = zeros(4, 4, 0);

theta_error = zeros(n,1);
dis_error = zeros(n,1);
theta_epsilon = 0.01;
dis_epsilon = 0.1;

rotation_error = zeros(n,1);
translation_error = zeros(n,1);
r_epsilon = 0.01;

count_rotation_error = 0;
count_rotation_threshold = 30;

count_initilization = 0;
count_optimization = 0;

%% Online calibration
for i = 1:n
    % step 1: Wrong pair filtration based on screw motion theory
    % input: lidar_front_data, lidar_right_data
    % output: lidar_front_Tform, lidar_right_Tform, theta_error, dis_error
    lidar_front_Tform = lidar_front_data.tformRel(:,:, i); 
    lidar_right_Tform = lidar_right_data.tformRel(:,:, i);
    [t_error, d_error] = motion_pair_filtration(lidar_front_Tform, lidar_right_Tform);
    theta_error(i) = t_error;
    dis_error(i) = d_error;
    
    if (t_error <= theta_epsilon) && (d_error <= dis_epsilon)   
        lidar_front_data.tformRel_val = cat(3, lidar_front_data.tformRel_val, lidar_front_Tform);
        lidar_right_data.tformRel_val = cat(3, lidar_right_data.tformRel_val, lidar_right_Tform);
        % step 2: Miscalibration detection
        [r_error, t_error] = miscalibration_error(lidar_front_Tform, lidar_right_Tform, X,'rad');
        rotation_error(i) = r_error;
        translation_error(i) = t_error;       
        if (r_error > r_epsilon)
            count_rotation_error = count_rotation_error + 1;
        end
    end    
    
    if (count_rotation_error > count_rotation_threshold) || (i == n)
        if (strcmp(str_initializer, 'matrix'))
            % step 3: Initializer(based on matrix)            
            [Rmean, Rvar] = calcR(lidar_front_data.tformRel_val, lidar_right_data.tformRel_val);    
            [Tmean, Tvar] = calcT(lidar_front_data.tformRel_val, lidar_right_data.tformRel_val, Rmean, false);        
            ini_X = [[Rmean,Tmean'];0,0,0,1];
            rotm2eul(ini_X(1:3,1:3))/pi*180        
            disp('Number of poses'); disp(length(lidar_front_data.tformRel_val));
            disp('start index'); disp(i);
            disp('Guess: lidar_1 to lidar_3'); disp(ini_X);
            disp('Count initializaion'); disp(count_initilization);
            
        elseif (strcmp(str_initializer, 'quaternion'))
            % step 3: Initializer(based on quaternion)
            % based on quaternion-optimization: minimize ||Aq||, st.||q||=1
            % non-planner movement  
            R_sol = estimate_Rzyx(lidar_front_data.tformRel_val, lidar_right_data.tformRel_val);
            %R_sol = quat2rotm(q_zyx);
%             R_sol = Tform_gt_lidar_1_lidar_3(1:3,1:3);
            [Tmean, Tvar] = calcT(lidar_front_data.tformRel_val, lidar_right_data.tformRel_val, R_sol, false);        
            ini_X = [[R_sol,Tmean'];0,0,0,1];
            rotm2eul(ini_X(1:3,1:3))/pi*180        
            disp('Number of poses'); disp(length(lidar_front_data.tformRel_val));
            disp('start index'); disp(i);
            disp('Guess: lidar_1 to lidar_3'); disp(ini_X);      
        end
        disp('Count initializaion'); disp(count_initilization);         
        count_initilization = count_initilization + 1;
        count_rotation_error = 0;        
    end 
    
    % step 4: Optimization, model uncertainty
    
    
end

figure;
subplot(2,1,1); title('invariant(rotation)'); hold on; plot(1:1:l, theta_error, 'r-'); hold off;
subplot(2,1,2); title('invariant(translation)'); hold on; plot(1:1:l, dis_error, 'b-'); hold off;

figure;
subplot(2,1,1); title('rotation error(angle)'); hold on; plot(1:1:l, rotation_error, 'r-'); hold off;
subplot(2,1,2); title('translation error'); hold on; plot(1:1:l, translation_error, 'b-'); hold off;