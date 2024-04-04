clc; close all;
addpath('./yaml');
addpath('./kabsch');
addpath('./camodocal');

%% parameter setting
str_gt_mat = 'data/yellow_20180104';
str_odom_obtain = 'data/simulation_odom_2';
str_odom_bag = 'read_from_bag'; % 'calculate_from_pcd'
str_initializer = 'quaternion';
b_odom_obtain = false;
b_abs_rel_odom_check = false;
b_plot_odom = false;
b_plot_sensor_odom = true;
b_noise = false;


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
disp('lidar_2 to lidar_3 ...');
l = min(lidar_front_data.odom_range, lidar_right_data.odom_range);
% X = Tform_gt_lidar_1_lidar_3;
X = eye(4,4);

lidar_front_data.tformRel_val = zeros(4, 4, 0); 
lidar_right_data.tformRel_val = zeros(4, 4, 0);

theta_error = zeros(l,1);
dis_error = zeros(l,1);
theta_epsilon = 0.01;
dis_epsilon = 0.1;

rotation_error = zeros(l,1);
translation_error = zeros(l,1);
r_epsilon = 0.01;

count_rotation_error = 0;
count_rotation_threshold = 30;

count_initilization = 0;
count_optimization = 0;

%% Online calibration
for i = 1:l
    % step 1: Wrong pair filtration based on screw motion theory
    % input: lidar_1_data, lidar_3_data
    % output: lidar_1_Tform, lidar_3_Tform, theta_error, dis_error
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
    
    if (count_rotation_error > count_rotation_threshold) || (i == l)
%     if (i == l)
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
            % planner movement
            [b_solve, b_rot_con, R_yx]  = estimate_Ryx(lidar_front_data.tformRel_val, lidar_right_data.tformRel_val);
%             R_yx = eul2rotm([pi 0 0], 'ZYX');

            [R_z, t_x, t_y] = estimate_Rz_tx_ty(lidar_front_data.tformRel_val, lidar_right_data.tformRel_val, R_yx);
            ini_R = R_z * R_yx;
            ini_t = [t_x; t_y; 0]; % z-translation is unknown
            ini_Tform = [[ini_R,ini_t];0,0,0,1];
            disp('# INFO: Motion-based before refinement: '); disp(ini_Tform);

            % step 4: nonlinear optimization
            refine_Tform = motion_based_refinement(lidar_front_data.tformRel_val, lidar_right_data.tformRel_val, ini_Tform);
            disp('# INFO: Motion-based after refinement: '); disp(refine_Tform);
            ini_X = refine_Tform;
            
            rotm2eul(ini_X(1:3,1:3))/pi*180        
            disp('Number of poses'); disp(length(lidar_front_data.tformRel_val));
            disp('start index'); disp(i);
            disp('Guess: lidar_1 to lidar_3'); disp(ini_X);     
        end
        disp('Count initializaion'); disp(count_initilization);         
        count_initilization = count_initilization + 1;
        count_rotation_error = 0;        
    end  
end

figure;
subplot(2,1,1); title('invariant(rotation)'); hold on; plot(1:1:l, theta_error, 'r-'); hold off;
subplot(2,1,2); title('invariant(translation)'); hold on; plot(1:1:l, dis_error, 'b-'); hold off;

figure;
subplot(2,1,1); title('rotation error(angle)'); hold on; plot(1:1:l, rotation_error, 'r-'); hold off;
subplot(2,1,2); title('translation error'); hold on; plot(1:1:l, translation_error, 'b-'); hold off;
