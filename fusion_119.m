%%%%%% ICP ALGORITHM %%%%%%%%%
% First run cornerstform.m to initialize 2D-3D coordinate system transforms
format long
img1 = imread('Images\11.9\SeriesP\P310-280.jpg');
pc_in1 = pcread("PointClouds\11.9\SeriesP\satonly\fullscan\310-280.ply");
dist_1 = 3.1;
img2 = imread('Images\11.9\SeriesP\P300-300.jpg');
pc_in2 = pcread("PointClouds\11.9\SeriesP\satonly\fullscan\300-300.ply");
dist_2 = 3.0;

interptform = 0; % Flag for x distance tform interpolation

% Correct control tform for RMSE comparison
% R_intrinsic = eye(4);
R_intrinsic =[cosd(20) -sind(20) 0 0;sind(20) cosd(20) 0 0;0 0 1 0;0 0 0 1];
T_intrinsic = [1 0 0 0;0 1 0 0.1;0 0 1 0;0 0 0 1];

% For Boxsat2
% R_intrinsic = [0.851871967316 -0.522744476795 0.032440010458 0;
%                0.522924959660 0.852372229099 0.003322570352 0;
%                -0.029387820512 0.014133287594 0.999468147755 0;
%                0 0 0 1];
% T_intrinsic = [1 0 0 0.048387654126;
%                0 1 0 -0.021505514160;
%                0 0 1 -0.000000007007;
%                0 0 0 1];

inlier_dist = 0.25; % maximum distance for inliers in meters
inlier_ratio = 0.8; % range: [0 1]
max_iter = 50; % default 30
Tolerance = [0.001 0.001]; % default [0.01 0.5] [Tdiff Rdiff]
Metric = "planeToPlaneWithColor";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if interptform == 1
    % Interpolate and create a new tform. x-translation only!
    % Distance img3
    s_x1 = tform_corners2.A(1,1)+(tform_corners3.A(1,1)-tform_corners2.A(1,1))*(dist_3-xref1)/(xref2-xref1);
    s_y2 = tform_corners2.A(2,2)+(tform_corners3.A(2,2)-tform_corners2.A(2,2))*(dist_3-xref1)/(xref2-xref1);
    A_tform4 = [s_x1 0 tform_corners2.A(1,3); 0 s_y2 tform_corners2.A(2,3); 0 0 1];
    tform_corners4 = affinetform2d(A_tform4);
    
    % Distance img4
    s_x3 = tform_corners2.A(1,1)+(tform_corners3.A(1,1)-tform_corners2.A(1,1))*(dist_4-xref1)/(xref2-xref1);
    s_y3 = tform_corners2.A(2,2)+(tform_corners3.A(2,2)-tform_corners2.A(2,2))*(dist_4-xref1)/(xref2-xref1);
    A_tform5 = [s_x3 0 tform_corners2.A(1,3); 0 s_y3 tform_corners2.A(2,3); 0 0 1];
    tform_corners5 = affinetform2d(A_tform5);
else
    tform_corners4 = tform_corners2;
    tform_corners5 = tform_corners2;
end

%%%%%% INCORRECT CONTROL %%%%%%%%%%%%
% R_control = [cosd(20) -sind(20) 0;sind(20) cosd(20) 0;0 0 1];
% %R_control = eye(3);
% T_control = [0 0.1 0]; % Remember it is the movement of 2 to 1
% tform_control = rigidtform3d(R_control,T_control);

%% Color PCs and Downsample

% tform_rigid2 = rigidtform3d([eul2rotm(rotm2eul(tform.Rotation)) tform.Translation'; 0 0 0 1]);
% A3 = [-0.034916288346678,0.023565017488715,0.999112377442522,0.075790253240288;
%     -0.999289763458403,0.013352630324858,-0.035237422020576,0.1;
%     -0.014171148695152,-0.999633131310765,0.023082056461581,-0.012916910073159;
%     0,0,0,1];
% tform_rigid3 = rigidtform3d(A3);

% % -------- Fantastic do not touch ---------------------------------------
% translation4 = [0.075790253240288,0.03,-0.012916910073159];
% %eulerAngles4 = rad2deg(rotm2eul(tform_rigid3.R));
% %eulerAngles4 = [-92.001163707193882 0.811974189604123 -88.677245270794401]; % actually calculated
% eulerAngles4 = [-89 0.811974189604123 -90.85];
% tform_rigid4 = rigidtform3d(eulerAngles4,translation4);
% % ----------------------------------------------------------------------

% Have a go at creating a camera intrinsics object
%intrinsics = cameraIntrinsics(cameraParams.FocalLength, cameraParams.PrincipalPoint, cameraParams.ImageSize,"RadialDistortion",cameraParams.RadialDistortion ...
    %,"Skew",cameraParams.Skew, "TangentialDistortion",cameraParams.TangentialDistortion);
%intrinsics = cameraParams.Intrinsics;

% Coloring the pointcloud
imPts_procr = transformPointsForward(tform_corners4,pc_in1.Location(:,2:3));
imPts_procr3 = transformPointsForward(tform_corners5,pc_in2.Location(:,2:3));

img1undist = undistortImage(img1,intrinsics2);
colors1 = impixel(img1undist,imPts_procr(:,1),imPts_procr(:,2));
colors_valid1 = validatecolor(uint8(colors1),"multiple");
pc_in1_colored = pointCloud(pc_in1.Location,"Color",colors_valid1);

img2undist = undistortImage(img2,intrinsics2);
colors2 = impixel(img2undist,imPts_procr3(:,1),imPts_procr3(:,2));
colors_valid2 = validatecolor(uint8(colors2),"multiple");
pc_in2_colored = pointCloud(pc_in2.Location,"Color",colors_valid2);


% Downsample 'Critical Step'
gridstep = 0.015; % Size of the grid filter in meters
%pcdenoise(pc_in2)
pc_out1 = pcdownsample(pcdenoise(pc_in1_colored),'gridAverage',gridstep); % gridAverage/nonuniform mandatory for color
pc_out2 = pcdownsample(pcdenoise(pc_in2_colored),'gridAverage',gridstep); % gridAverage/nonuniform mandatory for color

% [pc_out1, colormap1, pcindicies1] = fuseCameraToLidar(img1, pc_in1_clean, intrinsics2, tform_rigid4);
% [pc_out2, colormap2, pcindicies2] = fuseCameraToLidar(img2, pc_in2_clean, intrinsics2, tform_rigid4);

figure('Name','Downsampled PCs')
pcshowpair(pc_out2, pc_out1,"ColorSource","Color","MarkerSize",20)

% Camera Projection world points and imPts
%worldpoints = pc_out.Location;
%camProjection = estimateCameraProjection(imPts, worldpoints);
%camProjection2 = cameraProjection(intrinsics, tform_rigid4);

%% Calculate the Control tform with expected translation and Rotation
%%%%%% A = T * R, not R * T !!!!!!

% % pc1 transformation. Unnecessary because pc2 is being transformed.
% 
% Bbox_center1 = [((max(pc_in1_colored.Location(:,1))-min(pc_in1_colored.Location(:,1)))/2)+min(pc_in1_colored.Location(:,1)) 
%     ((max(pc_in1_colored.Location(:,2)-min(pc_in1_colored.Location(:,2))))/2)+min(pc_in1_colored.Location(:,2)) 
%     ((max(pc_in1_colored.Location(:,3))-min(pc_in1_colored.Location(:,3)))/2)+min(pc_in1_colored.Location(:,3))];
% T_global1 = [1 0 0 Bbox_center1(1,1);0 1 0 Bbox_center1(2,1);0 0 1 Bbox_center1(3,1);0 0 0 1];
% A_intrinsic1 = T_intrinsic * R_intrinsic;
% A_global1 = T_global1*A_intrinsic1;
% tform_predicted1 = rigidtform3d(A_global1);
% 
% % Bring pc1 to center
% pc_in1_centered = pctransform(pc_in1_colored,rigidtform3d(eye(3),-T_global1(1:3,4)'));
% pc_in1_transformed = pctransform(pc_in1_centered,tform_predicted1);
% %pc_in1_transformed = pctransform(pc_in1_centered,invert(tform_predicted));

%%%% pc2 WORKS %%%

Bbox_center2 = [((max(pc_out2.Location(:,1))-min(pc_out2.Location(:,1)))/2)+min(pc_out2.Location(:,1)) 
    ((max(pc_out2.Location(:,2)-min(pc_out2.Location(:,2))))/2)+min(pc_out2.Location(:,2)) 
    ((max(pc_out2.Location(:,3))-min(pc_out2.Location(:,3)))/2)+min(pc_out2.Location(:,3))];
T_global2 = [1 0 0 Bbox_center2(1,1);0 1 0 Bbox_center2(2,1);0 0 1 Bbox_center2(3,1);0 0 0 1];
A_intrinsic2 = T_intrinsic * R_intrinsic;
A_global2 = T_global2*A_intrinsic2;
tform_predicted2 = rigidtform3d(A_global2);
tform_centering2 = rigidtform3d(eye(3),-T_global2(1:3,4)');

% Bring pc2 to center
pc_out2_centered = pctransform(pc_out2,tform_centering2);
pc_out2_transformed = pctransform(pc_out2_centered,tform_predicted2);

% Multiply the two sequential tforms, tform2nd * tform1st
A_predicted2_total =  tform_predicted2.A * tform_centering2.A;
%%%%%% FINAL PREDICTED TFORM %%%%%%%%%
tform_control_final = rigidtform3d(A_predicted2_total);

%pc_in1_control = pctransform(pc_in2_colored,tform_control);
figure('Name','Control PC transformation')
pcshowpair(pc_in1_colored,pc_out2_transformed,"MarkerSize",20,"ViewPlane","ZY")

%% -----------------------------------------------------------------------
%  -----------------------------------------------------------------------
%  ------------- PC registration -----------------------------------------
%  -----------------------------------------------------------------------
%  -----------------------------------------------------------------------

%%%%%% ICP Algorithm %%%%%%%%%%%%%%
[icp_tform,movingReg,rmse_icp] = pcregistericp(pc_out2, pc_out1, ...
    Metric=Metric,InlierRatio=inlier_ratio,MaxIterations=max_iter, ...
    Tolerance=Tolerance,Verbose=true);

% ,InitialTransform=tform_control_final
%axang = rad2deg(rotm2axang(icp_tform.R));

%%%%%%%%% RMSE Control %%%%%%%%%%%%
% Translation RMSE
Trans_rmse = sqrt(mean((tform_control_final.Translation(1:3) - icp_tform.Translation(1:3)).^2));

% Rotation RMSE
euler_anglescontrol = rotm2eul(tform_control_final.R, 'XYZ');
euler_anglesicp = rotm2eul(icp_tform.R, 'XYZ');
Rotation_rmse = sqrt(mean((euler_anglescontrol - euler_anglesicp).^2));

RMSE_icpControl = [Trans_rmse Rotation_rmse];
RMSE_icpTotal = [rmse_icp Trans_rmse Rotation_rmse];

fprintf('Distance: %4.2f - %4.2f \n',dist_1,dist_2);
fprintf('R(1,1):                     %6.4f degrees\n', rad2deg(acos(icp_tform.R(1,1))));
fprintf('R(1,2):                    %6.4f degrees\n', rad2deg(-asin(icp_tform.R(1,2))));
fprintf('Translation:                [%6.4f, %6.4f, %6.4f]\n', icp_tform.Translation);
fprintf('ICP RMSE:                  %6.4f\n', rmse_icp)
fprintf('RMSE [Trans Rot]:           [%6.4f, %6.4f]\n\n',RMSE_icpControl)

figure('Name','Registered PCs')
pcshowpair(movingReg,pc_out1,"ColorSource","Color","MarkerSize",20)
% hold on
% plotTransforms(icp_tform.Translation,rotm2quat(icp_tform.R))

% FGR Algorithm
% gridsize = 0.3;
% [fgr_tform,rmse_fgr] = pcregisterfgr(pc_out2, pc_out1,gridsize);
% movingReg_fgr = pctransform(pc_out2,fgr_tform);
% %pcshowpair(movingReg_fgr,pc_in1,"AxesVisibility","on")

% mergesize = 0.01; % Grid size in meters
% merged = pcmerge(movingReg_fgr,pc_out1,mergesize);
% figure
% pcshow(merged,"AxesVisibility","on")

% --------------- For reflected PCs -------------------------
% xp = imPts(:,1);
% diffx = 320 - xp;
% x_new = 390 + diffx;
% 
% yp = imPts(:,2);
% y_new = yp - 15;
% 
% figure%('Colormap',[1 0 1; 0 0 1; 1 1 0])
% imshow(img)
% hold on
% plot(x_new, y_new, '.', 'Color', 'b')
% hold off
% -----------------------------------------------------------

%% World Pose PnP Problem
% %[imPts1, indicies_proj1] = projectLidarPointsOnImage(pc_out1, intrinsics2, tform);
% worldpoints1 = pc_in1_colored.Location;
% worldPose = estworldpose(imPts_procr,worldpoints1,intrinsics2);
% %[imPts2, indicies_proj2] = projectLidarPointsOnImage(pc_out2, intrinsics, tform);
% %worldpoints2 = pc_out2.Location;
% %worldPose2 = estworldpose(imPts2,worldpoints2,intrinsics);
% 
% %pcshow(worldpoints1,pc_in1_colored.Color);
% pcshowpair(pc_in2_colored, pc_in1_colored,"ColorSource","Color","MarkerSize",20)
% hold on
% %pcshow(worldpoints2,colormap2)
% plotCamera(Size=0.1,Orientation=worldPose.R',Location=worldPose.Translation);
% hold off

%% Other Stuff
% %% pcmatchfeatures
% % 
% % fixedFeature = extractFPFHFeatures(pc_out1);
% % movingFeature = extractFPFHFeatures(pc_out2);
% % %length(movingFeature)
% % 
% % [matchingPairs,scores] = pcmatchfeatures(fixedFeature,movingFeature,pc_out1,pc_out2);
% % %length(matchingPairs)
% % %mean(scores)
% % matchedPts1 = select(pc_out1,matchingPairs(:,1));
% % matchedPts2 = select(pc_out2,matchingPairs(:,2));
% % pcshowMatchedFeatures(pc_out1,pc_out2,matchedPts1,matchedPts2,"Method","montage")
% 
% %% Switching to Feature Tracking
% % 
% % %Image Feature tracking
% % 
% % gray1 = rgb2gray(img1);
% % gray2 = rgb2gray(img2);
% % gray1_cropped = imcrop(gray1,[0 60 555 350]);
% % gray2_cropped = imcrop(gray2,[0 60 555 350]);
% % 
% % %roi = drawrectangle("Position",[0 50 500 330]);
% % 
% % 
% % % MetricThreshold_Lamp = 200;
% % % MatchThreshold_Lamp = 9.0;
% % MetricThreshold_Direct = 200;
% % MatchThreshold_direct = 5.0;
% % 
% % 
% % Surfpoints1 = detectSURFFeatures(gray1,"MetricThreshold",MetricThreshold_Direct);
% % Surfpoints2 = detectSURFFeatures(gray2,"MetricThreshold",MetricThreshold_Direct);
% % 
% % [SurfFeat1,vpts1] = extractFeatures(gray1,Surfpoints1,"Method","SURF","FeatureSize",128);
% % [SurfFeat2,vpts2] = extractFeatures(gray2,Surfpoints2,"Method","SURF","FeatureSize",128);
% % 
% % %Threshold @ 0.4 worked well for translations
% % [indexPairs,matchmetric] = matchFeatures(SurfFeat1,SurfFeat2,"MatchThreshold",MatchThreshold_direct,"Unique",true,"MaxRatio",0.9);
% % 
% % matchedPoints1 = vpts1(indexPairs(:,1));
% % matchedPoints2 = vpts2(indexPairs(:,2));
% % 
% % figure; showMatchedFeatures(gray1,gray2,matchedPoints1,matchedPoints2);
% % legend("matched points 1","matched points 2");
% 
% %% Essential Matrix
% 
% [EssMatrix,EssInliers] = estimateEssentialMatrix(matchedPoints1,matchedPoints2,intrinsics2,MaxDistance=0.9);
% 
% EssinlierPoints1 = matchedPoints1(EssInliers);
% EssinlierPoints2 = matchedPoints2(EssInliers);
% figure
% showMatchedFeatures(img1,img2,EssinlierPoints1,EssinlierPoints2);
% title("Inlier Matches")
% 
% %% Fundamental Matrix
% 
% 
% [FundMatrix,FundInliers,FundStatus] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2,"Method","RANSAC");
% FundInlierPoints1 = matchedPoints1(FundInliers);
% FundInlierPoints2 = matchedPoints2(FundInliers);
% figure
% showMatchedFeatures(img1,img2,FundInlierPoints1,FundInlierPoints2);
% title("Inlier Matches")
