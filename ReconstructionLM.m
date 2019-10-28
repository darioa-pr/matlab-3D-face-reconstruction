close all ;
%clear variables;
clear all;
clc;
%% Camera calibration parameters
calibrationSession_LM1 = load('calibrationSession_LM1.mat');
% load('calibrationSession_MR1.mat');
inParaLM1 = calibrationSession_LM1.calibrationSession.CameraParameters.CameraParameters1.IntrinsicMatrix ;
stereoParams_LM1 = calibrationSession_LM1.calibrationSession.CameraParameters;
%%Rectified images
imgLeftSet = imageSet('ipcv_project3/Calibratie 1/calibrationLeft');
imgMiddleSet = imageSet('ipcv_project3/Calibratie 1/calibrationMiddle');
imgRightSet = imageSet('ipcv_project3/Calibratie 1/calibrationRight');

imPersonLeftSet1= imageSet('ipcv_project3/subject1/subject1Left');
imPersonMiddleSet1= imageSet('ipcv_project3/subject1/subject1Middle');
imPersonRightSet1= imageSet('ipcv_project3/subject1/subject1Right');

imPersonLeft1= rgb2gray(readimage(imPersonLeftSet1, 1));  
imPersonMiddle1 = rgb2gray(readimage(imPersonMiddleSet1,1));
imPersonRight1 = rgb2gray(readimage(imPersonRightSet1,1));

%Sketch out the histogram (increase constrast) of the images for better edge detection
lowhigh = stretchlim(imPersonMiddle1, [.03 .65]);
JM = imadjust(imPersonMiddle1, lowhigh, []);
lowhigh = stretchlim(imPersonLeft1, [.09 .85]);
JL = imadjust(imPersonLeft1, lowhigh, []);

%% Removing background for the first image
%Apply Gaussian filter 
sigma = 2;
filteredJM = imgaussfilt(JM,sigma);
%Detecting edge of the sketched image
edgeJM = edge(filteredJM, 'Canny', [.15 .34]);
SE = strel ('disk',20);
%Applying close morph, fill region and fill hole function
JM_close = imclose(edgeJM, SE);
JM_filled1 = imfill(JM_close, [532 685]);
JM_mask = imfill(JM_filled1, 'holes');
%Filter out the background (non-CBA image) with the mask 
imPersonMiddle1(JM_mask == 0) =0;
% figure(11);
% imshow(imPersonMiddle1);

%% Removing background for the second image
sigma = 2;
filteredJL = imgaussfilt(JL,sigma);
%Detecting edge of the sketched image
edgeJL = edge(filteredJL, 'Canny', [.22 .5]);
SE1 = strel ('disk',47);
%Applying close morph, fill region and fill hole function
JL_close = imclose(edgeJL, SE1);
JL_filled1 = imfill(JL_close, [190 126]);
JL_mask = imfill(JL_filled1, 'holes');
%Filter out the background (non-CBA image) with the mask 
imPersonLeft1(JL_mask == 0) =0;
% figure(13);
% imshow(imPersonLeft1);

%% Adjust CBA for images
lowhigh = stretchlim(imPersonMiddle1, [.05 .99]);
J = imadjust(imPersonMiddle1, lowhigh, []);
lowhigh2 = stretchlim(imPersonLeft1, [.05 .999]);
L = imadjust(imPersonLeft1, lowhigh, []);

J = imgaussfilt (J,sigma);
L = imgaussfilt (L,sigma);

% figure(15)
% imshow(J);
% figure(16);
% imhist(J);
% figure(17)
% imshow(L);
% figure(18);
% imhist(L);
%-> comment: the code is unstable, the algorithm sometimes does not filter
%with gaussian filter and with sketch out the histogram

%% Rectifying images

[JLeft, JMiddle]= rectifyStereoImages(L, J, stereoParams_LM1, 'OutputView', 'full');
% figure(19);
% imshow(stereoAnaglyph(JLeft, JMiddle));
disparityRange = [224 352];


disLM1 = disparity(JLeft, JMiddle , 'DisparityRange', disparityRange, 'BlockSize', 15 , 'ContrastThreshold', 0.2, 'DistanceThreshold', 80, 'UniquenessThreshold', 0);                                                                                     
% figure(20);
% imshow(disLM1, disparityRange);
% colormap (gca, jet);
% colorbar;

%% Cleaning, filtering the Inf and NaN value
mask = (disLM1 > 224);
% Mask the image using bsxfun() function to multiply the mask by each channel individually.
unreliable= bsxfun(@times, disLM1, cast(mask, 'like', disLM1));
%mask(isnan(mask))= 0;
mask = (unreliable < 352);
filt_disp= bsxfun(@times, unreliable, cast(mask, 'like', unreliable));
%
for i=1300:1754
    filt_disp(:,i) = 0;
end
for i=1:550
    filt_disp(:,i) = 0;
end
for i=1150:1271
    filt_disp(i,:) = 0;
end
for i=1:105
    filt_disp(i,:) = 0;
end

xyzPoints = reconstructScene(filt_disp, stereoParams_LM1);
ptCloud = pointCloud(xyzPoints); 
% figure(22);
% pcshow(ptCloud);

ROI = [-180 0, -180 100,  400 650];
indices = findPointsInROI(ptCloud, ROI);
ptCloud2=select(ptCloud,indices);
ptCloud2 = removeInvalidPoints(ptCloud2);
ptCloud2_LM = pcdenoise(ptCloud2); 
% figure(23);
% pcshow(ptCloud2_LM);
% save ptCloud2_LM ptCloud2_LM;

% %%%%%%%%%%% we can add argument (numneighbours,threshold ) read pcdenoise help
% 
% x = double(ptCloud2.Location(:,1));
% y = double(ptCloud2.Location(:,2));
% z = double(ptCloud2.Location(:,3));
% 
% % Construct the interpolant:
% F = scatteredInterpolant(x,y,z);
% % Evaluate the interpolant at the locations (qx, qy). The corresponding value at these locations is qz:
% sx=min(x);   
% hx=max(x);
% sy=min(y);
% hy=max(y);
% d =1;
% [qx,qy] = meshgrid(sx:d:hx,sy:d:hy);
% qz = F(qx,qy);
% surf(qx,qy,qz);
% axis equal
% shading interp
% hold on;
% patch(x,y,z,'o');


