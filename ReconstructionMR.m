close all ;
%clear variables;
clear all;
clc;
%% Camera calibration parameters
calibrationSession_MR1 = load('calibrationSession_MR1.mat');
% load('calibrationSession_MR1.mat');
inParaMR1 = calibrationSession_MR1.calibrationSession.CameraParameters.CameraParameters1.IntrinsicMatrix ;
stereoParams_MR1 = calibrationSession_MR1.calibrationSession.CameraParameters;
%%Rectified images
imgLeftSet = imageSet('ipcv_project3/Calibratie 1/calibrationLeft');
imgMiddleSet = imageSet('ipcv_project3/Calibratie 1/calibrationMiddle');
imgRightSet = imageSet('ipcv_project3/Calibratie 1/calibrationRight');

imPersonLeftSet1= imageSet('ipcv_project3/subject1/subject1Left');
imPersonMiddleSet1= imageSet('ipcv_project3/subject1/subject1Middle');
imPersonRightSet1= imageSet('ipcv_project3/subject1/subject1Right');

% imPersonLeft1Color = readimage(imPersonLeftSet1, 1);
% imPersonMiddle1Color = readimage(imPersonMiddleSet1, 1);
% imPersonRight1Color = readimage(imPersonRightSet1, 1);

imPersonLeft1= rgb2gray(readimage(imPersonLeftSet1, 1));  
imPersonMiddle1 = rgb2gray(readimage(imPersonMiddleSet1,1));
imPersonRight1 = rgb2gray(readimage(imPersonRightSet1,1));

%Sketch out the histogram (increase constrast) of the images for better
%edge detection
lowhigh = stretchlim(imPersonMiddle1, [.03 .65]);
JM = imadjust(imPersonMiddle1, lowhigh, []);
lowhigh = stretchlim(imPersonRight1, [.08 .65]);
JR = imadjust(imPersonRight1, lowhigh, []);

%we need to remove the background before applying contrast and rectifying 
% %% Removing background for the first image
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
filteredJR = imgaussfilt(JR,sigma);
%Detecting edge of the sketched image
edgeJR = edge(filteredJR, 'Canny', [.1 .3]);
SE1 = strel ('disk',20);
%Applying close morph, fill region and fill hole function
JR_close = imclose(edgeJR, SE1);
JR_filled1 = imfill(JR_close, [820 750]);
JR_filled2 = imfill(JR_filled1, 'holes');
%Applying open morph to clean the top left corner noise (white board
%corner)
JR_mask = imopen(JR_filled2, SE1);
%Filter out the background (non-CBA image) with the mask 
imPersonRight1(JR_mask == 0) =0;
% figure(12);
% imshow(imPersonRight1);

%% Adjust CBA for images
lowhigh = stretchlim(imPersonMiddle1, [.04 .99]);
J = imadjust(imPersonMiddle1, lowhigh, []);
lowhigh2 = stretchlim(imPersonRight1, [.04 .99]);
K = imadjust(imPersonRight1, lowhigh2, []);

J = imgaussfilt (J,sigma);
K = imgaussfilt(K, sigma);

% figure(15)
% imshow(J);
% figure(16);
% imhist(J);
% figure(17)
% imshow(K);
% figure(18);
% imhist(K);


%% Rectifying images

[JMiddle, JRight]= rectifyStereoImages(J, K, stereoParams_MR1, 'OutputView', 'full');
% figure();
% imshow(stereoAnaglyph(JMiddle, JRight));
disparityRange = [204 348];

%We should check the values again for better result: maybe remove
%uniqueness threshold
disMR1 = disparity(JMiddle, JRight , 'DisparityRange', disparityRange, 'BlockSize', 15 , 'ContrastThreshold', 0.1, 'DistanceThreshold', 60, 'UniquenessThreshold', 5);                                                                                     
% figure(20);
% imshow(disMR1, disparityRange);
% colormap (gca, jet);
% colorbar;

%% Cleaning, filtering the Inf and NaN value
mask = (disMR1 > 204);
% Mask the image using bsxfun() function to multiply the mask by each channel individually.
unreliable= bsxfun(@times, disMR1, cast(mask, 'like', disMR1));
%mask(isnan(mask))= 0;
mask = (unreliable < 348);
filt_disp= bsxfun(@times, unreliable, cast(mask, 'like', unreliable));
%
for i=1300:1797
    filt_disp(:,i) = 0;
end
for i=1:550
    filt_disp(:,i) = 0;
end
for i=1150:1325
    filt_disp(i,:) = 0;
end
for i=1:105
    filt_disp(i,:) = 0;
end

xyzPoints = reconstructScene(filt_disp, stereoParams_MR1);
ptCloud = pointCloud(xyzPoints); 
%ptCloud2 = zeros(:,:,:);
ROI = [165 320, -180 100,  500 650];
indices = findPointsInROI(ptCloud, ROI);
ptCloud2=select(ptCloud,indices);
ptCloud2 = removeInvalidPoints(ptCloud2);
ptCloud2_MR = pcdenoise(ptCloud2); 
save ptCloud2_MR ptCloud2_MR;
%%%%%%%%%%% we can add argument (numneighbours,threshold ) read pcdenoise help
