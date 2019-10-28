close all ;
clear all;
clc;

%% Loading the pointClouds
ptCloudLM2 = load('ptCloud2_LM.mat');
ptCloudLM = ptCloudLM2.ptCloud2_LM;
ptCloudMR2 = load('ptCloud2_MR.mat');
ptCloudMR = ptCloudMR2.ptCloud2_MR;

gridSize = 1;
fixed = pcdownsample(ptCloudLM, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudMR, 'gridAverage', gridSize);

Trans = [cos(pi/6)  0 sin(pi/6)  0; ...
    0     1     0      0; ...
    -sin(pi/6)  0 cos(pi/6)  0; ...          
               0       -10    -20      1];
          
tform = affine3d(Trans);
ptCloud_MR_Transformed = pctransform(ptCloudMR,tform);
mergeSize =.6;
ptCloud = pcmerge(fixed,ptCloud_MR_Transformed, mergeSize);

ROI = [-180 -10, -170 90,  400 640];
indices = findPointsInROI(ptCloud, ROI);
ptCloudNew=select(ptCloud,indices);
ptCloudNew = removeInvalidPoints(ptCloudNew);
figure(22);
pcshow(ptCloudNew);

x = double(ptCloudNew.Location(:,1));
y = double(ptCloudNew.Location(:,2));
z = double(ptCloudNew.Location(:,3));

% Construct the interpolant:
F = scatteredInterpolant(x,y,z);
% Evaluate the interpolant at the locations (qx, qy). The corresponding value at these locations is qz:
sx=min(x);   
hx=max(x);
sy=min(y);
hy=max(y);
d =1;
[qx,qy] = meshgrid(sx:d:hx,sy:d:hy);
qz = F(qx,qy);

mask = (qz > 500);
% Mask the image using bsxfun() function to multiply the mask by each channel individually.
unreliable= bsxfun(@times, qz, cast(mask, 'like', qz));
%mask(isnan(mask))= 0;
mask = (unreliable < 650);
qz= bsxfun(@times, unreliable, cast(mask, 'like', unreliable));
qz(qz==0)= 650;

figure(24);
mesh(qx,qy,qz);
axis equal;
shading interp
hold on;
patch(x,y,z,'o');


