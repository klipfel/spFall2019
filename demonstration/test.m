%%
% for features detection and extraction
% https://www.mathworks.com/help/vision/feature-detection-and-extraction.html
% Reading an image
clear all;

img = imread("identifications.jpg");
%img = imread("traffic_sign.jpg");
img_2 = imrotate(img,20,'bilinear','crop');
imshow(img);
rectangle = drawrectangle;
roi = rectangle.Position;
figure;
imshow(img_2);
rectangle2 = drawrectangle;
roi2 = rectangle.Position;
img_c = rgb2gray(img);
img_c_2 = rgb2gray(img_2);
%%
% ORB  the technique in the paper
corners = detectORBFeatures(img_c,'ROI',roi);
corners2 = detectORBFeatures(img_c_2,'ROI',roi2);
%hold on;
%plot(points);
%%
% FAST

% Detector
% FAST seems to give better results ..
corners = detectFASTFeatures(img_c,'ROI',roi);
corners2 = detectFASTFeatures(img_c_2,'ROI',roi2);
%hold on;
%plot(corners);

%%

% Extractor
[features, valid_corners] = extractFeatures(img_c, corners);
[features2, valid_corners2] = extractFeatures(img_c_2, corners2);

% Matching
indexPairs = matchFeatures(features,features2);
matchedPoints1 = valid_corners(indexPairs(:,1),:);
matchedPoints2 = valid_corners2(indexPairs(:,2),:);
figure; showMatchedFeatures(img,img_2,matchedPoints1,matchedPoints2);

%% Perspective warping
clear all;
img = imread("identifications.jpg");
img_2 = imrotate(img,20,'bilinear');
theta = -20;
tform = affine2d([cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1]);
outputImage = imwarp(img_2,tform);
figure, imshow(outputImage);title("Reconstruction");
figure; imshow(img);title("original");
