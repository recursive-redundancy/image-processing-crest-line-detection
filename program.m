%CSCE 4240
%Dr. Yuan
%Final Project Implementation
%Jeff Foster
%Daniel Morgan

% Program which reads in a LiDAR image of dunes, detects the edges of the
% image using four different basic edge detection methods.
% Then creates binary images based on the edge threshold.    
% the filter assigned to 'se' can be is static in our implementation, but could bemodified to increase/ decrease the dilation size. 
% An increased dilation results in increased accuracy so long as the crests do not overlap.
% Tolerance for each respective filter can also be adjusted using matlab edge() function arguments, increasing/decreasing the responsivenes to smaller elements. 
% Decreasing the tolerance can result in artifacts.
% We have used static values here as we only had the one image to
% interpret, though a more dynamic approach would be more versatile for
% interpreting a vast array of images.

 % load the LIDAR image (should be located in same folder as this script file)
img1 = imread('duneLiDARs.png');
%create image filter for dilation
se = strel('square',15);

%sobel edge
sobel=edge(img1);
%applies morphological op to sobel edge map
sobel = imdilate(sobel,se);
sobel = bwmorph(sobel,'thin',Inf);
sobel = bwareaopen(sobel,20); % we used a static max of 20 pixels for this situation

%canny edge
can = edge(img1,'Canny',.69); % we used a static threshold of .69 for this situation
%applies morphological op to canny edge map
can = imdilate(can,se);
can = bwmorph(can,'thin',Inf);
can = bwareaopen(can,20);

%prewitt edge
prew = edge(img1,'Prewitt');
%applies morphological op to prewitt edge map
prew = imdilate(prew,se);
prew = bwmorph(prew,'thin',Inf);
prew = bwareaopen(prew,20);

%wavelet implementation
%converts to wavelet channels
[c,s]=wavedec2(img1,2,'haar');
[H1,V1,D1] = detcoef2('all',c,s,1);
%reconstruct edge map without approx. channel
wave=idwt2([],H1,V1,D1,'haar');
%normalize the edgemap so the edges can be clearly seen(modified for uint 16)
normalWave=uint16(65535*mat2gray(wave));
%convert to binary image based on similar threshold to the above tests
wave=im2bw(normalWave,0.69);
%Perform morphological operations on wavelet edge map
wave = imdilate(wave,se);
wave = bwmorph(wave,'thin',Inf);
wave = bwareaopen(wave,20);

%composite the edge maps onto the original image
fusePrew = imfuse(img1,prew);
fuseCan = imfuse(img1,can);
fuseSobel = imfuse(img1,sobel);
fuseWavelet = imfuse(img1,wave);

%display the results of the 4 techniques
%create figure to display the results in
figure;
subplot(2,2,1);
imagesc(fusePrew);
title('Prewitt Edge Detection');
subplot(2,2,2);
imagesc(fuseSobel);
title('Sobel Edge Detection');
subplot(2,2,3);
imagesc(fuseWavelet);
title('Wavelete Edge Detection (supress approx.)');
subplot(2,2,4);
imagesc(fuseCan);
title('Canny Edge Detection');