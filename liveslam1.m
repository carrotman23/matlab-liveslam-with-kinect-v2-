colorDevice = imaq.VideoDevice('kinect',1);
depthDevice = imaq.VideoDevice('kinect',2);


  colorDevice();
  depthDevice();


  colorImage = colorDevice();
  depthImage = depthDevice();

  ptCloud = pcfromkinect(depthDevice, depthImage, colorImage);

  ptCloudRef = ptCloud;

  colorImage = colorDevice();

  depthImage = depthDevice();

  ptCloud = pcfromkinect(depthDevice, depthImage, colorImage);

  ptCloudCurrent = ptCloud;

  gridSize = 0.1;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize); % 점 갯수로 해상도 조절 함수
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(ptCloudCurrent,tform);

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

accumTform = tform; 

figure


hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

for i = 3:45
    tic;
    colorImage = colorDevice();
    depthImage = depthDevice();

    ptCloud = pcfromkinect(depthDevice, depthImage, colorImage);


    ptCloudCurrent = ptCloud;
       
    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    
    % Apply ICP registration.
    tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    drawnow('limitrate') 
    toc;
end

angle = -pi/40;
A = [1,0,0,0;...
     0, cos(angle), sin(angle), 0; ...
     0, -sin(angle), cos(angle), 0; ...
     0 0 0 1];
ptCloudScene = pctransform(ptCloudScene, affine3d(A));
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
        'Parent', hAxes)
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')


  release(colorDevice);
  release(depthDevice);

