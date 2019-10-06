%%  Creation of the world :  stationary planar scene, reference camera.
clear all;
% struct containing all the worls data.
world = struct();
% figure
dim = 10;
figure; grid on;axis([-dim dim -dim dim 0 dim]);
% Data storage.
% plane.
scene = SE3(0,0,5);
n = 4;  % at least 3
length_plane = 10;
measures = mkgrid(n,length_plane,'pose',scene); % generates the measures.
world.measures = {measures};
plot_sphere(measures, 0.03, 'b');
% Reference pose of the camera.
cam_0 = CentralCamera('name', 'reference', 'default','focal', 0.002, 'pose', SE3(0,0,0));
world.cam_0 = cam_0;
cam_0.plot_camera('scale',0.2,'color', 'b', 'label');
% current camera. at discrete instant k.
cam_k = CentralCamera('name', 'current', 'default','focal', 0.002, 'pose', SE3(0,0,0));
world.cam_k = cam_k;
%%  simulation parameters.
T = 10; % simulation time.
FPS = 20; % in Hz.
dt = 1/FPS; % in seconds.
k = 0;
for t=dt:dt:T+dt
   pause(dt)
   k = k +1;
   fprintf("--------\nt=%f, k = %i ..\n",t,k);
   %cam_k.T = cam_0.T;
   %cam_k.rpy(dt,0,0); % adds an interial rotation.
   cam_k.T = SE3(2*cos(t),2*sin(t),0)*SE3.rpy(0,0,t); 
   %cam_k.move;
   cam_k.plot_camera('scale',0.2,'color', 'r', 'label');
   p0 = cam_0.plot(measures);
   pk = cam_k.plot(measures);
   % Ground truth.
   H = homography(p0, pk);
   scaler = det(H);
   H = H/(scaler^(1/3));
   det(H);
   disp(H)
   % observer
   rotmZYX = eul2rotm([t 0 0]);
   %disp(rotmZYX)
   %disp(cam_k.T)
end
%% Observer simulation.