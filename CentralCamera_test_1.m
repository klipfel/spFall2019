T1 = SE3(-0.1, 0, 0) * SE3.Ry(0.4);
cam1 = CentralCamera('name', 'camera 1', 'default','focal', 0.002, 'pose', T1);
T2 = SE3(0.1, 0, 0) * SE3.Ry(-0.4);
cam2 = CentralCamera('name', 'camera 2', 'default','focal', 0.002, 'pose', T2);
axis([-0.5 0.5 -0.5 0.5 0 1]);
cam1.plot_camera('color', 'b', 'label'); 
cam2.plot_camera('color', 'g', 'label');
P = [0 0 0.5]';
%plot_sphere(P, 0.03, 'b'); 
% ploting and projecting does the same thing at at the end.
p = cam1.project(P);
p1 = cam1.plot(P);
p2 = cam2.plot(P);
disp(p);  % in pixels
disp(p1);

%% For homographies
Plane = SE3(0,0,0.5)*SE3.Rx(0.1)*SE3.Ry(0.1)*SE3.Rz(0.1);
measures = mkgrid(4,0.5,'pose',Plane); % at least 8 measures 8 DOF
plot_sphere(measures, 0.003, 'b');
p1 = cam1.plot(measures);
p2 = cam2.plot(measures);
H = homography(p1, p2);  % for the control you want H to be equal to the identity.
disp(H);
scaler = det(H);
H = H/(scaler^(1/3));
det(H);
cam2.invH(H);  % estimates the pose and the plane from the homography expression.