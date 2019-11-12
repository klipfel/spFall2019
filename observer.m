classdef observer < handle
    % handle and value class : https:
    % //fr.mathworks.com/help/matlab/matlab_oop/comparing-handle-and-value-classes.html
    properties
        P0 % cells of measures in the reference frame.
        p0 % cells of measures projected on the reference image plane.
        p  % cells of measures projected on the moving image plane.
        % Pose.
        R  % cells of rotations.
        Psi % cells of the displacement if the current frame with 
        % regard to the reference.
        % Velocities. (IMU)
        AdjOmega % cells of the adjoint of the angular velocities of the current frame
        omega % angular velocities of the camera.
        % with regard to the reference frame, or camera
        V0  % cells of the linear velocity of the current camera in 
        % the reference frame.
        Vin % inertial linear velocity.
        % Estimations.
        H % cells of the estimated homography
        H0  % initialization for the homography.
        % possible initalization distribution for the homography.
        rand_distr = struct('uniform',1,'gaussian',2);
        rand_distr_chosen
        H_gt  % ground truth homographies.
        I_H  % cells of the innovation terms of the homography estimation.
        Gamma % cells of the estimated Gamma, the translational movement.
        %I_G  % cells of the innovation terms of the gamma estimation.
        % Gains : for the comuptation of the innovation.
        H_gain
        Gamma_gain
        % Errors
        Proj_err  % error on the projections, each cell is a column vector
        % of the error on each measure.
        % Cameras.
        ref_camera % reference camera.
        cur_camera % current camera, latest.
        FPS  % frame per seconds
        dt   % sampling time in seconds.
        k % current discrete time.
        t % continuous time.
        t_support  % time support of the simulation.
        T_simu % simulation time in seconds.
        % Possible tested transformations.
        tranf_dic = struct('uniform_rotation',1,...
            'static',2,'staticTrans',3,'staticRot',4,...
            'staticRotTrans',5,...
            'pstaticRot',6,'uniform_trans',7);
        % Plotting parameters.
        dimensions = struct('xmin',-10,'xmax',10,'ymin',-10,'ymax',10,...
            'zmin',0,'zmax',10);
        % outlier removal.
        thres  % threshold.
        Niter=0;  % number of iterations per frame.
        % with 0  it uses a separated estimation and innovation function
        % with > 0 a gradient attenuation function.
        % mode for the observer equation 
        mode = 2;  % default
        
        % Planar scene
        % plane altitude from the reference/ over the reference camera
        planar_scene_z = 20;
        n = [0 0 1]'; % normal unit vector of the immobile planar scene in the reference frame.
        
    end    
    methods
        
        % random gradient descent initialization.
        function H0 = rand_grad_d(obj,inf,sup, distr_type)
            if distr_type==obj.rand_distr.uniform
                fprintf("Uniform initialization\n.");
                H0 = inf + (sup-inf).*rand(3,3);
                % H0 = obj.scaling_to_SL3(H0);
            elseif distr_type==obj.rand_distr.gaussian
                fprintf("Gaussian initialization\n.");
                sigma = 0.0;
                % the noise for the affine terms has to be really small.
                sigma_affine = 0.0;
                H0 = [ 1 + sigma*randn(1), sigma *randn(1), sigma *randn(1);...
                    sigma *randn(1), 1 + sigma *randn(1), sigma *randn(1);...
                    sigma_affine *randn(1), sigma_affine *randn(1),1 + sigma *randn(1)];
            else
                error("The distribution asked for the random generation is invalid.");
            end
        end
        
        
        function obj = observer(varargin)
            % initialization of the observer.
            fprintf("Initialization of the observer .. \n");
            obj.P0 = {};
            
            % Homography initialization.
            obj.rand_distr_chosen = obj.rand_distr.gaussian; 
            h = obj.rand_grad_d(-1,1,obj.rand_distr_chosen);  % return H0.
            obj.H = {h};
            obj.H0 = h;
            fprintf("Random initialization for the homography:\n");
            disp(obj.H0);
            
            obj.H_gt = {eye(3)};
            obj.R = {eye(3)};
            obj.Psi = {[0 0 0]'};
            obj.Gamma = {eye(3)*0};
            obj.FPS = 30;
            obj.dt = 1/obj.FPS;
            obj.t_support = [];
            obj.k = 0;
            obj.T_simu = 10;
            % gains
            obj.H_gain = 4;  % default 4 
            obj.Gamma_gain = 1;  % default 1
            % cameras : need an intrinsic matrix K = I, normalize all 
            % the measure (in [-1;1]).
            T_ini = SE3(0,0,0); % sets initial position of the camera in 
            % the world frame.
%             obj.ref_camera = CentralCamera('name', 'reference', 'default',...
%                 'focal', 0.002, 'pose', T_ini);
            obj.ref_camera = CentralCamera('name', 'reference', ...
                'focal', 1, 'centre', [0,0], 'pose', T_ini);
%             obj.cur_camera = CentralCamera('name', 'current', 'default',...
%                 'focal', 0.002, 'pose', T_ini);
            obj.cur_camera = CentralCamera('name', 'current',...
                'focal', 1, 'centre', [0,0], 'pose', T_ini);
            % figure.
            figure;
            grid on;
            axis([obj.dimensions.xmin obj.dimensions.xmax...
                obj.dimensions.ymin obj.dimensions.ymax...
                obj.dimensions.zmin obj.dimensions.zmax]);
            % outlier removal.
            obj.thres = 1000;
            %  Variable number of inputs.
            for i=1:2:length(varargin)
                if strcmp(varargin{i},'Niter')
                    obj.Niter = varargin{i+1};
                end
            end
        end
        
        
        function simulation(obj)
            fprintf("Simulation started ....\n ");
            for ts=obj.dt:obj.dt:obj.T_simu+obj.dt
                % Updates the discete time.
                obj.k = obj.k + 1;
                obj.t = ts;
                obj.t_support = [obj.t_support ts];
                fprintf("--------\nt=%f s, k = %i ..\n",obj.t,obj.k);
                obj.evolution(obj.tranf_dic.uniform_trans);
                % measures on the planar scene.
                obj.capture_planar_scene();
                % Proprioceptive Sensors.
                obj.angular_velocity();
                obj.linear_velocity();
                obj.inertial_lin_vel();
                %  ground thruth.
                obj.ground_thruth();
                % Estimation
                % without gradient attenuation.
                if obj.Niter == 0
                    error("Use the gradient attenuation for Niter = 1 instead.");
                    obj.innovation();
                    obj.estimation();
                else  % with gradient attenuation.
                    obj.observation_corr();
                end
                % draws.
                obj.draw_rt();
            end
        end
        
        % Transformation or motion of the camera.
        function evolution(obj,type)
           % evolution of the camera.
           if type==obj.tranf_dic.uniform_rotation
               % Homogeneous transformation : always with respect to the
               % reference pose.
               F = 1;
               w = 2*pi*F;  % frequence.
               T = SE3(cos(obj.t*w),sin(obj.t*w),0)*SE3.rpy(0,0,obj.t*w);
           elseif type==obj.tranf_dic.static
               fprintf("Simulation for a static transformation.\n");
               T = SE3(0,0,0);
           elseif type==obj.tranf_dic.staticTrans
               fprintf("Simulation for a static translation transformation.\n");
               T = SE3(1,0,0);
           elseif type==obj.tranf_dic.staticRot
               fprintf("Simulation for a static rotation transformation.\n");
               T = SE3.rpy(0,0,0.5);
           elseif type==obj.tranf_dic.staticRotTrans
               fprintf("Simulation for a static rotation and translation transformation.\n");
               T = SE3.rpy(0.5,0,0)*SE3.rpy(0,0,0.5);
           elseif type==obj.tranf_dic.pstaticRot
               fprintf("Simulation for a pseudo static rotation transformation.\n");
               k_period = 200;  % period between each new transformation.
               if mod(obj.k, k_period)==0
                   % new transformation.
                   T = SE3.rpy(0,0,0.05)*obj.cur_camera.T;
               else
                   % do not change.
                   T = obj.cur_camera.T;
               end
           elseif type==obj.tranf_dic.uniform_trans
               fprintf("Simulation for a uniform translation transformation.\n");
               dmax = 10; % maximum move in x, or y directions.
               s = dmax*obj.t/obj.T_simu;  % move from the reference
               T = SE3(s,0,0);
           else
               error('The given evolution configuration is not supported.');
           end
           % Updates the camera pose.
           obj.cur_camera.T = T;
           % updates the parameters of the camera pose.
           [obj.R{obj.k+1},obj.Psi{obj.k+1}] = tr2rt(T);
        end
        
        
        function draw_rt(obj)
            % real time drawing of the simulation.
            % Cameras.
            obj.ref_camera.plot_camera('scale',0.2,'color', 'r', 'label');
            obj.cur_camera.plot_camera('scale',0.2,'color', 'b', 'label');
            % Camera planes.
            obj.ref_camera.plot(obj.P0{obj.k});
            obj.cur_camera.plot(obj.P0{obj.k});
            % pause(obj.dt);
        end
        
        % Camera simulation.
        function capture_planar_scene(obj)
            % planar scene.
            % first argument is the number of measures : n*n
            n = 4;
            obj.P0{obj.k} = mkgrid(n,10,'pose',SE3(0,0,obj.planar_scene_z));
            % projection on the image planes.
            obj.p0{obj.k} = obj.ref_camera.project(obj.P0{obj.k}); 
            obj.p{obj.k} = obj.cur_camera.project(obj.P0{obj.k});
        end
        
        %%%%% Sensor simulation.
        
        function angular_velocity(obj)
            % Rotations matrices from the previous and current poses.
            Rk = obj.R{obj.k};
            Rkn = obj.R{obj.k+1};
            % adjoint of the angular velocity
            obj.AdjOmega{obj.k} = 1/obj.dt*(Rk'*Rkn - eye(3));
            % Angular velocities.
            obj.omega{obj.k} = vex(obj.AdjOmega{obj.k});
        end
        
        
        function linear_velocity(obj)
            obj.V0{obj.k} = (obj.Psi{obj.k+1} - obj.Psi{obj.k})/obj.dt;
        end
        
        
        function inertial_lin_vel(obj)
            Rkn = obj.R{obj.k+1};
            Psidotk = obj.V0{obj.k};
            % base changement.
            obj.Vin{obj.k} = Rkn'*Psidotk;
        end
        
        
        % computation of the distance to the planar scene in the current
        % frame.
        function distance_to_plane(obj)
            % normal of the plane in the current frame.
            %nc
        end
        
        
        function delete(obj)
            disp("Destruction of the object.");
            % pause();
            close all;  %% closes the figures.
        end
        
        
        function ground_thruth(obj)
            % Ground thruth such that : H :  p |-> p0
            obj.H_gt{obj.k+1} = homography(obj.p{obj.k},obj.p0{obj.k});
            % Scaling to SL3.
            obj.H_gt{obj.k+1} = obj.scaling_to_SL3(obj.H_gt{obj.k+1});
        end
        
        
        function [H_SL3] = scaling_to_SL3(obj,H)
            H_SL3 = H/nthroot(det(H),3);
        end
        
        % Innovation without gradient attenuation.
        function innovation(obj)
            % Error on the estimate, for each measurement point.
            n = length(obj.P0{obj.k});  % number of measurements.
            % Innovation initialization.
            inn = 0*eye(3);
            % Error initialization.
            %obj.Proj_err{obj.k} = [];
            % iterating over the measurements.
            for i=1:n
                % Error for the measurement point Pi0.
                pi = [obj.p{obj.k}(:,i) ; 1]; % use homogeneous coordinates.
                pi0 = [obj.p0{obj.k}(:,i); 1];  % projection in the reference
                % image plane.
                ei = obj.H{obj.k}*pi; % estimation of the projection
                %ei = obj.H_gt{obj.k+1}*pi;
                % using the estimate.
                ei = ei/norm(ei,2);
                %ri = norm(ei - pi0/norm(pi0,2),2);
                ri = norm(ei - pi0/norm(pi0,2),2);  % this error is big when no normalized?
                rif = obj.m_estimator(ri);  % filtered error.
                inn = inn - obj.H_gain*rif*(eye(3)-ei*ei')*pi0*ei'; 
            end
            obj.I_H{obj.k} = inn;
            % obj.I_H{obj.k} = obj.mat_sat(-1,1,inn); % with saturation
            %obj.I_H{obj.k} = inn;
            % obj.I_H{obj.k} = mod(inn,1);
        end
        
        
        function error(obj)
        end
        
        
        function Ad = Adjoint(obj,H,X)
            eps = 1.0e-10;
            if (abs(trace(X))> eps)  % the innovation is not really in sl(3)
                warning("In Adjoint : X not in sl(3).");
                fprintf("Trace(X) = %f.\n", trace(X));
                %pause();
            end
            if (abs(det(H)-1)> eps)
                warning("In Adjoint : H not in SL(3).");
                fprintf("det(H) = %f.\n", det(H));
                %pause();
            end
            % Ajoint operator.
            Ad = H*X/H;
        end
        
        % Observation without gradient attenuation.
        function estimation(obj)
            Hhat = obj.H{obj.k};
            inn = obj.I_H{obj.k};% innovation;
            AdjOm = obj.AdjOmega{obj.k};
            % Translational movement estimation.
            dGamma = obj.Gamma{obj.k}*AdjOm ...
                - obj.Gamma_gain*obj.Adjoint(Hhat',inn);
            obj.Gamma{obj.k+1} = obj.Gamma{obj.k} + obj.dt*dGamma;
            % in sl(3).
            % obj.Gamma{obj.k+1} = obj.Gamma{obj.k+1} - trace(obj.Gamma{obj.k+1})/3*eye(3);
            % Homography estimation.
            Gammahat = obj.Gamma{obj.k};
            dH = Hhat*(AdjOm + Gammahat - (1/3)*trace(Gammahat)*eye(3)) ...
                - inn*Hhat;
            obj.H{obj.k+1} = Hhat + obj.dt*dH;
            % scaling to SL(3).
            obj.H{obj.k+1} = obj.scaling_to_SL3(obj.H{obj.k+1}); % complex number appear.
        end
        
        %%% Observation with gradient attenuation.
        function observation_corr(obj)
            % The innovations are not stored on that case. (TODO)
            % Data of the frame Ak.
            AdjOm = obj.AdjOmega{obj.k};
            n = length(obj.P0{obj.k});
            % gains for the corrections.
            k_H = obj.H_gain/obj.Niter;
            k_G = obj.Gamma_gain/obj.Niter;
            ds = obj.dt/obj.Niter;  % integration step.
            % Initialization of the parameters which changes.
            %Hhat = obj.H{obj.k};
            Hhat = obj.H{obj.k};
            Gammahat = obj.Gamma{obj.k};
            % Correction steps.
            for k=1:obj.Niter
                % Innovation.
                inn = 0*eye(3);
                for i=1:n
                    % Error for the measurement point Pi0.
                    pi = [obj.p{obj.k}(:,i) ; 1]; % use homogeneous coordinates.
                    pi0 = [obj.p0{obj.k}(:,i); 1];  % projection in the reference
                    % image plane.
                    ei = Hhat*pi; % estimation of the projection
                    %ei = obj.H_gt{obj.k+1}*pi;
                    % using the estimate.
                    ei = ei/norm(ei,2);
                    %ei = ei/ei(3);
                    %ri = norm(ei - pi0/norm(pi0,2),2);
                    %ri = norm(ei - pi0/norm(pi0,2),2);  % this error is big when no normalized?
                    ri = norm(ei - pi0,2);
                    rif = obj.m_estimator(ri);  % filtered error.
                    inn = inn - k_H*rif*(eye(3)-ei*ei')*pi0*ei';
                    %inn = obj.mat_sat(-1,1,inn);
                end
                % Observation.
                % Translational movement estimation.
                if obj.mode == 1
                    dGamma = obj.lie_brackets(Gammahat,AdjOm) ...
                        - k_G*obj.Adjoint(Hhat',inn);
                    nGamma = obj.to_sl3(Gammahat + ds*dGamma);
                else
                    dGamma = Gammahat*AdjOm ...
                        - k_G*obj.Adjoint(Hhat',inn);
                    nGamma = Gammahat + ds*dGamma;
                end
                % Homography estimation.
                dH = Hhat*(AdjOm + Gammahat - (1/3)*trace(Gammahat)*eye(3)) ...
                    - inn*Hhat;
                nH = obj.scaling_to_SL3(Hhat + ds*dH);
                % Updates.
                Hhat = nH;
                Gammahat = nGamma;
            end
            % Stores the estimate a time k+1.
            obj.H{obj.k+1} = nH;
            obj.Gamma{obj.k+1} = nGamma;
        end
        
        
        function z = lie_brackets(obj, x, y)
            % x, and y must be in sl(3) in this case.
            z = x*y - y*x;
        end
        
        
        % To sl(3) if not.
        function y = to_sl3(obj, x)
            y = x - trace(x)*eye(3)*1/3;
        end
        
        
        function errf = m_estimator(obj, err)
            % removes outliers in the measurements points : points which have
            % a too big error.
            if abs(err) <= obj.thres
                errf = (1 - (err/obj.thres)^2)^2; 
            else
                errf = 0;
            end
            %disp(errf);
        end
        
        
        function sat = mat_sat(obj,a,b,mat)
            % saturation between a and b of the coefficient of a matrix mat.
            sat = mat + (mat>b).*(-mat + b) + (mat<a).*(-mat + a);
        end
        
        
    end
end

