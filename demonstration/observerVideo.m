classdef observerVideo < handle
    
    properties
        H  % estimated homograhies
        Gamma
        % Gains
        Gamma_gain
        H_gain
        H_gain0
        camera
        t
        k
        T
        dt
        FPS
        p0
        p0_pred
        n % number of measurements for a given frame
        N %  number of chosen measurements
        outlier_thres
        % Gradient descent
        Niter
    end
    
    methods
        function obj = observerVideo(varargin)
            % Initialization
            obj.H = {eye(3)};
            %obj.H = {obj.rand_ini("gaussian")};
            obj.Gamma = {0*eye(3)};
            % Camera
            obj.camera = camera("file","mariage.jpg");
            % Filter parameters
            obj.outlier_thres = 500;%0.2;
            obj.H_gain = 0.0001;
            obj.H_gain0 = 0.001;
            obj.Gamma_gain = 1;
            % Simulation parameters
            obj.T = 10;
            obj.FPS = 30;
            obj.dt = 1/obj.FPS;
            obj.k=0;
            % Measures 
            obj.p0 = {};
            obj.p0_pred = {};
            obj.n = [];
            % Gradient descent
            obj.Niter=1;
        end
        
        
        function run(obj)
           for tk=0:obj.dt:obj.T
               obj.k=obj.k+1;
               fprintf("------------"+newline+"TIME : %f"+newline,tk);
               obj.camera.get_next_frame("static_rotation");
               obj.observation(); 
               obj.correction();
           end
        end
        
        
        function [ref_pred_g, ref_pred] = prediction(obj)
            % prediction of the reference frame from the current.
            Hk = obj.H{obj.k};
            tform = projective2d(Hk);
            % Estimate of the reference frame.
            ref_pred = imwarp(obj.camera.curr_frame,tform,'OutputView', imref2d(size(obj.camera.ref_frame)));
            ref_pred_g = rgb2gray(ref_pred); 
        end
        
        function  observation(obj)
            % produces the observations, or measures on the planar scene.
            ref_g = obj.camera.ref_frame_gray;
            [ref_pred_g,ref_pred] = obj.prediction();
            % ORB keypoints detection
            points_ref = detectORBFeatures(ref_g,'ROI',obj.camera.roi);
            points_ref_pred = detectORBFeatures(ref_pred_g,'ROI',obj.camera.roi);
            % Extraction of the features
            [features_ref, valid_points_ref] = extractFeatures(ref_g, points_ref);
            [features_ref_pred, valid_points_ref_pred] = extractFeatures(ref_pred_g, points_ref_pred);
            % Matching
            % MAX_RATIO to reduce the number of matches : https://fr.mathworks.com/help/vision/ref/matchfeatures.html
            indexPairs = matchFeatures(features_ref,features_ref_pred,'MaxRatio',0.3);
            matchedPoints1 = valid_points_ref(indexPairs(:,1),:); % matched features expressed in the reference frame coordinates
            matchedPoints2 = valid_points_ref_pred(indexPairs(:,2),:);
            if matchedPoints1.Count==0 % no measures
                error("ERROR in the observation : No measures extracted.");
            end
            obj.n = [obj.n matchedPoints1.Count]; 
            fprintf("Number of points measured : %d."+newline,obj.n(obj.k));
            p0 = matchedPoints1.Location(1:end,:);
            p0_pred = matchedPoints2.Location(1:end,:);
            % normalization
            [height,width,channels] = size(ref_g);
            %obj.p0{obj.k} = (p0-[width/2 height/2])./[width/2 height/2];
            %obj.p0_pred{obj.k} = (p0_pred-[width/2 height/2])./[width/2 height/2];
            obj.p0{obj.k} = p0;
            obj.p0_pred{obj.k} = p0_pred;
            % display
            pause(obj.dt);
            close all;
            figure; showMatchedFeatures(obj.camera.ref_frame,ref_pred,matchedPoints1,matchedPoints2);
            legend('matched points 1','matched points 2');
            title("Matching of the truth and the prediction of the reference");
        end
        
        
        function correction(obj)
            % Outputs the estimate of the homography after correction.
            Hhat = obj.H{obj.k};
            % innovation
            inn = 0*eye(3);
            n_obs = length(obj.p0{obj.k});
            for i=1:n_obs
                % Error for the measurement point Pi0.
                pi0 = [obj.p0{obj.k}(i,:)' ; 1]; % use homogeneous coordinates.
                ei = [obj.p0_pred{obj.k}(i,:)'; 1]; % estimation of the projection
                % using the estimate.
                ei = ei/norm(ei,2);
                ri = norm(ei - pi0,2)
                rif = obj.m_estimator(ri);  % filtered error.
                inn = inn - obj.H_gain0/n_obs*rif*(eye(3)-ei*ei')*pi0*ei'
                %inn = inn - obj.H_gain*rif*(eye(3)-ei*ei')*pi0*ei'
            end
            AdjOm = 0*eye(3);
            % Translational movement estimation.
            dGamma = obj.Gamma{obj.k}*AdjOm ...
                - obj.Gamma_gain*obj.Adjoint(Hhat',inn)
            obj.Gamma{obj.k+1} = obj.Gamma{obj.k} + obj.dt*dGamma;
            % in sl(3).
            % Homography estimation.
            Gammahat = obj.Gamma{obj.k};
            dH = Hhat*(AdjOm + Gammahat - (1/3)*trace(Gammahat)*eye(3)) ...
                - inn*Hhat
            obj.H{obj.k+1} = Hhat + obj.dt*dH;
            % scaling to SL(3).
            obj.H{obj.k+1} = obj.scaling_to_SL3(obj.H{obj.k+1}); % complex number appear.
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
        
        
        function errf = m_estimator(obj, err)
            % removes outliers in the measurements points : points which have
            % a too big error.
            if abs(err) <= obj.outlier_thres
                errf = (1 - (err/obj.outlier_thres)^2)^2; 
            else
                errf = 0;
            end
        end
        
        
        function [H_SL3] = scaling_to_SL3(obj,H)
            H_SL3 = H/nthroot(det(H),3);
        end
        
        
        % random initialization of the homography.
        function H0 = rand_ini(obj,distr_type)
            if distr_type=="uniform"
                fprintf("Uniform initialization\n.");
                inf = -1;sup = 1;
                H0 = inf + (sup-inf).*rand(3,3);
                % H0 = obj.scaling_to_SL3(H0);
            elseif distr_type=="gaussian"
                fprintf("Gaussian initialization\n.");
                sigma = 1e-5;
                % the noise for the affine terms has to be really small.
                sigma_affine = 0.0;
                H0 = [ 1 + sigma*randn(1), sigma *randn(1), sigma *randn(1);...
                    sigma *randn(1), 1 + sigma *randn(1), sigma *randn(1);...
                    sigma_affine *randn(1), sigma_affine *randn(1),1 + sigma *randn(1)];
            else
                error("The distribution asked for the random generation is invalid.");
            end
        end
        
    end
end

