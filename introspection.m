classdef introspection < handle
    properties
        conf= struct('full',0); % display configuration
        trg  % instropected object, target object.
    end
    methods
        function obj = introspection(observer)
            fprintf("----------\nIntrospected object :\n");
            disp(observer);
            obj.trg = observer;
        end
        function introspect(obj, conf)
            % introspection of the object.
            if conf == obj.conf.full
                obj.angular_velocity();
                obj.linear_velocity();
                obj.inertial_lin_vel();
                %obj.homography();
                obj.homography_err();
                obj.homography_line();
            end
            % TODO set the precision of the display
            % using gca : https://fr.mathworks.com/help/matlab/ref/gca.html
        end
        function angular_velocity(obj)
            figure;
            for i=2:obj.trg.k
                for j=1:3
                    subplot(3,1,j);
                    hold on;
                    plot(obj.trg.t_support(i),...
                        obj.trg.omega{i}(j),'r.');
                end
            end
            subplot(3,1,1);title("Angular Velocity x axis");
            xlabel("t in seconds");ylabel("rad/s");
            subplot(3,1,2);title("Angular Velocity y axis");
            xlabel("t in seconds");ylabel("rad/s");
            subplot(3,1,3);title("Angular Velocity z axis");
            xlabel("t in seconds");ylabel("rad/s");
        end
        function linear_velocity(obj)
            figure;
            % the first element is not displayed.
            for i=2:obj.trg.k
                for j=1:3
                    subplot(3,1,j);
                    hold on;
                    plot(obj.trg.t_support(i),...
                        obj.trg.V0{i}(j),'r.');
                end
            end
            subplot(3,1,1);title("Linear Velocity in the reference frame x axis");
            xlabel("t in seconds");ylabel("m/s");
            subplot(3,1,2);title("Linear Velocity in the reference frame y axis");
            xlabel("t in seconds");ylabel("m/s");
            subplot(3,1,3);title("Linear Velocity in the reference frame z axis");
            xlabel("t in seconds");ylabel("m/s");
        end
        function inertial_lin_vel(obj)
            figure;
            % the first element is not displayed.
            for i=2:obj.trg.k
                for j=1:3
                    subplot(3,1,j);
                    hold on;
                    plot(obj.trg.t_support(i),...
                        obj.trg.Vin{i}(j),'r.');
                end
            end
            subplot(3,1,1);
            title("Inertial Linear Velocity x axis");
            xlabel("t in seconds");ylabel("m/s");
            subplot(3,1,2);
            title("Inertial Linear Velocity y axis");
            xlabel("t in seconds");ylabel("m/s");
            subplot(3,1,3);
            title("Inertial Linear Velocity z axis");
            xlabel("t in seconds");ylabel("m/s");
        end
        
        
        function homography(obj)
            figure;
            for i=2:obj.trg.k
                for j=1:9
                    subplot(3,3,j);
                    hold on;
                    hgt = obj.trg.H_gt{i}';  % needs to be transposed.
                    plot(obj.trg.t_support(i),...
                        hgt(j),'-r.')
                    h = obj.trg.H{i}';
                    plot(obj.trg.t_support(i),...
                        h(j),'-g.');
                end
            end
            for i=1:9
               subplot(3,3,i);title(sprintf("h%i",i));
               xlabel("t in seconds");ylabel("pixels");
               legend("ground truth","estimation")
               %ytickformat('%.1f');
               %ylim([-4 4]);
            end
        end
        
        
        function homography_err(obj)
            % plots the error on each coefficients of the estimation.
            figure;
            for i=2:obj.trg.k
                for j=1:9
                    subplot(3,3,j);
                    hold on;
                    err = obj.trg.H_gt{i}'-obj.trg.H{i}';
                    plot(obj.trg.t_support(i),...
                        err(j),'-r.')
                end
            end
            for i=1:9
               subplot(3,3,i);title(sprintf("h%i",i));
               xlabel("t in seconds");ylabel("pixels");
               legend("error : gt - estimate")
               %ytickformat('%.1f');
               %ylim([-4 4]);
            end
        end
        
        
        % line plots.
        function homography_line(obj)
            figure;
            % conversion to the matrix format.
            H = cell2mat(obj.trg.H);
            Hgt = cell2mat(obj.trg.H_gt);
            % time support
            t = 0:obj.trg.dt:obj.trg.t;
            % retrieves the coefficients as array.
            for i=0:8
               % i-th coefficent
               % extraction
               % computes the poisiton in the matrix of cells.
               q = fix(i/3);  % row
               r  = mod(i,3);  % column
               % rows and columns in the first matrix.
               row0 = q + 1 ;
               col0 = r + 1;
               hi  = H(row0, col0:3:end-(3-col0));
               hgti  = Hgt(row0, col0:3:end-(3-col0));
               % plotting
               subplot(3,3,i+1);
               plot(t, hgti, '-r.');
               hold on;
               plot(t, hi, '-g.');
               xlabel("t in seconds");ylabel("pixels");
               title(sprintf("h%i,%i",row0,col0));
               legend("ground truth","estimation")
               %ytickformat('%.1f');
               ylim([-4 4]);
            end
        end
        
    end
end

