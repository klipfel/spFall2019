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
                obj.homography();
            end
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
                    plot(obj.trg.t_support(i),...
                        obj.trg.H_gt{i}(j),'r.');
                    plot(obj.trg.t_support(i),...
                        obj.trg.H{i}(j),'g.');
                end
            end
            for i=1:9
               subplot(3,3,i);title(sprintf("h%i",i));
               xlabel("t in seconds");ylabel("pixels");
            end
        end
    end
end

