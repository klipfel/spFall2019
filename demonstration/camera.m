classdef camera < handle

    properties
        file % media file of the picture, video
        stream
        ref_frame
        ref_frame_gray
        curr_frame
        curr_frame_gray
        roi  % ROI of the planar scene
    end
    
    methods
        function obj = camera(varargin)
            % iteration over varargin
            for i=1:2:length(varargin)
                if strcmp(varargin{i},'file')
                    obj.file = varargin{i+1};
                end
            end
            % opens the stream
            obj.stream = imread(obj.file);
            obj.ref_frame = obj.stream;
            obj.curr_frame = obj.ref_frame;
            obj.ref_frame_gray = rgb2gray(obj.ref_frame);
            % asks to identify the planar scene in the reference frame.
            figure, imshow(obj.ref_frame);
            title("Choose the planar scene");
            call = drawrectangle;
            obj.roi = call.Position;
        end
        
        function get_next_frame(obj,mode)
            if mode=="static"
                obj.curr_frame = obj.ref_frame;
            elseif mode=="static_rotation"
                angle_d = -0.5;  % in degree
                rot = [cosd(angle_d), -sind(angle_d),0;...
                    sind(angle_d), cosd(angle_d), 0;...
                    0 0 1];
                tf = projective2d(rot);
                obj.curr_frame = imwarp( obj.ref_frame,tf, 'OutputView',imref2d(size(obj.ref_frame)));
            else
                error("In get_next_frame : mode not supported.");
            end
            obj.curr_frame_gray = rgb2gray(obj.curr_frame);
        end
        
    end
end

