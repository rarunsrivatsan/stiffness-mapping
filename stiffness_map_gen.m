classdef stiffness_map_gen
    % stiffness_map_gen Generate a stiffness map of the probed region using EM tracker and Optoforce.
    % Creating the object sets the bias to the held force.
    %
    properties
        opto
        trak
        meanz
        outputRaw
        outputFilt
        plots
        fThresh=0.15;
        useMag
        fHandle
    end
    
    methods
        function obj=stiffness_map_gen(meanz)
            % Create subscribers and store the mean z data for use.
            % To do: Add a function to calibrate mean in this class.
            obj.opto.optoSub=rossubscriber('/WrenchData');
            pause(0.1);
            obj.opto.bias= obj.opto.optoSub.LatestMessage.Wrench.Force;
            
            obj.trak.sub=rossubscriber('/matlab_sensor1_msg');
            pause(0.1);
            obj.meanz=meanz;
            obj.plots.surfReady=0;
            obj.plots.quivReady=0;
            obj.plots.pcReady=0;
            obj.useMag=0;
        end
        
        function obj=initSurface(obj)
            % Creates the initial random surface and stores the handle to
            % modify it.
            xInit=ceil(10*rand(1,4));
            yInit=ceil(10*rand(1,4));
            zInit=ceil(10*rand(1,4));
            triInit=delaunay(xInit,yInit);
            obj.fHandle=figure;
            obj.plots.sHandle=trisurf(triInit,xInit,yInit,zInit);
            
            % Labelling
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Stiffness');
            title('Stiffness Map as Surface');
            shading interp
            grid on;
            view(-114,53);
            obj.plots.surfReady=1;
        end
        
        function obj=startSurface(obj)
            
            if ~obj.plots.surfReady
                obj=obj.initSurface();
            end
            
            count=1;
            while(ishghandle(obj.fHandle))
                % Obtain latest measurements and then subtract the bias
                % from it, store for further use.
                obj.outputRaw.fvec.X(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.X-obj.opto.bias.X)/1000;
                obj.outputRaw.fvec.Y(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Y-obj.opto.bias.Y)/1000;
                obj.outputRaw.fvec.Z(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Z-obj.opto.bias.Z)/1000;
                
                if obj.useMag
                    obj.outputRaw.optoMag(count)=sqrt((obj.outputRaw.fvec.X)^2+(obj.outputRaw.fvec.Y)^2+(obj.outputRaw.fvec.Z)^2);
                else
                    obj.outputRaw.optoMag(count)=obj.outputRaw.fvec.Z(count);
                end
                
                if obj.outputRaw.optoMag(count)>obj.fThresh
                    
                    % DO STUFF HERE according to readings
                    obj.outputFilt.force(count)=obj.outputRaw.optoMag(count);
                    [obj.outputFilt.x(count),obj.outputFilt.y(count),obj.outputFilt.z(count)]=fetchXYZtool(obj.trak.sub);
                    obj.outputFilt.stiffness(count)=abs(obj.outputFilt.force(count)./-(obj.outputFilt.z(count)-obj.meanz));
                    
                    if count>15
                        tri=delaunay(obj.outputFilt.x,obj.outputFilt.y);
                        set(obj.plots.sHandle,'faces',tri,'vertices',[obj.outputFilt.x(:) obj.outputFilt.y(:) obj.outputFilt.stiffness(:)],'facevertexcdata',obj.outputFilt.stiffness(:));
                    end
                    count=count+1;
                end
                
                pause(0.01);
            end
        end
        
        function obj=initQuiver(obj)
            obj.fHandle=figure;
            obj.plots.qHandle=quiver3(0,0,0,0,0,0);
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Stiffness');
            title('Stiffness Map as Vectors');
            grid on;
            view(-114,53);
            obj.plots.quivReady=1;
        end
        
        function obj=startQuiver(obj)
            
            if ~obj.plots.quivReady
                obj=obj.initQuiver();
            end
            
            count=1;
            while(ishghandle(obj.fHandle))
                % Obtain latest measurements and then subtract the bias
                % from it, store for further use.
                obj.outputRaw.fvec.X(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.X-obj.opto.bias.X)/1000;
                obj.outputRaw.fvec.Y(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Y-obj.opto.bias.Y)/1000;
                obj.outputRaw.fvec.Z(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Z-obj.opto.bias.Z)/1000;
                
                if obj.useMag
                    obj.outputRaw.optoMag(count)=sqrt((obj.outputRaw.fvec.X)^2+(obj.outputRaw.fvec.Y)^2+(obj.outputRaw.fvec.Z)^2);
                else
                    obj.outputRaw.optoMag(count)=obj.outputRaw.fvec.Z(count);
                end
                
                if obj.outputRaw.optoMag(count)>obj.fThresh
                    
                    % DO STUFF HERE according to readings
                    % Save the measurements
                    obj.outputFilt.force(count)=obj.outputRaw.optoMag(count);
                    % Fetch XYZ of tool point
                    [obj.outputFilt.x(count),obj.outputFilt.y(count),obj.outputFilt.z(count)]=fetchXYZtool(obj.trak.sub);
                    % Calculate stiffness and store
                    obj.outputFilt.stiffness(count)=obj.outputFilt.force(count)./-(obj.outputFilt.z(count)-obj.meanz);
                    % Set vector height according to the Stiffness
                    
                    % KNOWN BUG: Since the quiver uses vector components,
                    % the height of the arrow never exceeds 1. Should
                    % represent magnitude ideally, but it isn't the case
                    % right now.
                    set(obj.plots.qHandle,'XData',obj.outputFilt.x,'YData',obj.outputFilt.y,'ZData',ones(1,count).*obj.meanz,'UData',zeros(1,count),'VData',zeros(1,count),'WData',abs(obj.outputFilt.stiffness));
                    count=count+1;
                end
                
                pause(0.01);
            end
        end
        
        function obj=calibZ(obj)
            
            obj.fHandle=figure;
            obj.plots.scHandle=scatter(0,0);
            xlabel('Z Displacement (mm)');
            ylabel('Force (grams)');
            
            count=1;
            tic;
            while(ishghandle(obj.fHandle) && count<400)
                % Obtain latest measurements and then subtract the bias
                % from it, store for further use.
                obj.outputRaw.fvec.X(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.X-obj.opto.bias.X)/1000;
                obj.outputRaw.fvec.Y(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Y-obj.opto.bias.Y)/1000;
                obj.outputRaw.fvec.Z(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Z-obj.opto.bias.Z)/1000;
                
                if obj.useMag
                    obj.outputRaw.optoMag(count)=sqrt((obj.outputRaw.fvec.X)^2+(obj.outputRaw.fvec.Y)^2+(obj.outputRaw.fvec.Z)^2);
                else
                    obj.outputRaw.optoMag(count)=obj.outputRaw.fvec.Z(count);
                end
                
                if obj.outputRaw.optoMag(count)>obj.fThresh
                    
                    % DO STUFF HERE according to readings
                    % Save the measurements
                    obj.outputFilt.force(count)=obj.outputRaw.optoMag(count);
                    
                    % Fetch XYZ of tool point
                    [obj.outputFilt.x(count),obj.outputFilt.y(count),obj.outputRaw.z(count)]=fetchXYZtool(obj.trak.sub);
                    
                    % Z threshold so that outliers do not enter
                    % the Z calibration data
                    if obj.outputRaw.z(count)>50
                        continue;
                    else
                        obj.outputFilt.z(count)=obj.outputRaw.z(count);
                        set(obj.plots.scHandle,'XData',obj.outputFilt.z,'YData',obj.outputFilt.force);
                        count=count+1;
                    end
                end
                
                pause(0.01);
            end
            
            obj.meanz=mean(obj.outputFilt.z);
            disp('Mean Z is');
            disp(obj.meanz);
            disp('Deviation of Z is');
            disp(std(obj.outputFilt.z));
            
        end
        
        function obj=setfThresh(obj,f)
            obj.fThresh=f;
        end
        
        function obj=initPointCloud(obj)
            % Creates the initial random surface and stores the handle to
            % modify it.
            obj.fHandle=figure;
            obj.plots.pcHandle=scatter3(0,0,0);
            
            % Labelling
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
            title('Point Cloud Generation');
            axis equal;
            grid on;
            view(-166,19);
            obj.plots.pcReady=1;
        end
        
        function obj=startPointCloud(obj)
            if ~obj.plots.pcReady
                obj=obj.initPointCloud();
            end
            obj.outputFilt=[];
            obj.outputRaw=[];
            count=1;
            while(ishghandle(obj.fHandle))
                
                % Obtain latest measurements and then subtract the bias
                % from it, store for further use.
                obj.outputRaw.fvec.X(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.X-obj.opto.bias.X)/1000;
                obj.outputRaw.fvec.Y(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Y-obj.opto.bias.Y)/1000;
                obj.outputRaw.fvec.Z(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Z-obj.opto.bias.Z)/1000;
                
                if obj.useMag
                    obj.outputRaw.optoMag(count)=sqrt((obj.outputRaw.fvec.X)^2+(obj.outputRaw.fvec.Y)^2+(obj.outputRaw.fvec.Z)^2);
                else
                    obj.outputRaw.optoMag(count)=obj.outputRaw.fvec.Z(count);
                end
                
                if obj.outputRaw.optoMag(count)>obj.fThresh
                    
                    % DO STUFF HERE according to readings
                    obj.outputFilt.force(count)=obj.outputRaw.optoMag(count);
                    [obj.outputFilt.x(count),obj.outputFilt.y(count),obj.outputFilt.z(count)]=fetchXYZtool(obj.trak.sub);
                    set(obj.plots.pcHandle,'XData',obj.outputFilt.x,'YData',obj.outputFilt.y,'ZData',obj.outputFilt.z);
                    count=count+1;
                    
                end
                
                pause(0.01);
            end
            obj.plots.pcReady=0;
        end
        
        function obj=startPointCloudEM(obj)
            if ~obj.plots.pcReady
                obj=obj.initPointCloud();
            end
            
            count=1;
            while(ishghandle(obj.fHandle))
                
                % Obtain latest measurements and then subtract the bias
                % from it, store for further use.
                obj.outputRaw.fvec.X(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.X-obj.opto.bias.X)/1000;
                obj.outputRaw.fvec.Y(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Y-obj.opto.bias.Y)/1000;
                obj.outputRaw.fvec.Z(count)=(obj.opto.optoSub.LatestMessage.Wrench.Force.Z-obj.opto.bias.Z)/1000;
                
                if obj.useMag
                    obj.outputRaw.optoMag(count)=sqrt((obj.outputRaw.fvec.X)^2+(obj.outputRaw.fvec.Y)^2+(obj.outputRaw.fvec.Z)^2);
                else
                    obj.outputRaw.optoMag(count)=obj.outputRaw.fvec.Z(count);
                end
                
                if obj.outputRaw.optoMag(count)>obj.fThresh
                    
                    % DO STUFF HERE according to readings
                    obj.outputFilt.force(count)=obj.outputRaw.optoMag(count);
                    [obj.outputFilt.x(count),obj.outputFilt.y(count),obj.outputFilt.z(count)]=fetchXYZEM(obj.trak.sub);
                    set(obj.plots.pcHandle,'XData',obj.outputFilt.x,'YData',obj.outputFilt.y,'ZData',obj.outputFilt.z);
                    count=count+1;
                    
                end
                
                pause(0.01);
            end
        end
    end
end