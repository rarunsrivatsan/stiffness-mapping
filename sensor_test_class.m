% Create a sensor class and read and compare data from ati and optoforce sensors.
% TODO Calibration using the cal matrix. Should be included in this class
% file.

classdef sensor_test_class
    
    properties
        atiSub
        optoSub
        bias
        log
    end
    
    methods
        
        function obj=sensor_test_class()
            obj.atiSub=rossubscriber('/ft_sensor/netft_data');
            obj.optoSub=rossubscriber('/WrenchData');
        end
        
        function obj=biasATI(obj)
            obj.bias.ati.f=obj.atiSub.LatestMessage.Wrench.Force;
            obj.bias.ati.t=obj.atiSub.LatestMessage.Wrench.Torque;
        end
        
        function obj=biasOpto(obj)
            obj.bias.opto.f=obj.optoSub.LatestMessage.Wrench.Force;
        end
        
        function [atiMag,fvec,tvec]=readAtiSensor(obj)
            tvec.X=obj.atiSub.LatestMessage.Wrench.Torque.X-obj.bias.ati.t.X;
            tvec.Y=obj.atiSub.LatestMessage.Wrench.Torque.Y-obj.bias.ati.t.Y;
            tvec.Z=obj.atiSub.LatestMessage.Wrench.Torque.Z-obj.bias.ati.t.Z;
            
            fvec.X=(obj.atiSub.LatestMessage.Wrench.Force.X-obj.bias.ati.f.X);
            fvec.Y=(obj.atiSub.LatestMessage.Wrench.Force.Y-obj.bias.ati.f.Y);
            fvec.Z=(obj.atiSub.LatestMessage.Wrench.Force.Z-obj.bias.ati.f.Z);
            
            atiMag=sqrt((fvec.X)^2+(fvec.Y)^2+(fvec.Z)^2);
        end
        
        function [optoMag,fvec]=readOptoSensor(obj)
            fvec.X=(obj.optoSub.LatestMessage.Wrench.Force.X-obj.bias.opto.f.X)/1000;
            fvec.Y=(obj.optoSub.LatestMessage.Wrench.Force.Y-obj.bias.opto.f.Y)/1000;
            fvec.Z=(obj.optoSub.LatestMessage.Wrench.Force.Z-obj.bias.opto.f.Z)/1000;
            optoMag=sqrt((fvec.X)^2+(fvec.Y)^2+(fvec.Z)^2);
        end
        
        function obj=simPlot(obj)
            tic;
            obj.log.atiMag=readAtiSensor(obj);
            obj.log.optoMag=readOptoSensor(obj);
            obj.log.time=toc;
            figure;
            figA=plot(obj.log.time,obj.log.atiMag,'b');
            hold on;
            figO=plot(obj.log.time,obj.log.optoMag,'r');
            title('Force Readings in N');
            
            figure;
            figE=plot(obj.log.time,obj.log.atiMag-obj.log.optoMag,'k');
            title('Error Readings in N');
            
            count=2;
            while(1)
                set(figA,'XData',obj.log.time,'YData',obj.log.atiMag);
                set(figO,'XData',obj.log.time,'YData',obj.log.optoMag);
                set(figE,'XData',obj.log.time,'YData',obj.log.atiMag-obj.log.optoMag);
                
                obj.log.atiMag(count,:)=readAtiSensor(obj);
                obj.log.optoMag(count,:)=readOptoSensor(obj);
                obj.log.time(count,:)=toc;
                count=count+1;
                pause(0.01);
            end
        end
        
    end
    
end