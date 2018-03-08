clc
clear all
% myev3=legoev3('wifi','172.20.10.2','001653481485');
myev3=legoev3('usb');

if exist('ultrasonic_sensor.txt', 'file')==2
  delete('ultrasonic_sensor.txt');
end
fileID = fopen('ultrasonic_sensor.txt','w');
kp=1;
ki=0;
kd=3;

kpdis=4;
kidis=0.1;
kddis=10;

   



ix=0;
inputA=60;
inputB=60;
ine1=0;
dee1=0;
lasterror1=0;
ine2=0;
dee2=0;
lasterror2=0;

inerr=0;
deer=0;
lasterr=0;


mymotor1=motor(myev3,'A');
start(mymotor1);
mymotor1.Speed=inputA;

mymotor2=motor(myev3,'D');
start(mymotor2);
mymotor2.Speed=inputB;

ls(1)=readInputDeviceREADY_RAW(myev3,1,0,1);
ls(3)=readInputDeviceREADY_RAW(myev3,3,0,1);
ls=zeros(1,4);
diff=ls(1)-ls(3);

counter=0;
t=clock;

while(true)
   
   % obtain raw data
    ls(1)=readInputDeviceREADY_RAW(myev3,1,0,1);
    ls(2)=readInputDeviceREADY_RAW(myev3,2,0,1);
    ls(3)=readInputDeviceREADY_RAW(myev3,3,0,1);
   

    ls(4)=0;
    for i=1:10
        ls(4)=readInputDeviceREADY_RAW(myev3,4,0,1)+ls(4);
    end
    ls(4)=ls(4)/10;
    ls(4)=round((ls(4)-104.6627)/3.7776);
    ls(1:3)=ls(1:3)/35;
    
    % stop at red tape 
    
    if ((ls(2)>=90 && ls(2)<=99)) 
        if ((ls(1)>=90 && ls(1)<=100) && (ls(3)>=90 && ls(3)<=100))
           mymotor1.Speed=0;
           mymotor2.Speed=0;
           break;
        end
    end


% platooning distance =20 cm; pid; ultrasonic sensor
display(ls);
errdis=ls(4)-20;
inerr=inerr+errdis;
deer=errdis-lasterr;
turndis=kpdis*errdis+kidis*inerr+kddis*deer;

% path following; straight line    
    error1=(ls(1)-ls(3)+diff);   % calculate error
    ine1=ine1+error1;                % integral error
    dee1=error1-lasterror1;           % derivative error   
    turn1=kp*error1+ki*ine1+kd*dee1;  % PID error
     
    error2=(ls(3)-ls(1)-diff);   % calculate error
    ine2=ine2+error2;                % integral error
    dee2=error2-lasterror2;           % derivative error   
    turn2=kp*error2+ki*ine2+kd*dee2;  % PID error
    
%     accum_count=accum_count+1;
%     
%     if accum_count==50
%         ine1=0;
%         ine2=0;
%         accum_count=0;
%     end
     
if ls(4)>25
    powerA=inputA+turn2;                % new input    
    powerB=inputB+turn1;
    if powerA>70                  % motor threshold from 0-100 
        powerA=70;
    elseif powerA<60
        powerA=60;
    end
    if powerB>70
        powerB=70;
    elseif powerB<60
        powerB=60;
    end
else
     inputA=30;
     inputB=30;
    powerA=inputA+turn2+turndis;
    powerB=inputB++turn1+turndis;

    if powerA>50                  % motor threshold from 0-100 
        powerA=50;
    elseif powerA<0
        powerA=0;
    end
    if powerB>50
        powerB=50;
    elseif powerB<0
        powerB=0;
  
    end
end
    mymotor1.Speed=powerA;
    mymotor2.Speed=powerB;
    
    
    lasterror1=error1;
    lasterror2=error2;
    lasterr=errdis;
    % ultrasonic sensor stop before
    
    time=etime(clock,t);
    if time>=counter
        a=strcat('@',num2str(ls(4)),'@',num2str(counter));
        fprintf(fileID,a);
        fprintf(fileID,'\n');
        counter=counter+0.05;
    end
end

clear all