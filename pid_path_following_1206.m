clc
clear all
myev3=legoev3('usb');
ix=0;
kp=36.148;
ki=0;
kd=30;                                        
inputA=50;
inputB=50;
ine=[0 0];
dee=[0 0];
lasterror=[0 0];

mymotor1=motor(myev3,'A');
start(mymotor1);
mymotor1.Speed=inputA;

mymotor2=motor(myev3,'D');
start(mymotor2);
mymotor2.Speed=inputB;

ls(1)=readInputDeviceREADY_RAW(myev3,1,0,1)/34;
ls(3)=readInputDeviceREADY_RAW(myev3,3,0,1)/34;

diff=ls(1)-ls(3); %initial difference between sensors 1 and 3

while(true)
    t1= (clock);
   % path follow using three light sensor
   
   % obtain raw data
    ls(1)=readInputDeviceREADY_RAW(myev3,1,0,1);
    ls(2)=readInputDeviceREADY_RAW(myev3,2,0,1);
    ls(3)=readInputDeviceREADY_RAW(myev3,3,0,1);
    ls(4)=readInputDeviceREADY_RAW(myev3,4,0,1);
    ls(4)=(ls(4)-104.6627)/3.7776;
    ls(1:3)=ls(1:3)/35;
    
    if ls(4)< 20
        mymotor1.Speed=0;
        mymotor2.Speed=0;
        break;
    end 
    
 
    error=(ls(1)-ls(3));   % calculate error
    %ine(1)=ine(1)+error(1);                % integral error
    dee(1)=error-lasterror(1);           % derivative error   
    turn1=kp*error+ki*ine(1)+kd*dee(1);  % PID error
    %ine(2)=ine(2)+error(2);                % integral error
    dee(2)=(-error)-lasterror(2);           % derivative error   
    turn2=kp*(-error)+ki*ine(2)+kd*dee(2);  % PID error
    error;
    turn1;
    turn2;
    powerA=inputA+turn2;                % new input    
    powerB=inputB+turn1;
    powerA;
    powerB;
    if powerA>70               % motor threshold from 0-100 
        powerA=70;
    elseif powerA<20
        powerA=20;
    end
    
    if powerB>70
        powerB=70;
    elseif powerB<20
        powerB=20;
    end
    mymotor1.Speed=powerA;
    mymotor2.Speed=powerB;
    
    lasterror(:)=error;
    diff=0;
    
    t2=(clock);
    t3=t2-t1;
end

clear all

