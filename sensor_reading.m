clear all;
%myev3=legoev3('wifi','172.20.10.2','001653481485');
myev3=legoev3('usb');

while(true)
   
   % obtain raw data
    ls(1)=readInputDeviceREADY_RAW(myev3,1,0,1);
    ls(2)=readInputDeviceREADY_RAW(myev3,2,0,1);
    ls(3)=readInputDeviceREADY_RAW(myev3,3,0,1);
    display(ls);
    
    
    
   
end


