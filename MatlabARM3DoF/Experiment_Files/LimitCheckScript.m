% This script can be used to check the motor encoder values at any point. I originally used it to test limits
pc104 = PC104_Arm3DoF();
pc104.connect();
disp('Make sure camera is working');
pc104.resetEncoders();
while(1)
    disp('unpause to read encoder positions')
    pause;
    pc104.currentEncoderPos();
end