pc104 = PC104_Arm3DoF();
pc104.connect();
disp('Make sure camera is working');
pc104.resetEncoders();
while(1)
    %Unpause to read encoder positions
    pause;
    pc104.currentEncoderPos();
end