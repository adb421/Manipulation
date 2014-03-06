clear classes

params = ParametersFunction();

pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();

home = [-0.2 -0.1 0];

pc104.sendHomePos(home);

disp('unpause to go home')
pause 
pc104.goHome();

while(1)
    pc104.misc();
    pause(1);
end
