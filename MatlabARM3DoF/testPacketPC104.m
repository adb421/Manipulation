% Test the new packetizing stuff

%Create our pc104 object
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.misc();
pc104.killProgram();