%Integrate accelerometer data and see what we get


accelerometerX = VarName1;
accelerometerY = VarName2;
%Get rid of constant offsets
acclerometerX = accelerometerX - mean(accelerometerX);
acclerometerY = accelerometerY - mean(accelerometerY);
%Convert to m/s^2, and definitely invert x, maybe y as well
accelerometerX = -accelerometerX;
accelerometerX = accelerometerX*9.81/0.15;
accelerometerY = accelerometerY*9.81/0.15;

dt = 0.001;

integVelX = zeros(size(accelerometerX));
integVelY = zeros(size(accelerometerY));
integPosX = zeros(size(integVelX));
integPosY = zeros(size(integVelY));
integPosX(1) = xActual(1);
integPosY(1) = yActual(1);
integVelX(1) = (xActual(2) - xActual(1))/dt;
integVelY(1) = (yActual(2) - yActual(1))/dt;
for i = 2:length(accelerometerX)
    integVelX(i) = integVelX(i-1) + accelerometerX(i-1)*dt;
    integVelY(i) = integVelY(i-1) + accelerometerY(i-1)*dt;
    integPosX(i) = integPosX(i-1) + integVelX(i-1)*dt + ...
        0.5*accelerometerX(i-1)*dt^2;
     integPosY(i) = integPosY(i-1) + integVelY(i-1)*dt + ...
        0.5*accelerometerY(i-1)*dt^2;
end
figure(1)
plot(xActual);
hold on
plot(integPosX,'-r');
figure(2)
plot(yActual);
hold on
plot(integPosY,'-r');