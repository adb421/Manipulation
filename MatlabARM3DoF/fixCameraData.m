j = 2;
tcam = zeros(size(pc104.t));
fixedCamX = zeros(size(pc104.camPosX));
fixedCamY = zeros(size(pc104.camPosY));
fixedObjX = zeros(size(pc104.objPosX));
fixedObjY = zeros(size(pc104.objPosY));
fixedObjTh = zeros(size(pc104.objPosTh));
indices = zeros(size(pc104.t));
tcam(1) = pc104.t(1);
fixedCamX(1) = pc104.camPosX(1);
fixedCamY(1) = pc104.camPosY(1);
fixedObjX(1) = pc104.objPosX(1);
fixedObjY(1) = pc104.objPosY(1);
fixedObjTh(1) = pc104.objPosTh(1);
indices(1) = 1;
for i = 2:length(pc104.camPosX)
    if(pc104.camPosX(i) ~= pc104.camPosX(i-1))
        fixedCamX(j) = pc104.camPosX(i);
        fixedCamY(j) = pc104.camPosY(i);
        tcam(j) = pc104.t(i);
        fixedObjX(j) = pc104.objPosX(i);
        fixedObjY(j) = pc104.objPosY(i);
        fixedObjTh(j) = pc104.objPosTh(i);
        indices(j) = i;
        j = j+1;
    end
end
fixedCamX = fixedCamX(1:j-1);
fixedCamY = fixedCamY(1:j-1);
tcam = tcam(1:j-1);
fixedObjX = fixedObjX(1:j-1);
fixedObjY = fixedObjY(1:j-1);
fixedObjTh = fixedObjTh(1:j-1);
indices = indices(1:j-1);
alpha_cam = 0.112;
dt = pc104.dt;
camXVel = derivative(fixedCamX)/dt;
camYVel = derivative(fixedCamY)/dt;
objXVel = derivative(fixedObjX)./derivative(tcam);
objYVel = derivative(fixedObjY)./derivative(tcam);
objThVel = derivative(fixedObjTh)./derivative(tcam);
camXAccel = derivative(camXVel)./derivative(tcam);
camYAccel = derivative(camYVel)./derivative(tcam);
objXAccel = derivative(objXVel)./derivative(tcam);
objYAccel = derivative(objYVel)./derivative(tcam);
objThAccel = derivative(objThVel)./derivative(tcam);
camXVel_filt = camXVel*alpha_cam;
camYVel_filt = camYVel*alpha_cam;
objXVel_filt = objXVel*alpha_cam;
objYVel_filt = objYVel*alpha_cam;
objThVel_filt = objThVel*alpha_cam;
for i = 2:length(tcam)
    camXVel_filt(i) = (1-alpha_cam)*camXVel_filt(i-1) ...
        + alpha_cam*camXVel(i);
    camYVel_filt(i) = (1-alpha_cam)*camYVel_filt(i-1) ...
        + alpha_cam*camYVel(i);
    objXVel_filt(i) = (1-alpha_cam)*objXVel_filt(i-1) ...
        + alpha_cam*objXVel(i);
    objYVel_filt(i) = (1-alpha_cam)*objYVel_filt(i-1) ...
        + alpha_cam*objYVel(i);
    objThVel_filt(i) = (1-alpha_cam)*objThVel_filt(i-1) ...
        + alpha_cam*objThVel(i);
end