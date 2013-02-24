%Script just to test out gains
kp = single([3 2 1]);
kd = single([3 2 1]);
ki = single([3 2 1]);
[pos1 pos2 pos3 traj1 traj2 traj3] = plotTrajectory(kp,kd,ki);