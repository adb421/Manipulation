% Adam Barber
% 3/15/2012
% Euler integrator in case you don't want to use ode45 or whatever

function [times states] = eulerIntegrator(derivFxn, timeSpan, dt, initialConditions)

    %Generate times
    times = timeSpan(1):dt:timeSpan(2);
    %Preallocate states
    states = zeros(length(initialConditions),length(times));
    %Set initial conditions
    states(:,1) = initialConditions;
    %Loop and integrate
    for i = 2:length(times)
        states(:,i) = states(:,i-1) + dt*derivFxn(times(i-1), states(:,i-1));
    end

end