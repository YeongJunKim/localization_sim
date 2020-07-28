function r = dynamics(x,dt)
    
    A = [1 0 dt 0;
        0 1 0 dt;
        0 0 1 0;
        0 0 0 1];
    
    B = [(dt^2)/2 0;
        dt 0;
        0 (dt^2)/2;
        0 dt];
    
    w = ones(2,1) * 0.01;
    
    r = A * x + B * w;
end