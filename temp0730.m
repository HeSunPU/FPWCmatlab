while true
    [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, 1);
    figure(101), imagesc(A-dark), colorbar
    drawnow
end

H_stage = 22.845;

V_stage = 40.58