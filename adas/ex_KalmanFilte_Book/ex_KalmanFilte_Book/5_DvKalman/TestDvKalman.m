function TestDvKalman

clear all

dt = 0.1;
t  = 0:dt:10;

Nsamples = length(t);

Xsaved = zeros(Nsamples, 2);
Zsaved = zeros(Nsamples, 1);
realVel = zeros(Nsamples,1);

for k=1:Nsamples
  [z,y] = GetPos();      
  [pos vel] = DvKalman(z);
  
  Xsaved(k, :) = [pos vel];
  Zsaved(k)    = z;
  realVel(k) = y;
end

figure(1); hold on;
plot(t, Zsaved(:), 'r.');
plot(t, Xsaved(:, 1), 'b');
legend('Measurements', 'Kalman Filter');
title('위치 측정/예측 그래프');

figure(2); hold on;
plot(t, realVel(:), 'b-.');
plot(t, Xsaved(:, 2), 'r');
legend('Real speed', 'Kalman Filter');
title('속도 측정/예측 그래프');
end
