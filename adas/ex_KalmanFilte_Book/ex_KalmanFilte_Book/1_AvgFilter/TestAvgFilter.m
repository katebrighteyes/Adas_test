function TestAvgFilter

dt = 0.2;
t  = 0:dt:10;           % 0.2초 간격 샘플링

Nsamples = length(t);   % 샘플링된 개수

Avgsaved = zeros(Nsamples, 1); % 0으로 초기화된 Nsamples칸 1차원 벡터
Xmsaved  = zeros(Nsamples, 1); % 0으로 초기화된 Nsamples칸 1차원 벡터 

for k=1:Nsamples
  xm  = GetVolt();
  avg = AvgFilter(xm);
  
  Avgsaved(k) = avg;
  Xmsaved(k)  = xm;
end


figure
plot(t, Xmsaved, 'r:*')
hold on
plot(t, Avgsaved, 'o-')
legend('Measured', 'Average');
end