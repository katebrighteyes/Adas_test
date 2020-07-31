function volt = SimpleKalman(z)
%
%
persistent A H Q R 
persistent x P
persistent firstRun


if isempty(firstRun)
    A = 1;  % 시스템 모델 변수 A = 1 ( x_(k+1) = x_k 이므로)
    H = 1;  % 시스템 모델 변수 H = 1 ( z_k = x_k + v_k 이므로)
    Q = 0;  % 시스템 모델 변수 Q = 0 ( w_k = 0 이므로)
    R = 4;  % 시스템 모델 변수 R = 4 ( v ~ N(0, 2)인 정규분포이므로)
    x = 14; % 초기 예측 전압 14
    P = 6;  % 초기 예측 오차 공분산 6
  
  firstRun = 1;  
end

  
xp = A*x;                       %추정값 예측
Pp = A*P*A' + Q;                %오차 공분산 예측

K = Pp*H'*inv(H*Pp*H' + R);     %칼만 이득 계산

x = xp + K*(z - H*xp);          %추정값 계산
P = Pp - K*H*Pp;                %오차 공분산 계산


volt = x;                       %계산된 추정값 반환
end