function [z, y] = GetPos()

persistent Velp Posp

if isempty(Posp)
    Posp = 0;   % 위치 초기 예측값, 이후에도 사용
    Velp = 80;  % 속도 초기 예측값
end

dt = 0.1;       % 측정 시간간격 0.1초

w = 0 + 10*randn;   % 0~10까지 랜덤한 실수 (시스템 잡음)
v = 0 + 10*randn;   % 0~10까지 랜덤한 실수 (측정 잡음)

z = Posp + Velp*dt+v;
% 현재위치 = 이전위치 + 현재속도(v)*시간간격(t) + 측정오차
y = Velp;
Posp = z-v;
Velp = 80+w;
