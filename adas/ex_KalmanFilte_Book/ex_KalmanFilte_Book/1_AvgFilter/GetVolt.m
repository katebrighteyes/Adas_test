function z = GetVolt()

w = 0 + 4*randn(1,1);   % 잡음을 생성하기 위한 randn() 함수
z = 14.4 + w;           % 전압에 잡음이 추가된다.
