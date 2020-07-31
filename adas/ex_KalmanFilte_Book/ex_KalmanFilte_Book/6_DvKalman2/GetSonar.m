function h = GetSonar()

persistent sonarAlt
persistent k firstRun

if isempty(firstRun)
    load SonarAlt
    k = 1;
    
    firstRun = 1;
end

h = sonarAlt(k);

k = k + 1;
end
%% 여기서는 미리 저장해 놓은 측정 데이터를 하나씩 반환하도록 하는 코드
%% matlab sonarAlt.mat 파일에 데이터가 저장되어있음. (line 7)