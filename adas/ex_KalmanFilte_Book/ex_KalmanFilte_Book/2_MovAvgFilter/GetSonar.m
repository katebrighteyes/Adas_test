function res = GetSonar()

persistent sonarAlt k firstRun

if isempty(firstRun)
    load SonarAlt
    k = 1;
    firstRun = 1;
end

res = sonarAlt(k);

k = k + 1;