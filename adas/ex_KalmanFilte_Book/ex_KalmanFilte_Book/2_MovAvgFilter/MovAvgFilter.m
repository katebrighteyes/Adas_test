function avg = MovAvgFilter(x)

persistent prevAvg n xbuf firstRun;

if isempty(firstRun)
    prevAvg = x;
    firstRun = 1;
    n =10;
    xbuf = x * ones(n+1, 1);
end

for m = 1:n
    xbuf(m) = xbuf(m+1);
end
xbuf(n+1) = x;

avg = prevAvg + (x - xbuf(1)) / n;

prevAvg = avg;
end