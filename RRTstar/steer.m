function vnew = steer(qnear,qrand)

% ode parameters
q0 = qnear.pose;
qf = qrand;
N = 100;
[nr nc] = size(q0);
for i=1:nr
    qnew_1 = linspace(q0(i), qf(i), N);
    Qnew(:,i) = qnew_1;
end


velnew = 1;
omeganew = 1;


vnew.id = -1;  % to indicate that it will be set later
vnew.pose = Qnew(end,:)';
vnew.edgeq = Qnew;
vnew.edgeu = [velnew omeganew];
vnew.pid = qnear.id;


