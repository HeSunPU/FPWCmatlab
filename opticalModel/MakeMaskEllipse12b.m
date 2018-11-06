function out = MakeMaskEllipse12b(N, folder)

xs = 0.5+[-N:N-1];
ys = xs';

load([folder.optics '/SPs/Ripple3'],'-ascii');
out = zeros(2*N,2*N);
sgn = -1;
for k=1:8
    A = Ripple3(1+(k-1)*80:k*80,:);
    [m, n] = size(A);
    A=[(A(m:-1:1,:)*[-1 0; 0 1])' A']';

    yofx = spline(2*N*A(:,1),2*N*A(:,2),xs);
    for j=1:2*N
	m1 = (ys < yofx(j)-0.5) .* (ys > -yofx(j)+0.5);
	m2 = (ys < yofx(j)+0.5) .* (ys > -yofx(j)-0.5);
	m21 = m2-m1;
        out(:,j) = out(:,j) + sgn*(m21*(yofx(j)-floor(yofx(j))) + m1);
    end
    sgn = -sgn;
end
