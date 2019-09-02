%plot mask
coef3 = 22;
mask.rangeR = [5, 11];
mask.rangeAngle = 35;%42.5;
lw=.5;
n=2;
rs = linspace(mask.rangeR(1),mask.rangeR(2),n)*pixperlamD/coef3;
line(linspace(0,0,2), linspace(-RY/2/coef2 , RY/coef2,2), 'color', 'c', 'linewidth', lw)
line(linspace(-RX*.55/coef , RX*0.55/coef,2), linspace(0,0,2), 'color', 'c', 'linewidth', lw)
line(0 + cosd(mask.rangeAngle)*rs, 0 + sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
line(0 - cosd(mask.rangeAngle)*rs, 0 + sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
line(0 - cosd(mask.rangeAngle)*rs, 0 - sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
line(0 + cosd(mask.rangeAngle)*rs, 0 - sind(mask.rangeAngle)*rs, 'color', 'c', 'linewidth', lw)
n=100;
circd = @(radius,deg_ang)  [radius*cosd(deg_ang);  radius*sind(deg_ang)];
arc1 = circd(mask.rangeR(1),linspace(mask.rangeAngle, 180-mask.rangeAngle, n))*pixperlamD/coef3; 
arc2 = circd(mask.rangeR(2),linspace(mask.rangeAngle, 180-mask.rangeAngle, n))*pixperlamD/coef3;
line(0 + arc1(1,:), 0 + arc1(2,:), 'color', 'c', 'linewidth', lw)
line(0 + arc2(1,:), 0 + arc2(2,:), 'color', 'c', 'linewidth', lw)
line(0 + arc1(1,:), 0 - arc1(2,:), 'color', 'c', 'linewidth', lw)
line(0 + arc2(1,:), 0 - arc2(2,:), 'color', 'c', 'linewidth', lw)
%axis([0  2*RX2    0  2*RY])%axis equal


