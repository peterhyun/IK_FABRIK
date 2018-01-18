clc
clear
close all

% Step1: Set Desired position and orientation
Rotd = [cos(pi/4) -sin(pi/4); sin(pi/4) cos(pi/4)];
xd1 = [0.8, 0.6];
xd2 = [0.7, 0.2];
s = 0.1;

% Step2: Init Robot Model
l= 0.3;
dof = 7;

j1.coord = [0, 0];
j1.parent = 0;
j1.rotation = [1 0; 0 1];
nodes(1) = j1;

for i=2:7
    nodes(i).parent = i-1;
    nodes(i).rotation = j1.rotation;
    nodes(i).r=0.0;
    nodes(i).lambda=0.0;
    nodes(i).coord = [l*(i-1),0];
end

% Step3: Plot Robot Model
figure(1)
hold on
grid on
plot(xd1(1), xd1(2), '*r');
plot(xd2(1), xd2(2), '*r');
line([xd1(1), xd1(1)+Rotd(1,1)*s], [xd1(2), xd1(2)+Rotd(2,1)*s], 'Color', 'r', 'LineWidth', 1.5);
line([xd1(1), xd1(1)+Rotd(1,2)*s], [xd1(2), xd1(2)+Rotd(2,2)*s], 'Color', 'g', 'LineWidth', 1.5);
line([xd2(1), xd2(1)+Rotd(1,1)*s], [xd2(2), xd2(2)+Rotd(2,1)*s], 'Color', 'r', 'LineWidth', 1.5);
line([xd2(1), xd2(1)+Rotd(1,2)*s], [xd2(2), xd2(2)+Rotd(2,2)*s], 'Color', 'g', 'LineWidth', 1.5);
line([nodes(1).coord(1), nodes(2).coord(1)], [nodes(1).coord(2), nodes(2).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
line([nodes(2).coord(1), nodes(3).coord(1)], [nodes(2).coord(2), nodes(3).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
for i=2:3
    line([nodes(2*i-1).coord(1), nodes(2*i+1).coord(1)], [nodes(2*i-1).coord(2), nodes(2*i+1).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
    line([nodes(2*i-2).coord(1), nodes(2*i).coord(1)], [nodes(2*i-2).coord(2), nodes(2*i).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
end
for i=1:7
    plot(nodes(i).coord(1), nodes(i).coord(2), 'ob');
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,1)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,1)*s], 'Color', 'r', 'LineWidth', 0.5);
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,2)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,2)*s], 'Color', 'g', 'LineWidth', 0.5);
end
axis([-0.1 1.5 -0.1 1.5]);

% Step4: Fabrik IK Solver
tol = 1e-5;
iter = 0;
dist1 = norm(xd1 - nodes(1).coord);
dist2 = norm(xd2 - nodes(1).coord);
%2 arms
if (dist1 > 4*l || dist2 > 3*l)
    disp('unreachable');
else
    difA = norm(xd1-nodes(7).coord);
    difB = norm(xd2-nodes(6).coord);
    iterationCount=0;
    while(difA>tol || difB>tol)
        %first sub-arm
        subbase_origin = nodes(2).coord;
        nodes(7).coord = xd1;
        for i=3:-1:2
            nodes(2*i-1).r = norm(nodes(2*i+1).coord - nodes(2*i-1).coord);
            nodes(2*i-1).lambda = l/nodes(2*i-1).r;
            nodes(2*i-1).coord = (1-nodes(2*i-1).lambda)*nodes(2*i+1).coord + nodes(2*i-1).lambda*nodes(2*i-1).coord;
        end %forward reaching
        nodes(2).r = norm(nodes(3).coord-nodes(2).coord);
        nodes(2).lambda = l/nodes(2).r;
        nodes(2).coord = (1-nodes(2).lambda)*nodes(3).coord + nodes(2).lambda*nodes(2).coord;
        subbase_cand1 = nodes(2).coord; % 일단 첫번째 sub-base 위치 저장
        
        %second sub-arm
        nodes(2).coord = subbase_origin; %다시 원상복귀
        nodes(6).coord = xd2;
        for i=2:-1:1
            nodes(2*i).r = norm(nodes(2*(i+1)).coord - nodes(2*i).coord);
            nodes(2*i).lambda = l/nodes(2*i).r;
            nodes(2*i).coord = (1-nodes(2*i).lambda)*nodes(2*(i+1)).coord + nodes(2*i).lambda*nodes(2*i).coord;
        end %forward reaching
        subbase_cand2 = nodes(2).coord;
        
        centroid = (subbase_cand1+subbase_cand2)/2;       
        nodes(2).coord = centroid;
        
        %subbase to root
        nodes(1).r = norm(nodes(2).coord - nodes(1).coord);
        nodes(1).lambda = l/nodes(1).r;
        nodes(1).coord = (1-nodes(1).lambda)*nodes(2).coord + nodes(1).lambda*nodes(1).coord;
        %forward reach 끝!
        
        %다시 base부터 sub-base까지 backward        
        nodes(1).coord = [0,0];
        nodes(1).r = norm(nodes(2).coord - nodes(1).coord);
        nodes(1).lambda = l/nodes(1).r;
        nodes(2).coord = (1-nodes(1).lambda)*nodes(1).coord + nodes(1).lambda*nodes(2).coord;
        
        %각 chain마다 backward
        for i=2:4
            if(i==2)
                nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                nodes(i).lambda = l/nodes(i).r;
                nodes(i+1).coord = (1-nodes(i).lambda)*nodes(i).coord + nodes(i).lambda*nodes(i+1).coord;
            else
                nodes(2*i-3).r = norm(nodes(2*i-1).coord - nodes(2*i-3).coord);
                nodes(2*i-3).lambda = l/nodes(2*i-3).r;
                nodes(2*i-1).coord = (1-nodes(2*i-3).lambda)*nodes(2*i-3).coord + nodes(2*i-3).lambda*nodes(2*i-1).coord;
            end
        end
        
        for i=1:2
            nodes(2*i).r = norm(nodes(2*(i+1)).coord-nodes(2*i).coord);
            nodes(2*i).lambda = l/nodes(2*i).r;
            nodes(2*(i+1)).coord = (1-nodes(2*i).lambda)*nodes(2*i).coord + nodes(2*i).lambda*nodes(2*(i+1)).coord;
        end
        difA = norm(xd1-nodes(7).coord);
        difB = norm(xd2-nodes(6).coord);
        iterationCount = iterationCount+1;
    end
end

% Step5: Result plotting
for i=1:6
    r1=nodes(i+1).coord-nodes(i).coord;
    r1 = r1 / norm(r1);
    nodes(i).rotation(1,1) = r1(1);
    nodes(i).rotation(2,1) = r1(2);
    nodes(i).rotation(1,2) = -r1(2);
    nodes(i).rotation(2,2) = r1(1);
end
nodes(7).rotation = nodes(6).rotation;
for i=1:4
    if(i<3)
        line([nodes(i).coord(1), nodes(i+1).coord(1)], [nodes(i).coord(2), nodes(i+1).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
    else
        line([nodes(2*i-3).coord(1), nodes(2*i-1).coord(1)], [nodes(2*i-3).coord(2), nodes(2*i-1).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
        line([nodes(2*(i-2)).coord(1), nodes(2*(i-1)).coord(1)], [nodes(2*(i-2)).coord(2), nodes(2*(i-1)).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
    end
end
for i=1:7
    plot(nodes(i).coord(1), nodes(i).coord(2), 'ob');
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,1)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,1)*s], 'Color', 'r', 'LineWidth', 0.5);
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,2)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,2)*s], 'Color', 'g', 'LineWidth', 0.5);
end