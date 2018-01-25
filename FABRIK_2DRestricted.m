clc
clear all
close all
%restriction of angle : -30~30

% Step1: Set Desired position and orientation
Rotd = [cos(pi/4) -sin(pi/4); sin(pi/4) cos(pi/4)];
MAX_iter = 100000;
xd = [1,1];
s = 0.1;

% Step2: Init Robot Model
l= 0.3;
dof = 8;

j1.coord=[0 0];
j1.parent = 0;
j1.rotation=[1 0; 0 1];
nodes(1) = j1;

for i=2:8
    nodes(i).coord = [l*(i-1) 0];
    nodes(i).parent = i-1;
    nodes(i).rotation = j1.rotation;
    nodes(i).r=0.0;
    nodes(i).lambda=0.0;
end

% Step3: Plot Robot Model
figure(1)
hold on
grid on
plot(xd(1), xd(2), '*r');
line([xd(1), xd(1)+Rotd(1,1)*s], [xd(2), xd(2)+Rotd(2,1)*s], 'Color', 'r', 'LineWidth', 1.5);   %�Ƹ� �� ��ǥ���� �����ϸ� �׳� 45�� ������ �� �ߴ� ��?
line([xd(1), xd(1)+Rotd(1,2)*s], [xd(2), xd(2)+Rotd(2,2)*s], 'Color', 'g', 'LineWidth', 1.5);
for i=1:7
    line([nodes(i).coord(1), nodes(i+1).coord(1)], [nodes(i).coord(2), nodes(i+1).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
end
for i=1:8
    plot(nodes(i).coord(1), nodes(i).coord(2), 'ob'); %���� ������ ������ �Ķ��� ������ ǥ��!!!
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,1)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,1)*s], 'Color', 'r', 'LineWidth', 0.5);
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,2)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,2)*s], 'Color', 'g', 'LineWidth', 0.5);
end
axis([-1 2.0 -1 2.0]);

% Step4: Fabrik IK Solver
tol = 1e-5;
iter = 0;
dist = norm(xd - nodes(1).coord);
if (dist > l*dof)
    disp('unreachable');
else
    b = nodes(1).coord;
    difa = norm(xd - nodes(8).coord);
    while (difa > tol && iter<MAX_iter)
        nodes(8).coord = xd;
        for i=7:-1:1
            if(i==7)
                nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                nodes(i).lambda = l/nodes(i).r;
                nodes(i).coord = (1-nodes(i).lambda)*nodes(i+1).coord + nodes(i).lambda*nodes(i).coord;
            else
                %30�� ���� ������ ���� ��ġ��
                vec1 = nodes(i+2).coord - nodes(i+1).coord;
                vec2 = nodes(i).coord - nodes(i+1).coord;
                tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));                
                if(int32(rad2deg(tempAngle))<150)  %150������ ������
%                     disp('150������ �۴�');
                    moveAngle = (5/6)*pi-tempAngle;
                    %either clockwise or counter clockwise
                    rotateM = [cos(moveAngle) -sin(moveAngle);sin(moveAngle) cos(moveAngle)]; %�ϴ� clockwise rotation
                    coordTemp = (rotateM * (nodes(i).coord-nodes(i+1).coord)')'+nodes(i+1).coord;
                    vec2 = coordTemp - nodes(i+1).coord;
                    tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));
                    if(int32(rad2deg(tempAngle))<150) %�׷��� ���� �� ������ ������ٸ� �߸� ȸ����Ų ��. �ݴ�� ȸ����Ű�� ��!
%                         disp('�߸� ��ȯ�ߴ�');
                        fprintf('(5/6)*pi�� %f. ��ġ��ķ� ������ �� ���� : %f\n',(5/6)*pi,tempAngle); %????�����ϱ�... �� �Ȱ��� ��쵵���⿡ ����...??�Ҽ��� ���� �ذ��
                        rotateM = rotateM';
                        nodes(i).coord = (rotateM * (nodes(i).coord-nodes(i+1).coord)')'+nodes(i+1).coord;
                        
                        vec2 = nodes(i).coord - nodes(i+1).coord;
                        tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));
%                         if(int32(rad2deg(tempAngle))<150)
%                             fprintf('%dth iteration and the nodes are %d, %d, %d. Angle is %f\n',iter,i,i+1,i+2,rad2deg(tempAngle));
%                         end
                    else
                        nodes(i).coord = coordTemp;
%                         disp('�� ��ȯ�ߴ�');
                    end
                    nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                    nodes(i).lambda = l/nodes(i).r;
                    nodes(i).coord = (1-nodes(i).lambda)*nodes(i+1).coord + nodes(i).lambda*nodes(i).coord;
                else
%                   disp('������');
                    nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                    nodes(i).lambda = l/nodes(i).r;
                    nodes(i).coord = (1-nodes(i).lambda)*nodes(i+1).coord + nodes(i).lambda*nodes(i).coord;
                end
            end
            
        end
        
        %Backward Reaching
        nodes(1).coord = b;
        for i=1:7
            if(i==1)
                nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                nodes(i).lambda = l/nodes(i).r;
                nodes(i+1).coord = (1-nodes(i).lambda)*nodes(i).coord + nodes(i).lambda*nodes(i+1).coord;
            else
                vec1 = nodes(i-1).coord - nodes(i).coord;
                vec2 = nodes(i+1).coord - nodes(i).coord;
                tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));
                if(int32(rad2deg(tempAngle))<150)
%                   disp('�� 150������ �۴�');
                    moveAngle = (5/6)*pi-tempAngle;
                    %either clockwise or counter clockwise
                    rotateM = [cos(moveAngle) -sin(moveAngle);sin(moveAngle) cos(moveAngle)]; %�ϴ� counter clockwise rotation
                    coordTemp = (rotateM * (nodes(i+1).coord-nodes(i).coord)')'+nodes(i).coord;
                    vec2 = coordTemp - nodes(i).coord;
                    tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));
                    if(int32(rad2deg(tempAngle))<150)
%                         disp('�� �߸� ��ȯ�ߴ�');
                        rotateM = rotateM';
                        nodes(i+1).coord = (rotateM * (nodes(i+1).coord-nodes(i).coord)')'+nodes(i).coord;
                    else
%                         disp('�� �� ��ȯ�ߴ�.');
                        nodes(i+1).coord = coordTemp;
                    end
                    nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                    nodes(i).lambda = l/nodes(i).r;
                    nodes(i+1).coord = (1-nodes(i).lambda)*nodes(i).coord + nodes(i).lambda*nodes(i+1).coord;
                else
%                     disp('�� ������');
                    nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                    nodes(i).lambda = l/nodes(i).r;
                    nodes(i+1).coord = (1-nodes(i).lambda)*nodes(i).coord + nodes(i).lambda*nodes(i+1).coord;
                end %backward reaching
            end
        end
        difa = norm(xd-nodes(8).coord);
        iter = iter+1;
    end
end

%Step5: Result plotting
for i=1:7
    %orientation ���� ��
    r1=nodes(i+1).coord-nodes(i).coord;
    r1 = r1 / norm(r1);
    nodes(i).rotation(1,1) = r1(1);
    nodes(i).rotation(2,1) = r1(2);
    nodes(i).rotation(1,2) = -r1(2);
    nodes(i).rotation(2,2) = r1(1);
    %������ �ǵ���
end
nodes(8).rotation = nodes(7).rotation;

for i=1:7
    line([nodes(i).coord(1), nodes(i+1).coord(1)], [nodes(i).coord(2), nodes(i+1).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
end
for i=1:8
    plot(nodes(i).coord(1), nodes(i).coord(2), 'ob');
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,1)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,1)*s], 'Color', 'r', 'LineWidth', 0.5);
    line([nodes(i).coord(1), nodes(i).coord(1)+nodes(i).rotation(1,2)*s], [nodes(i).coord(2), nodes(i).coord(2)+nodes(i).rotation(2,2)*s], 'Color', 'g', 'LineWidth', 0.5);
end

iter = iter
vec1 = nodes(3).coord - nodes(4).coord;
vec2 = nodes(5).coord - nodes(4).coord;
seta = rad2deg(acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2))))
%line function : x ��ǥ�鳢�� ������ ', ' ��� y ��ǥ�鳢�� ������.