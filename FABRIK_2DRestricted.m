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
line([xd(1), xd(1)+Rotd(1,1)*s], [xd(2), xd(2)+Rotd(2,1)*s], 'Color', 'r', 'LineWidth', 1.5);   %아마 두 좌표값이 동일하면 그냥 45도 각도로 선 긋는 듯?
line([xd(1), xd(1)+Rotd(1,2)*s], [xd(2), xd(2)+Rotd(2,2)*s], 'Color', 'g', 'LineWidth', 1.5);
for i=1:7
    line([nodes(i).coord(1), nodes(i+1).coord(1)], [nodes(i).coord(2), nodes(i+1).coord(2)], 'Color', 'k', 'LineWidth', 1.0);
end
for i=1:8
    plot(nodes(i).coord(1), nodes(i).coord(2), 'ob'); %원래 상태의 노드들을 파란색 원으로 표식!!!
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
                %30도 각도 측정을 위한 장치임
                vec1 = nodes(i+2).coord - nodes(i+1).coord;
                vec2 = nodes(i).coord - nodes(i+1).coord;
                tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));                
                if(int32(rad2deg(tempAngle))<150)  %150도보다 작으면
%                     disp('150도보다 작다');
                    moveAngle = (5/6)*pi-tempAngle;
                    %either clockwise or counter clockwise
                    rotateM = [cos(moveAngle) -sin(moveAngle);sin(moveAngle) cos(moveAngle)]; %일단 clockwise rotation
                    coordTemp = (rotateM * (nodes(i).coord-nodes(i+1).coord)')'+nodes(i+1).coord;
                    vec2 = coordTemp - nodes(i+1).coord;
                    tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));
                    if(int32(rad2deg(tempAngle))<150) %그래도 아직 그 범위에 못들었다면 잘못 회전시킨 것. 반대로 회전시키면 됨!
%                         disp('잘못 변환했다');
                        fprintf('(5/6)*pi는 %f. 전치행렬로 돌리기 전 각도 : %f\n',(5/6)*pi,tempAngle); %????질문하기... 왜 똑같은 경우도여기에 있지...??소숫점 오류 해결법
                        rotateM = rotateM';
                        nodes(i).coord = (rotateM * (nodes(i).coord-nodes(i+1).coord)')'+nodes(i+1).coord;
                        
                        vec2 = nodes(i).coord - nodes(i+1).coord;
                        tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));
%                         if(int32(rad2deg(tempAngle))<150)
%                             fprintf('%dth iteration and the nodes are %d, %d, %d. Angle is %f\n',iter,i,i+1,i+2,rad2deg(tempAngle));
%                         end
                    else
                        nodes(i).coord = coordTemp;
%                         disp('잘 변환했다');
                    end
                    nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                    nodes(i).lambda = l/nodes(i).r;
                    nodes(i).coord = (1-nodes(i).lambda)*nodes(i+1).coord + nodes(i).lambda*nodes(i).coord;
                else
%                   disp('괜찮다');
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
%                   disp('또 150도보다 작다');
                    moveAngle = (5/6)*pi-tempAngle;
                    %either clockwise or counter clockwise
                    rotateM = [cos(moveAngle) -sin(moveAngle);sin(moveAngle) cos(moveAngle)]; %일단 counter clockwise rotation
                    coordTemp = (rotateM * (nodes(i+1).coord-nodes(i).coord)')'+nodes(i).coord;
                    vec2 = coordTemp - nodes(i).coord;
                    tempAngle = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)));
                    if(int32(rad2deg(tempAngle))<150)
%                         disp('또 잘못 변환했다');
                        rotateM = rotateM';
                        nodes(i+1).coord = (rotateM * (nodes(i+1).coord-nodes(i).coord)')'+nodes(i).coord;
                    else
%                         disp('또 잘 변환했다.');
                        nodes(i+1).coord = coordTemp;
                    end
                    nodes(i).r = norm(nodes(i+1).coord - nodes(i).coord);
                    nodes(i).lambda = l/nodes(i).r;
                    nodes(i+1).coord = (1-nodes(i).lambda)*nodes(i).coord + nodes(i).lambda*nodes(i+1).coord;
                else
%                     disp('또 괜찮다');
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
    %orientation 수정 중
    r1=nodes(i+1).coord-nodes(i).coord;
    r1 = r1 / norm(r1);
    nodes(i).rotation(1,1) = r1(1);
    nodes(i).rotation(2,1) = r1(2);
    nodes(i).rotation(1,2) = -r1(2);
    nodes(i).rotation(2,2) = r1(1);
    %수직이 되도록
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
%line function : x 좌표들끼리 모으고 ', ' 찍고 y 좌표들끼리 모은다.