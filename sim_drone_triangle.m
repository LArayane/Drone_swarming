% G={}
% for i=1:3
%  for j=1:10
% G=[G {double([Gall{:,i}'+ Gall{:,i}- Pall{:,j}   , zeros(5) , Gall{:,i}*Aa{:,i}-Kall{:,i}*Cc , Gall{:,i}; ...
%     zeros(5)                           , ones(5)  , ones(5)                        , zeros(5) ; ...
%     Aa{:,i}'*Gall{:,i}'-Cc'*Kall{:,i}' , ones(5)  , Pall{:,i}                      , zeros(5) ; ...
%     Gall{:,i}'                         , zeros(5) , zeros(5)                       , gamma*eye(5)])}];
%  end 
% end

% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** MEAM 620 QUADROTOR SIMULATION *****************
time = var(:,1);
% number of quadrotors
nquad = 3;

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = length(time);      % max iteration
% starttime = 0;         % start of simulation in seconds
% if OUTPUT_TO_VIDEO == 1
%     close(v);
% end

fprintf('finished.\n')
x1 = var(:,2); y1 = var(:,3); z1 = var(:,4); 
x2 = var(:,29); y2 = var(:,30); z2 = var(:,31); 
x3 = var(:,56); y3 = var(:,57); z3 = var(:,58); 
xf = var(:,125); yf = var(:,126); zf = var(:,127);
xfr = var(:,116); yfr = var(:,117); zfr = var(:,118);

X1 = [x1(1), x2(1), x3(1) x1(1)];
Y1 = [y1(1), y2(1), y3(1) y1(1)];
Z1 = [z1(1), z2(1), z3(1) z1(1)];
plot3(X1, Y1, Z1, '-r', 'Linewidth', 2), hold on
% plot3(xf(1), yf(1), zf(1), '*r'), hold on

time1 = time(2:8000);
% x = x0;        % state
%% ************************* RUN SIMULATION *************************
OUTPUT_TO_VIDEO = 1;
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('diamond.avi');
    open(v)
end

fprintf('Simulation Running....')

x1 = out1(:,1:13);
x2 = out2(:,1:13);
x3 = out3(:,1:13);
x_ = [{x1} , {x2} , {x3}];
desired_state_1 = des1(:,1:6);
desired_state_2 = des2(:,1:6);
desired_state_3 = des3(:,1:6);
desired_state_ = [{desired_state_1} , {desired_state_2} , {desired_state_3}];
% Main loop%%%%%%%%%%%%%%%%%%%%%
Xfin = [x1(length(time)), x2(length(time)), x3(length(time)) x1(length(time))];
Yfin = [y1(length(time)), y2(length(time)), y3(length(time)) y1(length(time))];
Zfin = [z1(length(time)), z2(length(time)), z3(length(time)) z1(length(time))];
plot3(Xfin, Yfin, Zfin, '-g', 'Linewidth',2), hold on
% plot3(xf(length(time)), yf(length(time)), zf(length(time)), '*g'), hold on
    for qn = 1:nquad
        % Initialize quad plot
    x = x_{:,qn};
    desired_state = desired_state_{:,qn};
    
%         if iter == 1
%             QP{qn} = 
            QuadPlot(qn, x(max_iter,:), 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
% %             desired_state = trajhandle(time, qn);             
%             QP{qn}.UpdateQuadPlot(x(iter,:),desired_state(iter,:), time(iter));
%             h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time(iter)));
            hold on
    end
for iter = 1:1000:max_iter
%     timeint = time:tstep:time+cstep;
      timeint = time(iter)
      i=iter;
          X = [x1(i), x2(i), x3(i) x1(i)];
    Y = [y1(i), y2(i), y3(i) y1(i)];
    Z = [z1(i), z2(i), z3(i) z1(i)];
    plot3(X, Y, Z,'-b'), hold on
%     plot3(xf(i), yf(i), zf(i),'*b'), hold on
%     plot3(xfr(i), yfr(i), zfr(i), 'or'), hold on
       hold on
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
    x = x_{:,qn};
    desired_state = desired_state_{:,qn};
    
%         if iter == 1
%             QP{qn} = 
            QuadPlot(qn, x(iter,:), 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
% %             desired_state = trajhandle(time, qn);             
%             QP{qn}.UpdateQuadPlot(x(iter,:),desired_state(iter,:), time(iter));
%             h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time(iter)));
            hold on
    end
end


% % time1 = time(2:660);
% for i = 1:1000:length(time1)
%     X = [x1(i), x2(i), x3(i) x1(i)];
%     Y = [y1(i), y2(i), y3(i) y1(i)];
%     Z = [z1(i), z2(i), z3(i) z1(i)];
%     plot3(X, Y, Z,'-b'), hold on
%     plot3(xf(i), yf(i), zf(i),'*b'), hold on
%     plot3(xfr(i), yfr(i), zfr(i), 'or'), hold on
% end 
% hold on
%  i = length(time1)
%     X = [x1(i), x2(i), x3(i) x1(i)];
%     Y = [y1(i), y2(i), y3(i) y1(i)];
%     Z = [z1(i), z2(i), z3(i) z1(i)];
%     plot3(X, Y, Z,'-b'), hold on
%     plot3(xf(i), yf(i), zf(i),'*b'), hold on
%     plot3(xfr(i), yfr(i), zfr(i), 'or'), hold on

