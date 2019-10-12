%
%  the version 1 : the parameter of Standard Deviation is setted 
%
%
% our implementation based on the original code of the paper 
%    < SLAM with objects using a nonparametric pose graph >
% the github: 
%    https://github.com/BeipengMu/objectSLAM
%
%
%

close all;
clc;
clear;

addpath('include/');

%% load data and add noise
load('data/simulation.mat');

% generate Simulation Data


% New： create common view relationships
if ~exist('commonview.mat')
    commonview = zeros(length(node_edge.dpos), length(node_edge.dpos));
    %commonview = zeros(max(lm_edge.id1), max(lm_edge.id1));

    tmp = cell(length(node_edge.dpos),1);
    commonviewFrame = cell(length(node_edge.dpos),1);
    for i = 1:length(node_edge.dpos)
        tmp{i} = lm_edge.id2(find(lm_edge.id1==i));
    end

    for i = 1:length(tmp)
        i
        t1 = tmp{i};

        for j = (i+1):length(tmp)

            t2 = tmp{j};
            t3 = intersect(t1, t2);
            if ~isempty(t3)
                commonview(j,i) = 1;
                commonview(i,j) = commonview(j,i);
            end
        end

        commonviewFrame{i} = find(commonview(i,:)==1);
    end
    
    measurements_Commonview = cell(length(lm_edge.id1), 1);
    for i = 1:length(lm_edge.id1) 
        
        idx_frame = lm_edge.id1(i);
        commonviewKeyFrame = find(commonview(idx_frame, :)==1);
            
        measurement_Tmp = [i];
        for j = 1:length(commonviewKeyFrame)
            tmp = find(lm_edge.id1==commonviewKeyFrame(j));
            measurement_Tmp = [measurement_Tmp tmp];
        end
        measurements_Commonview{i} = measurement_Tmp;
    end
    
    
    save commonview commonview
    save commonviewFrame commonviewFrame
    save measurements_Commonview measurements_Commonview
else
    load commonview
    load commonviewFrame
    load measurements_Commonview
end

lm_edge.commonview = commonview;
lm_edge.commonviewFrame = commonviewFrame;
lm_edge.measurements_Commonview = measurements_Commonview;

%%
%  three noise parameters are setted here
noise_1 = 0.1;     % 0.1 -  0.3
noise_2 = 0.02;    % 0.02 - 0.1
noise_3 = 0.01;

lm_edge.dpos = lm_edge.dpos + randn(2,1098)*noise_1;  % 0.1
node_edge.dpos = node_edge.dpos+ randn(size(node_edge.dpos))*noise_2; % 0.02 
node_edge.dtheta = node_edge.dtheta + randn(size(node_edge.dtheta))*noise_3; % 0.01

%% plot dataset
fig = figure;
set(fig,'Position', [100, 100, 400, 300]);
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

plot(truth_traj(:,1),truth_traj(:,2),'k');
hold on; 
        
label={'tajectory'};
for i=1:5
    idx = truth_objects(:,3)==i-1;
    if sum(idx)>0
        label{end+1}=['class ' num2str(i)];
        switch i
        case 1
            plot(truth_objects(idx,1),truth_objects(idx,2),'bo','MarkerFaceColor','b');            
        case 2
            plot(truth_objects(idx,1),truth_objects(idx,2),'rd','MarkerFaceColor','r');
        case 3
            plot(truth_objects(idx,1),truth_objects(idx,2),'ms','MarkerFaceColor','m');
        case 4
            plot(truth_objects(idx,1),truth_objects(idx,2),'g^','MarkerFaceColor',[0.2 1 0.2]);
        case 5
            plot(truth_objects(idx,1),truth_objects(idx,2),'yp','MarkerFaceColor',[1 0.7 0.3],'MarkerSize',15);
        end               
   end
end
axis equal; axis off;
legend(label);
title('GroundTruth');
        
%% DP
pr = Processer();
pr = pr.setupobjects(node_edge,lm_edge);
% pr.plot();
pr = pr.optimizeDP(10);
[Eodom_NP, Eobj_NP]=pr.computeError(truth_traj',truth_objects');
pr.plot();
title('DP non-parametric');


%% HDP
pr = Processer();
pr = pr.setupobjects(node_edge,lm_edge);
% pr.plot();
pr = pr.optimizeHDP(10);
[Eodom_NP, Eobj_NP]=pr.computeError(truth_traj',truth_objects');
pr.plot();
title('HDP non-parametric');



% %% frame based
% pr_frame = Processer();
% pr_frame = pr_frame.setupobjects(node_edge,lm_edge);
% % [Iodom_frame, Iobj_frame]=pr_frame.computeEntropy();
% [Eodom_frame, Eobj_frame]=pr_frame.computeError(truth_traj',truth_objects');
% pr_frame.plot();
% title('frame based');
% 
% 
% %% open loop
% dp_alpha = 0.2:0.1:0.9;
% for i=1:length(dp_alpha)
%     pr_OL = Processer();
%     pr_OL.DP_alpha=dp_alpha(i);
%     pr_OL = pr_OL.setupobjects(node_edge,lm_edge);
%     pr_OL = pr_OL.OpenLoop;
% %     pr_OL.plot();
%     % [Iodom_OL, Iobj_OL]=pr_OL.computeEntropy();
%     [Eodom_OL, Eobj_OL]=pr_OL.computeError(truth_traj',truth_objects');
%     err_obj_ol(i)=mean(sqrt(Eobj_OL));
%     no_obj_ol(i)=length(pr_OL.objects);
% end
% pr_OL.plot();
% title('open loop');
% 
% %% robust slam
% for i=1:length(dp_alpha)
%     pr_Robust = Processer();
%     pr_Robust.DP_alpha=dp_alpha(i);
%     pr_Robust = pr_Robust.setupobjects(node_edge,lm_edge);
%     pr_Robust = pr_Robust.RobustSlam;
%     pr_Robust.write_isam('isam_input.txt');
%     % CPlusPlus program
%     !isam/bin/isam isam_input.txt -W isam_output.txt -B  
%     pr_Robust = pr_Robust.read_isam('isam_output.txt');
% %     pr_Robust.plot();
%     % [Iodom_PL, Iobj_PL]=pr_PL.computeEntropy();
%     [Eodom_Robust, Eobj_Robust]=pr_Robust.computeError(truth_traj',truth_objects');
%     err_obj_robust(i)=mean(sqrt(Eobj_Robust));
%     no_obj_robust(i)=length(pr_Robust.objects);
% end
% pr_Robust.plot();
% title('robust slam');
% 
% %%
% algo = {'NP pose graph'; 'Open Loop'; 'Robust SLAM'; 'Frame by Frame'};
% mean_odom = mean( sqrt([Eodom_NP; Eodom_OL; Eodom_Robust; Eodom_frame]),2);
% cum_odom = sum( sqrt([Eodom_NP; Eodom_OL; Eodom_Robust; Eodom_frame]),2);
% no_msts = sum( [pr.measurements.obj_id; pr_OL.measurements.obj_id;...
%     pr_Robust.measurements.obj_id; pr_frame.measurements.obj_id]>0,2);
% no_obj = [length(pr.objects); length(pr_OL.objects); length(pr_Robust.objects); length(pr_frame.objects)];
% err_obj = [mean(sqrt(Eobj_NP)); mean(sqrt(Eobj_OL)); mean(sqrt(Eodom_Robust)); mean(sqrt(Eobj_frame))];
% T = table(mean_odom,cum_odom,no_msts,no_obj,err_obj,...
%     'RowNames',algo);
% %%
% figure
% set(0,'DefaultLineMarkerSize',10)
% semilogx(no_obj(1), err_obj(1),'ko','MarkerFaceColor','k');hold on;
% semilogx(no_obj_ol, err_obj_ol,'b^','MarkerFaceColor','b');
% semilogx(no_obj_robust, err_obj_robust,'rs','MarkerFaceColor','r');
% semilogx(no_obj(4), err_obj(4),'mp','MarkerFaceColor','m');
% semilogx(15, 0,'gd','MarkerFaceColor',[0 0.8 0]);
% xlabel('number of objects');ylabel error;
% legend('NP-Graph','OL','R-SLAM','FbF','Ground Truth');
% 
% %%
% figure
% semilogy(cumsum(sqrt(Eodom_NP(1:20:end))),'k-o');hold on;
% semilogy(cumsum(sqrt(Eodom_OL(1:20:end))),'b-^');
% semilogy(cumsum(sqrt(Eodom_Robust(1:20:end))),'r-s');
% semilogy(cumsum(sqrt(Eodom_frame(1:20:end))),'m-p');
% xlabel time;ylabel 'cumulative trajectory error';
% legend('NP-Graph','OL','R-SLAM','FbF');

