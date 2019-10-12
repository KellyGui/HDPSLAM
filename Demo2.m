%
%  the version 2 : use distribution with the hyperparameter gamma to 
%                  sample the parameter of Standard Deviation
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


% New： create common view relationship between
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

HDP_lambda = 1.0;


%stdV = normrnd(0, HDP_lambda)
% noise_1 = rand()*0.3*HDP_lambda; 
% noise_2 = rand()*0.1*HDP_lambda;
% noise_3 = 0.01;

%%
%  three noise parameters are sampled from Normal distribution noise ~ N(0, lambda)
noise_1 = normrnd(0, HDP_lambda)^2; 
noise_2 = normrnd(0, HDP_lambda)^2;
noise_3 = 0.01;

% HDP_lambda1 = 0.3;  % 0.1 -  0.3
% HDP_lambda2 = 0.02;  % 0.02 - 0.1

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


