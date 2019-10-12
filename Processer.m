classdef Processer
properties
    odoms
    objects
    measurements
    fp_thre=0
    DP_alpha = 0.9
    HDP_alpha = 0.9
    HDP_gamma = 2
    odom_noise = [0.1; 0.1; 0.1]
    info_odom = [10 0 0 10 0 10]
    lm_noise = [0.5; 0.5; 1000]
    info_lm = [2 0 2]
end
methods
    function obj = setupobjects(obj,node_edge, lm_edge)
        obj.measurements=lm_edge;
        obj.odoms=node_edge;
        theta=0; pos = zeros(2,1);
        for i=1:length(node_edge.dtheta)   % 一共766个关键帧
            R=[cos(theta) sin(theta); -sin(theta) cos(theta)];   
            pos=pos+R*obj.odoms.dpos(:,i);       % dpos: 前后关键帧之间的位置偏差
            theta=theta + obj.odoms.dtheta(i);   % dtheta: 前后帧图像之间的角度变化差值；  
            theta = mod(theta+pi,2*pi)-pi;       
            obj.odoms.pos(:,i)=pos;              % pos: 当前关键帧的位置
            obj.odoms.theta(i)=theta;            % theta: 当前关键帧的朝向   
        end
        obj.measurements.obj_id = 1:length(lm_edge.id1);
        for i=1:length(lm_edge.id1)
            obj.objects(i).id=i;
            obj.objects(i).idPrev=i;
            obj.objects(i).P=zeros(5,1);            % 一共五个类别
            obj.objects(i).P(lm_edge.label(i))=1;   % 观测measurement属于的那个类的概率置为1
            obj.objects(i).msts = i;                % 观测ID：1098个观测
            t = obj.odoms.theta(lm_edge.id1(i));    % 第i个观测所在的关键帧的朝向
            R=[cos(t) sin(t); -sin(t) cos(t)];      
            % 所在关键帧位置的基础上，再加上当前观测的位置偏差乘以所在关键帧朝向角，得到当前观测点的位置坐标
            obj.objects(i).pos = obj.odoms.pos(:,lm_edge.id1(i)) + R*lm_edge.dpos(:,i);   
        end
    end
    
    function [] = write_isam(obj, filename)
        %% write file
        fid = fopen(filename,'w');       
        
        for i=1:length(obj.odoms.dtheta)
            fprintf(fid,'ODOMETRY %d %d %f %f %f',i-1,i,obj.odoms.dpos(1,i),obj.odoms.dpos(2,i),-obj.odoms.dtheta(i));
            for j=1:length(obj.info_odom) fprintf(fid,' %f',obj.info_odom(j)); end
            fprintf(fid,'\n');
        end
        
        for k=1:length(obj.objects)
            if( length(obj.objects(k).msts)>obj.fp_thre)
                for i=obj.objects(k).msts            
                    fprintf(fid,'LANDMARK %d %d %f %f', obj.measurements.id1(i), obj.objects(k).id+1000,...
                        obj.measurements.dpos(1,i), obj.measurements.dpos(2,i));
                    for j=1:length(obj.info_lm) fprintf(fid,' %f',obj.info_lm(j)); end
                    fprintf(fid,'\n');
                end
            end
        end
        fclose(fid);
    end
    
    function [obj, Iodom, Iobj] = read_isam(obj, filename)
        fid = fopen(filename);
        tline = fgetl(fid);
        odom = [];
        objects=[];
        entropy=[];
        while ischar(tline)
            if strcmp(tline(1:11),'Pose2d_Node')
                line = tline(13:end);
                line(line=='(')=[]; line(line==')')=[];line(line==';')=',';
                odom = [odom; str2num(line)];
            end
            if strcmp(tline(1:12),'Point2d_Node')
            	line = tline(14:end);
                line(line=='(')=[]; line(line==')')=[];
                objects = [objects; str2num(line)];
            end
            if strcmp(tline(1:7),'entropy')
            	line = tline(9:end);
                entropy = [entropy; str2num(line)];
            end
            tline = fgetl(fid);
        end
        fclose(fid);
        
        N=size(odom,1);
        obj.odoms.pos = odom(:,2:3)';
        obj.odoms.theta = -odom(:,4)';
%         obj.odoms.entropy = entropy(1:N,:);
        
        %         id = [obj.objects.id];
%         id = unique(id,'stable');
        id=[];
        for i=1:length(obj.objects)
            if( length(obj.objects(i).msts)>obj.fp_thre)
                id=[id obj.objects(i).id];
            end
        end
        for i=1:length(objects)
            obj_idx = [obj.objects.id]==id(i);
            obj.objects(obj_idx).pos = objects(i,2:end)';
%             obj.objects(obj_idx).entropy = entropy(i,2);
        end
    end
   
    function plot(obj)
        odom = obj.odoms.pos;
        fig = figure;
        set(fig,'Position', [100, 100, 400, 300]);
        set(fig,'Units','Inches');
        pos = get(fig,'Position');
        set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
        plot(odom(1,:),odom(2,:),'k');
        hold on; 
        
        label={'tajectory'};
        p=[obj.objects.P];
        objpose = [obj.objects.pos];
        for i=1:size(p,1)
            idx = p(i,:)>obj.fp_thre;
            if sum(idx)>0
                label{end+1}=['class ' num2str(i)];
                switch i
                case 1
                    plot(objpose(1,idx),objpose(2,idx),'bo','MarkerFaceColor','b');            
                case 2
                    plot(objpose(1,idx),objpose(2,idx),'rd','MarkerFaceColor','r');
                case 3
                    plot(objpose(1,idx),objpose(2,idx),'ms','MarkerFaceColor','m');
                case 4
                    plot(objpose(1,idx),objpose(2,idx),'g^','MarkerFaceColor',[0.2 1 0.2]);
                case 5
                    plot(objpose(1,idx),objpose(2,idx),'yp','MarkerFaceColor',[1 0.7 0.3],...
                                        'MarkerSize',15);
                end               
            end
        end
        axis equal; axis off;
        legend(label);
    end
    
    function obj = RobustSlam(obj)
        for k=1:length(obj.measurements.id1)
            t = obj.odoms.theta(obj.measurements.id1(k));
            R = [cos(t) sin(t); -sin(t) cos(t)];
            pos(:,k) = obj.odoms.pos(:,obj.measurements.id1(k))+R*obj.measurements.dpos(:,k);
        end
        
        for i=1:5
            objects(i).id = i;
            objects(i).P = zeros(5,1); 
            objects(i).P(i)=1;
            objects(i).msts = find(obj.measurements.label==i,1,'first');
            obj.measurements.obj_id(objects(i).msts)=i;
        end
        for k=1:length(obj.measurements.id1);
            msts = [objects.msts];
            sq_dist = sum((pos(:,msts) - repmat(pos(:,k),1,length(msts))).^2,1);
            posterior = exp(-0.4*sq_dist).*(obj.measurements.label(msts)==obj.measurements.label(k));
            [prob, I]=max(posterior);
            I = obj.measurements.obj_id(msts(I));
            l = obj.measurements.label(k);
            if(prob<1)
                if (prob>obj.DP_alpha)
                    objects(I).P(l)=objects(I).P(l)+1;
                    objects(I).msts = [objects(I).msts k];
                    obj.measurements.obj_id(k)=objects(I).id;
                else
                    obj.measurements.obj_id(k)=0;
                end
            end
        end
        for i=1:length(objects)
            objects(i).pos = mean(pos(:,objects(i).msts),2);
        end
        obj.objects=objects;
    end
    
    function obj = OpenLoop(obj)
        for k=1:length(obj.measurements.id1)
            t = obj.odoms.theta(obj.measurements.id1(k));
            R = [cos(t) sin(t); -sin(t) cos(t)];
            pos(:,k) = obj.odoms.pos(:,obj.measurements.id1(k))+R*obj.measurements.dpos(:,k);
        end
        
        objects.id = 1;
        objects.P = zeros(5,1); objects.P(obj.measurements.label(1))=1;
%         objects.pos = pos(:,1);
        objects.msts = 1;
        obj.measurements.obj_id(1)=1;                
        for k=2:length(obj.measurements.id1);
%             p = [objects.P];
%             obj_poses = [objects.pos];  
%             N = length(objects);
            
            sq_dist = sum((pos(:,1:k-1) - repmat(pos(:,k),1,k-1)).^2,1);
%             dp_prior = p(obj.measurements.label(k),:);
            posterior = exp(-4*sq_dist).*(obj.measurements.label(1:k-1)==obj.measurements.label(k));
            [prob, I]=max(posterior);
            I = obj.measurements.obj_id(I);
            l = obj.measurements.label(k);
            if(prob>obj.DP_alpha)
                objects(I).P(l)=objects(I).P(l)+1;
                objects(I).msts = [objects(I).msts k];
                obj.measurements.obj_id(k)=objects(I).id;
%                 objects(I).pos = mean(pos(:,objects(I).msts),2);
            else
                N_obj = max([objects.id]);
                object.id = N_obj+1;
                object.P = zeros(5,1);object.P(l)=1;
%                 object.pos = pos(:,k);
                object.msts = k;
                objects(end+1)=object;
                obj.measurements.obj_id(k) = N_obj+1;
            end
        end
        for i=1:length(objects)
            objects(i).pos = mean(pos(:,objects(i).msts),2);
        end
        obj.objects=objects;
    end
    
    function obj = DPsample(obj)
        
        % for each measurement y_t^k
        % 遍历1089个观测点
        for k=length(obj.measurements.id1):-1:1    
            
            %% Step 1: Maximize likelihood 
            obj_idx = find([obj.objects.id]==obj.measurements.obj_id(k))
            l=obj.measurements.label(k);
            obj.objects(obj_idx).P(l)=obj.objects(obj_idx).P(l)-1;
            if sum(obj.objects(obj_idx).P)==0
                obj.objects(obj_idx)=[];
            else
                obj.objects(obj_idx).msts(obj.objects(obj_idx).msts==k)=[];
            end
            p = [obj.objects.P];  % 去除当前观测点后，其他观测点的标签信息
            obj_poses = [obj.objects.pos];   % 去除当前观测点后，其他观测点的位置信息
            
            N = length(obj.objects);

            % 第k个观测所在关键帧的朝向角度
            t = obj.odoms.theta(obj.measurements.id1(k));
            R = [cos(t) sin(t); 
                -sin(t) cos(t)];
            % 所在关键帧位置的基础上，第k个观测点的位置
            pos = obj.odoms.pos(:,obj.measurements.id1(k))+R*obj.measurements.dpos(:,k);
            
            % DP(i)先验信息
            % 除了当前观测点之外的剩下观测点中，其他观测点包含所在观测点的标签情况，值为1的表示为跟当前观测相同的标签
            dp_prior = p(obj.measurements.label(k),:);   
            % 当前观测点跟其他观测点的距离
            sq_dist = sum((obj_poses - repmat(pos,1,N)).^2,1);
            
            
            % Compute posterior p_i of being object i:
            posterior = exp(-4*sq_dist).*dp_prior;   %点乘选择跟当前观测有相同标签的观测点的距离差
            
            % y_t^k = argmax(posterior)
                 
            [prob, I]=max(posterior);   % 跟当前观测点距离最近的观测点有较大概率被选中； 等价于 [aa, I]=min(sq_dist)； 
            
            
            %% STep 2: Assign data association to their maximum likelihood objects
            % y_t is assigned to be the maximum likelihood object if the
            % likelihood is within some certain threshold, 
            % otherwise it is assigned to a new object.
            if(prob>obj.DP_alpha)  % 如果这个后验概率大于某个阈值，则在找到的新观测点所在标签数目加一，观测列表加入当前的观测
                obj.objects(I).P(l)=obj.objects(I).P(l)+1;
                obj.objects(I).msts = [obj.objects(I).msts k];
                obj.measurements.obj_id(k)=obj.objects(I).id;
            else
                N_obj = max([obj.objects.id]);
                object.id = N_obj+1;
                object.P = zeros(5,1);object.P(l)=1;
                object.pos = pos;
                object.msts = k;
                object.idPrev = k;
                obj.objects=[obj.objects object];
                obj.measurements.obj_id(k) = N_obj+1;
            end
        end
    end
    
    
    
    function obj = HDPsample(obj)
        
        % for each measurement y_t^k
        % 遍历1089个观测点
        %for k=length(obj.measurements.id1):-1:1  
        for k=length(obj.measurements.id1):-1:1  
            
            %k
            %% Step 1: Maximize likelihood 
            obj_idx = find([obj.objects.id]==obj.measurements.obj_id(k));
            l=obj.measurements.label(k);
            obj.objects(obj_idx).P(l)=obj.objects(obj_idx).P(l)-1;
            if sum(obj.objects(obj_idx).P)==0
                obj.objects(obj_idx)=[];
            else
                obj.objects(obj_idx).msts(obj.objects(obj_idx).msts==k)=[];
            end
            p = [obj.objects.P];  % 去除当前观测点后，其他观测点的标签信息
            obj_poses = [obj.objects.pos];   % 去除当前观测点后，其他观测点的位置信息
            
            
            
            % To do
%             idx_frame = obj.measurements.id1(k);
%             commonviewKeyFrame = find(obj.measurements.commonview(idx_frame, :)==1);
%             
%             measurement_Tmp = [];
%             
%             idPrev1 = [obj.objects.idPrev];
%             for i = 1:length(commonviewKeyFrame)
%                 tmp = find(obj.measurements.id1==commonviewKeyFrame(i));
%                 measurement_Tmp = [measurement_Tmp tmp];
%             end
%             
%             measurement_Tmp = [measurement_Tmp k];
            
            measurement_Tmp = obj.measurements.measurements_Commonview{k};

            p2 = [];
            pos1 = [];
            dp_prior = [];
            for i = 1:length(measurement_Tmp)
                tmp = find([obj.objects.idPrev]==measurement_Tmp(i));
                if ~isempty(tmp)
                    pos1 = [pos1 obj_poses(:,tmp)];
                    dp_prior = [dp_prior p(obj.measurements.label(k),tmp)*obj.HDP_gamma];
                    p2 = [p2 tmp];
                end
            end
            
            N = size(pos1,2);

            % 第k个观测所在关键帧的朝向角度
            t = obj.odoms.theta(obj.measurements.id1(k));
            R = [cos(t) sin(t); 
                -sin(t) cos(t)];
            % 所在关键帧位置的基础上，第k个观测点的位置
            pos = obj.odoms.pos(:,obj.measurements.id1(k))+R*obj.measurements.dpos(:,k);
            
            % DP(i)先验信息
            % 除了当前观测点之外的剩下观测点中，其他观测点包含所在观测点的标签情况，值为1的表示为跟当前观测相同的标签
            %dp_prior = p(obj.measurements.label(k),:);   
            % 当前观测点跟其他观测点的距离
%             if isempty(pos1)
%                 k
%             end
%             size(pos1)
%             N
            sq_dist = sum((pos1 - repmat(pos,1,N)).^2,1);
            
            % Compute posterior p_i of being object i:
            posterior = exp(-4*sq_dist).*dp_prior;   %点乘选择跟当前观测有相同标签的观测点的距离差
            
            % y_t^k = argmax(posterior)
                 
            [prob, I1]=max(posterior);   % 跟当前观测点距离最近的观测点有较大概率被选中； 等价于 [aa, I]=min(sq_dist)； 
            
            I = p2(I1);
            
            
            %% STep 2: Assign data association to their maximum likelihood objects
            % y_t is assigned to be the maximum likelihood object if the
            % likelihood is within some certain threshold, 
            % otherwise it is assigned to a new object.
            if(prob>obj.HDP_alpha)  % 如果这个后验概率大于某个阈值，则在找到的新观测点所在标签数目加一，观测列表加入当前的观测
                obj.objects(I).P(l)=obj.objects(I).P(l)+1;
                obj.objects(I).msts = [obj.objects(I).msts k];
                obj.measurements.obj_id(k)=obj.objects(I).id;
            else
                N_obj = max([obj.objects.id]);
                object.id = N_obj+1;
                object.P = zeros(5,1);object.P(l)=1;
                object.pos = pos;
                object.msts = k;
                object.idPrev = k;
                obj.objects=[obj.objects object];
                obj.measurements.obj_id(k) = N_obj+1;
            end
        end
    end
    
    
    
    
    
    function [obj]= optimizeDP(obj,maxiteration)
        for i=1:maxiteration
            obj.plot();
            title(sprintf('Iter %d, %d objects', i-1, length(obj.objects)))
            obj=obj.DPsample();
            obj.write_isam('isam_input.txt');
            !isam/bin/isam isam_input.txt -W isam_output.txt -B
            [obj] = obj.read_isam('isam_output.txt');
        end

    end
    
    
    function [obj]= optimizeHDP(obj,maxiteration)

        for i=1:maxiteration
            obj.plot();
            title(sprintf('Iter %d, %d objects', i-1, length(obj.objects)))
            obj=obj.HDPsample();
            obj.write_isam('isam_input.txt');
            !isam/bin/isam isam_input.txt -W isam_output.txt -B
            [obj] = obj.read_isam('isam_output.txt');
        end
        
    end
    
    function [Iodom, Iobj] = computeEntropy(obj)
        factorgraph     = gtsam.NonlinearFactorGraph;
        estimates = gtsam.Values;
        odom_cov = gtsam.noiseModel.Diagonal.Sigmas(obj.odom_noise);
        ms_cov = gtsam.noiseModel.Diagonal.Sigmas(obj.lm_noise);
        
        factorgraph.add(gtsam.NonlinearEqualityPose2(0,gtsam.Pose2));
        estimates.insert(0, gtsam.Pose2);
        
        odoms = [obj.odoms.pos; obj.odoms.theta];
        dodom = [obj.odoms.dpos; obj.odoms.dtheta];
        for k=1:size(dodom,2)
            estimates.insert(k,gtsam.Pose2(odoms(:,k)));
            factorgraph.add(gtsam.BetweenFactorPose2(k-1, k , gtsam.Pose2(dodom(:,k)), odom_cov));
        end
        
        objpos = [obj.objects.pos];
        for k=1:length(obj.objects)
            estimates.insert(obj.objects(k).id+1000, gtsam.Pose2([objpos(:,k);0]));
        end
        id1=obj.measurements.id1;
        id2=obj.measurements.obj_id;
        for k=1:length(id1)
            factorgraph.add(gtsam.BetweenFactorPose2(...
                id1(k), id2(k)+1000, gtsam.Pose2([obj.measurements.dpos(:,1);0]), ms_cov));
        end
        
        marginals = gtsam.Marginals(factorgraph,estimates);
        Iodom=zeros(size(dodom,2),1);
        for k=1:size(dodom,2)
            Iodom(k) = log(det(marginals.marginalCovariance(k)));
        end
        Iobj=zeros(length(obj.objects),1);
        for k=1:length(obj.objects)
            covar = marginals.marginalCovariance(obj.objects(k).id+1000);
            Iobj(k) = log(det(covar(1:2,1:2)));
        end 
    end
    
    function [err_odom, err_obj]=computeError(obj, true_traj,true_obj)
        est_traj = [obj.odoms.pos];
        Nodom = min([length(true_traj) length(obj.odoms.pos)]);
        true_traj(3,:)=0; true_traj=true_traj(:,1:Nodom);
        est_traj(3,:)=0; est_traj=est_traj(:,1:Nodom);
        [T] = estimateRigidTransform(true_traj,est_traj);
        est_traj(4,:)=1; est_traj = T*est_traj;
        err_odom = sum( (true_traj(1:2,:)-est_traj(1:2,:)).^2,1);
        
        idx = obj.measurements.obj_id>0;
        tobj  = true_obj(:,obj.measurements.id2(idx));
        tobj(3,:)=0;
        
        objpos = [obj.objects.pos];
        for i=1:length(obj.objects)
            id2idx(obj.objects(i).id)=i;
        end
        eobj = objpos(:,id2idx(obj.measurements.obj_id(idx)));
        eobj(3,:)=0; eobj(4,:)=1;
        eobj = T*eobj;
        err_obj = sum( (tobj(1:2,:)-eobj(1:2,:)).^2,1);
    end
    
end    
end

function [ rectsLTWH ] = RectLTRB2LTWH( rectsLTRB )
%rects (l, t, r, b) to (l, t, w, h)
rectsLTWH = [rectsLTRB(:, 1), rectsLTRB(:, 2), rectsLTRB(:, 3)-rectsLTRB(:,1)+1, rectsLTRB(:,4)-rectsLTRB(:,2)+1];
end

function rpy = rot2angle(R)
% QUAT2ROT
%   R = QUAT2ROT(Q) converts a quaternion (4x1 or 1x4) into a 3x3 rotation mattrix
%
%   reference: google!
%
%   Babak Taati, 2003
%   (revised 2009)

rpy(1) = atan2(R(3,2),R(3,3));
rpy(2) = -asin(R(3,1));
rpy(3) = atan2(R(2,1),R(1,1));
end