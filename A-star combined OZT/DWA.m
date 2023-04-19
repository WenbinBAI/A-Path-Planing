function a = DWA(Obs_Closed,Obs_d_j,Area_MAX,Goal,Line_path,path_node,Start0,s_du,Kinematic,evalParam)
figure
num_obc=size(Obs_Closed,1); %  计算障碍物的数量
num_path=size(path_node,1);
xTarget=path_node(num_path,1);
yTarget=path_node(num_path,2);
% 
% num_odL=size(Obst_d_d_line,1);
%  Obst_d_line=[];


xm=path_node(1,1);
ym=path_node(1,2);
 % 初始位置坐标 
 %angle_S=pi;
 angle_node=sn_angle(path_node(1,1),path_node(1,2),path_node(2,1),path_node(2,2));
 du_flog=1;
 
  du_max=angle_node+pi/18;
  du_min=angle_node-pi/18;

 %zhuangjiao_node=angle_S-angle_node;
 
 x=[xm ym s_du 0 0]';% 机器人的初期状态 x=[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
 
 xt_yu=[xm ym];
%G_goal=path_node(num_path,:);% 目标点位置 [x(m),y(m)]

 obstacle=Obs_Closed;
 Obs_dong=Obs_d_j  ;   
obstacleR=0.8;% 静态障碍物 冲突判定用的障碍物半径
R_dong_obs=0.7; % 动态障碍物 冲突判定用的障碍物半径
global dt;  %   全局变量
dt=0.1;%   时间[s]

% 机器人运动学模型
% [   最高速度[m/s], 最高旋转速度[rad/s], 加速度[m/ss], 旋转加速度[rad/ss], 速度分辨率[m/s], 转速分辨率[rad/s]  ]
% 
%  Kinematic=[2.0, toRadian(40.0), 0.5, toRadian(120.0), 0.01, toRadian(1)];
% %Kinematic=[1, toRadian(20.0), 0.3, toRadian(60), 0.01, toRadian(1)];
% % 评价函数参数 [heading,dist,velocity,predictDT][方位角偏差系数， 障碍物距离系数， 当前速度大小系数, 动态障碍物距离系数，预测是时间 ]
% evalParam=[0.2,  0.1,  0.3, 0.4, 3.0];%0.3 0.1 0.1   [0.05,  0.2,  0.1,  3.0] [0.2,  0.5,  0.3,  3.0]
MAX_X=Area_MAX(1,1);
MAX_Y=Area_MAX(1,2);
% 模拟区域范围 [xmin xmax ymin ymax]
% 模拟实验的结果
result.x=[];
goal=path_node(2,:);

tic;
% movcount=0;
% Main loop
for i=1:5000
   
    dang_node=[x(1,1) x(2,1)];
    dis_ng=distance(dang_node(1,1),dang_node(1,2),xTarget,yTarget);
    dis_x_du=distance(xt_yu(1,1),xt_yu(1,2),goal(1,1),goal(1,2));
    if num_path==2||dis_ng<0.5
        Ggoal=[xTarget yTarget];
    else
        Ggoal=Target_node(dang_node,path_node,Obs_dong,xTarget,yTarget,goal,dis_x_du);
    end
    goal=Ggoal;
    % obstacle=OBSTACLE(Obs_Closed,Obs_dong,path_node);
    if (s_du>du_max||s_du<du_min)&&du_flog==1
        [u,traj]=DynamicWindowApproach_du(x,Kinematic,goal,evalParam,obstacle,obstacleR,Obs_dong,R_dong_obs);
        x=f(x,u);% 机器人移动到下一个时刻
        result.x=[result.x; x'];
        if angle_node>=(17/18)*pi
            if x(3)>=du_min %x(3)==angle_node
              du_flog=0;
            end
        elseif angle_node<=(-17/18)*pi
            if x(3)<=du_max %x(3)==angle_node
              du_flog=0;
            end
        else
            if x(3)<=du_max&&x(3)>=du_min %x(3)==angle_node
              du_flog=0;
            end
        end
    else
         [u,traj,xt_yu]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR,Obs_dong,R_dong_obs);
         % u = [ 速度 转速 ] traj=[ 3s内的所有状态轨迹 ]
         x=f(x,u);% 机器人移动到下一个时刻
         result.x=[result.x; x'];
         
    end
    % 模拟结果的保存
    
    
    % 是否到达目的地
  
    %if norm(x(1:2)-G_goal')<0.2
    if dis_ng<0.2
        disp('Arrive Goal!!');break;
    end
    
    %====Animation====
    hold off;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %  画图
   for i_obs=1:1:num_obc
         x_obs=Obs_Closed(i_obs,1);
         y_obs=Obs_Closed(i_obs,2);
         fill([x_obs,x_obs+1,x_obs+1,x_obs],[y_obs,y_obs,y_obs+1,y_obs+1],'k');hold on;
    end
     plot( Line_path(:,1)+.5, Line_path(:,2)+.5,'b:','linewidth',1); 
     plot(Start0(1,1)+.5,Start0(1,2)+.5,'b^');
     plot(Goal(1,1)+.5,Goal(1,2)+.5,'bo'); 

%     dong_num=size(Obs_d_j,1);
%     for i_d=1:1:dong_num
%       x_do=Obs_d_j(i_d,1);
%       y_do=Obs_d_j(i_d,2);
%       fill([x_do,x_do+1,x_do+1,x_do],[y_do,y_do,y_do+1,y_do+1],[0.8 0.8 0.8]);
%     end
%     fill([7.2,7.8,7.8,7.2],[10.2,10.2,10.8,10.8],[0.8 0.8 0.8]);hold on;
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%     fill([7.2,7.8,7.8,7.2],[7.2,7.2,7.8,7.8],[0.8 0.8 0.8]);hold on;
    ArrowLength=0.5;% 
    % 机器人
    quiver(x(1)+0.5,   x(2)+0.5,  ArrowLength*cos(x(3)),  ArrowLength*sin(x(3)),'ok');hold on;
    %  x=[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
    plot(result.x(:,1)+0.5, result.x(:,2)+0.5,'-b');hold on;
    
    plot(goal(1)+0.5,goal(2)+0.5,'*r');hold on;
  
     
   % 探索轨迹
  %  % traj = [ （第一组5行 v w ）3s内的所有状态轨迹 31个点 1-5行31列；
  %             （第二组5行 v w ）3s内的所有状态轨迹 31个点 6-10行31列；
  %           。。。。。。]
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5 % 
            ind=1+(it-1)*5;
            plot(traj(ind,:)+0.5,traj(ind+1,:)+0.5,'-g','linewidth',1.5);hold on;%%模拟轨迹
        end
    end
%     axis(area);
%     grid on;


    axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限
    set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
        'xGrid','on','yGrid','on');   
   grid on; 
    
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% 
    
end

a=result.x;
toc
%movie2avi(mov,'movie.avi');
 





 

 

 

 

 

