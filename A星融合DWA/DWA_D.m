function a = DWA_D(Obs_Closed,Obs_d_j,Start,Target,Area_MAX)
 
num_obc=size(Obs_Closed,1);


 
%disp('Dynamic Window Approach sample program start!!')
 xm=Start(1,1);
 ym=Start(1,2);
x=[xm ym pi/2 0 0]';% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
goal=Target;% 目标点位置 [x(m),y(m)]
obstacle=Obs_Closed;
      
obstacleR=0.8;% 冲突判定用的障碍物半径
global dt; dt=0.1;% 时间[s]
% 机器人运动学模型
% 最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]]
%Kinematic=[1, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)];
Kinematic=[2.0, toRadian(40.0), 0.4, toRadian(100.0), 0.01, toRadian(1)]; 
% 评价函数参数 [heading,dist,velocity,predictDT][0.05,  0.2,  0.1,  3.0]
evalParam=[0.05,  0.5,  0.3,  3.0];
MAX_X=Area_MAX(1,1);
MAX_Y=Area_MAX(1,2);
% 模拟区域范围 [xmin xmax ymin ymax]
% 模拟实验的结果
result.x=[];

% movcount=0;
% Main loop
for i=1:5000
    % DWA参数输入
    [u,traj]=DynamicWindowApproach_ct(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x=f(x,u);% 机器人移动到下一个时刻
    
    % 模拟结果的保存
    result.x=[result.x; x'];
    
    % 是否到达目的地
    if norm(x(1:2)-goal')<0.2
        disp('Arrive Goal!!');break;
    end
    
    %====Animation====
    hold off;
 
    for i_obs=1:1:num_obc
         x_obs=Obs_Closed(i_obs,1);
         y_obs=Obs_Closed(i_obs,2);
         fill([x_obs,x_obs+1,x_obs+1,x_obs],[y_obs,y_obs,y_obs+1,y_obs+1],'k');hold on;
    end
     dong_num=size(Obs_d_j,1);
    for i_d=1:1:dong_num
      x_do=Obs_d_j(i_d,1);
      y_do=Obs_d_j(i_d,2);
      fill([x_do,x_do+1,x_do+1,x_do],[y_do,y_do,y_do+1,y_do+1],[0.8 0.8 0.8]);
    end 
    
    ArrowLength=0.5;% 
    % 机器人
    quiver(x(1)+0.5,x(2)+0.5,ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
    plot(result.x(:,1)+0.5,result.x(:,2)+0.5,'-b');hold on;
    %plot(goal(1)+0.5,goal(2)+0.5,'*r');hold on;
  
     
   % 探索轨迹
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:)+0.5,traj(ind+1,:)+0.5,'-g');hold on;%%模拟轨迹
        end
    end
%     axis(area);
%     grid on;


    axis([0 MAX_X+1, 0 MAX_Y+1])                %%%  设置x，y轴上下限
    set(gca,'xtick',0:1:MAX_X+1,'ytick',0:1:MAX_Y+1,'GridLineStyle','-',...
        'xGrid','on','yGrid','on');   
    grid on;       
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% 
    
end

a=result.x;

%movie2avi(mov,'movie.avi');






 

 

 

 

 

