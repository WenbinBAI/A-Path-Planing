function createfigure2(X1, YMatrix1)
%CREATEFIGURE(X1, YMATRIX1)
%  X1:  x 数据的矢量
%  YMATRIX1:  y 数据的矩阵

%  由 MATLAB 于 11-Jun-2019 09:44:44 自动生成

% 创建 figure
figure1 = figure;

% 创建 axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% 使用 plot 的矩阵输入创建多行
plot1 = plot(X1,YMatrix1);
set(plot1(1),'DisplayName','线速度','Color',[0 0 1]);
set(plot1(2),'DisplayName','角速度','Color',[1 0 0]);

% 创建 xlabel
xlabel('控制节点个数','FontSize',11);

% 创建 ylabel
ylabel('线速度(m/s) 角速度(rad/s)','FontSize',11);

% 取消以下行的注释以保留坐标轴的 X 范围
% xlim(axes1,[0 2100]);
% 取消以下行的注释以保留坐标轴的 Y 范围
% ylim(axes1,[-0.4 0.8]);
% 设置其余坐标轴属性
set(axes1,'XTick',...
    [0 100 200 300 400 500 600 700 800 900 1000 1100 1200 1300 1400 1500 1600 1700 1800 1900 2000 2100],...
    'YTick',[-0.4 -0.2 0 0.2 0.4 0.6 0.8]);
% 创建 legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.809338793362948 0.188691530461689 0.0770870441239708 0.0732931707757545]);

