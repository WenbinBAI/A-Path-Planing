function Plan_path = A_path(xStart,yStart,xTarget,yTarget,Num_obs,CLOSED,MAX_X,MAX_Y)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* ALGORITHM Demo

% �Ľ�A*�㷨   1  8������������ 5��,������������
%             2  ��б���ϰ��ﶥ��  ���ⷢ����ײ
%             3  ɾ���м����ڵ㣬����ת�ۣ�����·����ƽ����
%             4  ���ۺ�����f(n)=g(n)+h(n) ��Ϊ��f(n)=g(n)+��1+r/R)*h(n)
%                          ����rΪ��ǰ�㵽Ŀ���ľ��룬RΪ��ʼ�㵽Ŀ���ľ��롣

%%%ֻ�����������ξ����к�����ȣ�������תʱ����ִ���




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM %%%�����㷨���б�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE  %%%�����б��ṹ
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE %%% ��յ��б��ṹ
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2); %%% ����MAX_VAL�У�2�е�0����
CLOSED_COUNT=size(CLOSED,1);   %%% CLOSED�����������ϰ���ĸ��� 
Nobs=CLOSED_COUNT;
%set the starting node as the first node %%%����ʼ�ڵ�����Ϊ��һ���ڵ�
xNode=xStart;      %%% =xStart
yNode=yStart;      %%% =yStart
OPEN_COUNT=1;    %%% OPEN_COUNT �����б���������־
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);                                                                                    %%%  ***����distance�������������������֮��ĵѿ�������
Sgoal=goal_distance;
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);  %%%   ���뵽�����б�
                            %%%        OPEN����һ�е�Ԫ�أ�=��1��xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distanc����
OPEN(OPEN_COUNT,1)=0;      %%%   OPEN(1,1)=0
CLOSED_COUNT=CLOSED_COUNT+1;  %%%   CLOSED �洢���ϰ������һ����Ԫ
CLOSED(CLOSED_COUNT,1)=xNode; %%%   ��һ���洢��ʼ��� ����
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;
% p_xNode=xNode;
% p_yNode=yNode;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM ��ʼ�㷨
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)       %%%  �жϵ�ǰ���Ƿ����Ŀ���
%  plot(xNode+.5,yNode+.5,'go');
%  xnode=xNode,ynode=yNode  %%%****�����ǰ�ڵ㣬����ѧϰ�˽�A*�㷨���������****  ///����Ҫ֪�����̿�ע�͵�///
 exp_array=expand_array06(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y,Nobs,xStart,yStart);  %%% ���ڹر��б����ӽڵ㣬��x,y,gn,hn,fn��,�����Ǹ���
 exp_count=size(exp_array,1);   %%%  ��ѡ����ӽڵ����
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT ��չ���и�ʽ
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count         %%% ��exp_array�ڵ�Ԫ�����ӵ� �����б� ����
    flag=0;                %%% ��exp_array�ڵĵ�ı�־λ��Ϊ0
    for j=1:OPEN_COUNT         %%% OPEN_COUNT ��1��ʼ���Լ�
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )    %%%�жϿ�ѡ�ӽڵ��Ƿ���OPEN[]�еĵ���ͬ
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5));                       %%%�����ͬ���Ƚ�����fn��ֵ�Ĵ�С������fnС������㸳ֵ��OPEN(j,8)
            if OPEN(j,8)== exp_array(i,5)                                  %%% ��ʾ����һ���Ƚ��� exp_array(i,5)С�����exp_array(i,�����е�ֵ����OPEN
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;                    %%%����ϣУţ���ͬ�ģ���磽1
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1; 
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn  �ҳ�fn��С�Ľڵ�
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);   %%%ѡ��fn��С��һ�У����������� index_min_node
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn ��xNode��yNode����Ϊ��Сfn�Ľڵ�
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);% Update the cost of reaching the parent node ���µ��︸�ڵ�ĳɱ�  gn
  
%    p_xNode=OPEN(index_min_node,4);
%    p_yNode=OPEN(index_min_node,5);
   %Move the Node to list CLOSED   ���ڵ��ƶ����б�CLOSED  
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
%   CLOSED  %%%****���CLOSE[]������ѧϰ�˽�A*�㷨���������****  ///����Ҫ֪�����̿�ע�͵�///
%   OPEN    %%%****���OPEN[]������ѧϰ�˽�A*�㷨���������****  ///����Ҫ֪�����̿�ע�͵�///
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);    %%%CLOSE����ĳ���
Optimal_path=[];     %%%·������
xval=CLOSED(i,1);    %%%��CLOSE���һ��������������һ����ΪĿ���
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval; %%%��Ŀ�������긳�� ·������� ��һ��
Optimal_path(i,2)=yval;
            
if ( (xval == xTarget) && (yval == yTarget))  %%%���CLOSE���һ���Ƿ�ΪĿ���
 
   %Traverse OPEN and determine the parent nodes ����OPEN��ȷ�����ڵ�
   Target_ind=node_index(OPEN,xval,yval);
   parent_x=OPEN(Target_ind,4); %node_index returns the index of the node  node_index���ؽڵ������
   parent_y=OPEN(Target_ind,5);%%% ����ǰ��ĸ��ڵ������
   Optimal_path(i,3)=parent_x; 
   Optimal_path(i,4)=parent_y;
   
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 while( parent_x ~= xStart || parent_y ~= yStart)   %%% �жϸ��ڵ��Ƿ�Ϊ��ʼ��
           i=i+1; 
           Optimal_path(i,1) = parent_x;             %%% ���� �򽫸��ڵ��͸�·������
           Optimal_path(i,2) = parent_y;
          
           inode=node_index(OPEN,parent_x,parent_y); 
           
           Optimal_path(i,3) = OPEN(inode,4);
           Optimal_path(i,4) = OPEN(inode,5);
           parent_x=Optimal_path(i,3);
           parent_y=Optimal_path(i,4);
        
 end;
  Num_Opt=size(Optimal_path,1);
 
 %%%  �Ż�����
 Optimal_path_one=Line_OPEN_ST(Optimal_path,CLOSED,Num_obs,Num_Opt);
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%   ��·����ȡ����  %%%%%%%%%%%%%%%%%%%%%
 Optimal_path_try=[1 1 1 1];
 Optimal_path=[1 1 ];node_l=[1 1];
 i=1;q=1;
 x_g=Optimal_path_one(Num_Opt,3);y_g=Optimal_path_one(Num_Opt,4);
 
 Optimal_path_try(i,1)= Optimal_path_one(q,1);
 Optimal_path_try(i,2)= Optimal_path_one(q,2);
 Optimal_path_try(i,3)= Optimal_path_one(q,3);
 Optimal_path_try(i,4)= Optimal_path_one(q,4);
 
   while (Optimal_path_try(i,3)~=x_g || Optimal_path_try(i,4)~=y_g)
      i=i+1;
      q=Optimal_index(Optimal_path_one,Optimal_path_one(q,3),Optimal_path_one(q,4));
      
      Optimal_path_try(i,1)= Optimal_path_one(q,1);
      Optimal_path_try(i,2)= Optimal_path_one(q,2);
      Optimal_path_try(i,3)= Optimal_path_one(q,3);
      Optimal_path_try(i,4)= Optimal_path_one(q,4);
      
   end
 %%%%%%%%%%%%%%%     ����������·�߽ڵ�      %%%%%%%%%%%%%%%%%%%%%

 n=size(Optimal_path_try,1);
   for i=1:1:n
       Optimal_path(i,1)=Optimal_path_try(n,3);
       Optimal_path(i,2)=Optimal_path_try(n,4);
%        Optimal_path(i,3)=Optimal_path_try(n,1);
%        Optimal_path(i,4)=Optimal_path_try(n,2);
       n=n-1;
   end
   num_op=size(Optimal_path,1)+1;
   Optimal_path(num_op,1)=Optimal_path_try(1,1);
   Optimal_path(num_op,2)=Optimal_path_try(1,2);
 %  plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'linewidth',1); %5%����
% ���������Ż�
 
 Optimal_path_two=Line_OPEN_STtwo(Optimal_path,CLOSED,Num_obs,num_op);
 num_optwo=size(Optimal_path_two,1)+1;
 Optimal_path_two(num_optwo,1)=xStart;
 Optimal_path_two(num_optwo,2)=yStart;
 % plot(Optimal_path_two(:,1)+.5,Optimal_path_two(:,2)+.5,'linewidth',1); %5%����
 % ��������
  j=num_optwo;
 Optimal_path_two2=[xStart yStart];
 for i=1:1:num_optwo
     Optimal_path_two2(i,1)=Optimal_path_two(j,1);
     Optimal_path_two2(i,2)=Optimal_path_two(j,2);
     j=j-1;
 end
  Optimal_path_three=Line_OPEN_STtwo(Optimal_path_two2,CLOSED,Num_obs,num_optwo);
  num_opthree=size(Optimal_path_three,1)+1;
  Optimal_path_three(num_opthree,1)=xTarget;
  Optimal_path_three(num_opthree,2)=yTarget;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
Plan_path=Optimal_path_three;
 
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

    




