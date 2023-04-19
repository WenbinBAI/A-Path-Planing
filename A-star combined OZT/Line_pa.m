function pa=Line_pa(x2,y2,x3,y3)
   % line_node=LINE(p_node_2,p_node_3)
%    x2=p_node_2(1,1);
%    y2=p_node_2(1,2);
%    x3=p_node_3(1,1);
%    y3=p_node_3(1,2);
   
   line_node=[x2 y2];
  a=2;
   
   
 if abs(x2-x3)<2 && abs(y2-y3)<2
     pa=[x3 y3];
 else
   if x3==x2&&y3~=y2
       w=1; %%垂直的直线
   elseif x3~=x2&&y3==y2
       w=2;%水平直线
   else
       w=3;% 斜线
   end
    
   switch w
       case 1
           if y3>y2
                n=a;
           else 
                n=-a;
           end
           j=0;
           for i=(y2-n):n:y3
               j=j+1;
               line_node(j,1)=x3;
               line_node(j,2)=i;
           end
       case 2
           if x3>x2
                n=a;
           else
                n=-a;
           end
           j=0;
           for i=(x2-n):n:x3
               j=j+1;
               line_node(j,1)=i;
               line_node(j,2)=y3;
           end
       case 3
           if x3>x2
               n=a;
           else
               n=-a;
           end
           j=0; k = (y3-y2)/(x3-x2); b =y2-k*x2;
           for i=(x2-n):n:x3%(x2-2*n)
               j=j+1;
               line_node(j,1)=i;
               line_node(j,2)=k*i+b;
           end
           
   end
   pa=line_node;
 end
   