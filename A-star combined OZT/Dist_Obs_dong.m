function disttmp=Dist_Obs_dong(Obs_dong,xt,R_dong_obs)
   num_obs=size(Obs_dong,1);
   disttmp=200;
   for i=1:num_obs
       d = distance(Obs_dong(i,1),Obs_dong(i,2),xt(1,1),xt(2,1));
       if d<disttmp
           disttmp=d;
       end
   end
   
   if disttmp>=R_dong_obs
    disttmp=R_dong_obs;
   end
       
       
       
       
   