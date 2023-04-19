
clc
clear all

a=[1,5,7,3,2,1,6,9];%L1=8 L2=5  L1-L2+1=4
b=[2,4,6,1,3];
c=conv(a,b)
d=cconv(a,b,13)
