clear all
close all
clc

x_tag=5;
y_tag=linspace(-3,3,5);
theda_tag=0;
xic=[0 0 0 0]';

for k=1:5
[path(:,:,k) p(k,:)]= my_path( x_tag, y_tag(k), theda_tag, xic );
end


%%  Velocity Ganarator %%
a0=0;a1=0;s0=0;
s1=max(p(:,4));
v0=6;v1=linspace(0,10,7);
for i=1:7
m=v1(i);
Q_ini=[s0 v0 a0]
Q_tag=[s1 m a1]
[ v(i,:) rho(i,:) ] = my_velocity( Q_ini,Q_tag );

    figure (1)
    hold on
    plot(v(i,:),'b');

end