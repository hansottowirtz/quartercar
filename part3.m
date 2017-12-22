k2=1e5;count=1;D=[];k_2=[];
x=0;
while k2<1e6
m1=100;m2=400;
k1=3e5;
c1=1e3;c2=200;

open_system('OpdrachtDeel2SimulinkSine')
names = find_system('OpdrachtDeel2SimulinkSine');
set_param('OpdrachtDeel2SimulinkSine/1//m1','Gain',num2str(1./m1))
set_param('OpdrachtDeel2SimulinkSine/1//m2','Gain',num2str(1./m2))
set_param('OpdrachtDeel2SimulinkSine/c1//m1','Gain',num2str(c1./m1))
set_param('OpdrachtDeel2SimulinkSine/c2//m1','Gain',num2str(c2./m1))
set_param('OpdrachtDeel2SimulinkSine/c2//m2','Gain',num2str(c2./m2))
set_param('OpdrachtDeel2SimulinkSine/k1//m1','Gain',num2str(k1./m1))
set_param('OpdrachtDeel2SimulinkSine/k2//m1','Gain',num2str(k2./m1))
set_param('OpdrachtDeel2SimulinkSine/k2//m2','Gain',num2str(k2./m2))
[t,x,a]=sim('OpdrachtDeel2SimulinkSine');

N = size(x,1);
omega = (0:(length(t)-1))./max(t)*2*pi;

D(count)=sqrt((1/length(t))*sum(a.^2));
k2=k2+10000;
k_2(count)=k2;
count=count+1;
end
plot(k_2,D);
xlabel('stijfheid ks'); 
ylabel('comfort');
Comfort=D(1)

