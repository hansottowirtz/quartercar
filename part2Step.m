m1=100;m2=400;
k1=3e5;k2=1e5;
c1=1e3;c2=200;

open_system('OpdrachtDeel2SimulinkStep')
names = find_system('OpdrachtDeel2SimulinkStep')
set_param('OpdrachtDeel2SimulinkStep/1//m1','Gain',num2str(1./m1))
set_param('OpdrachtDeel2SimulinkStep/1//m2','Gain',num2str(1./m2))
set_param('OpdrachtDeel2SimulinkStep/c1//m1','Gain',num2str(c1./m1))
set_param('OpdrachtDeel2SimulinkStep/c2//m1','Gain',num2str(c2./m1))
set_param('OpdrachtDeel2SimulinkStep/c2//m2','Gain',num2str(c2./m2))
set_param('OpdrachtDeel2SimulinkStep/k1//m1','Gain',num2str(k1./m1))
set_param('OpdrachtDeel2SimulinkStep/k2//m1','Gain',num2str(k2./m1))
set_param('OpdrachtDeel2SimulinkStep/k2//m2','Gain',num2str(k2./m2))

[t,x,f]=sim('OpdrachtDeel2SimulinkStep');

plot(t,x(:,[2]));
hold on;
xlabel('Tijd, in seconden')
ylabel('Verplaatsing, in m')


addpath(fullfile(getenv('AME'),'scripting','matlab','amesim'));
cd '.\Amesim'
Parameters = amegetp('OpdrachtDeel2AmesimStep')
ameputp('OpdrachtDeel2AmesimStep','MAS001 instance 1 mass [kg]',num2str(m2))
ameputp('OpdrachtDeel2AmesimStep','MAS002 instance 1 mass [kg]',num2str(m1))
ameputp('OpdrachtDeel2AmesimStep','SD0000A-1 instance 1 spring rate [N/m]',num2str(k2))
ameputp('OpdrachtDeel2AmesimStep','SD0000A-1 instance 1 damper rating [N/(m/s)]',num2str(c2))
ameputp('OpdrachtDeel2AmesimStep','SD0000A-2 instance 1 spring rate [N/m]',num2str(k1))
ameputp('OpdrachtDeel2AmesimStep','SD0000A-2 instance 1 damper rating [N/(m/s)]',num2str(c1))
amerun('OpdrachtDeel2AmesimStep',0,10);
[R,S] = ameloadt('OpdrachtDeel2AmesimStep');
time  = amegetvar(R,S,'time [s]');
x_amesim = amegetvar(R,S,'MAS001_1 displacement port 1 [m]');
plot(time,-x_amesim,'--');
xlabel('Tijd, in seconden');
ylabel('Positie van de massa, in m')
legend('x_1','x_2');
hold off;