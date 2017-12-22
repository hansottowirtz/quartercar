m1=100;m2=400;
k1=3e5;k2=1e5;
c1=1e3;c2=200;

open_system('OpdrachtDeel2SimulinkRandom')
names = find_system('OpdrachtDeel2SimulinkRandom')
set_param('OpdrachtDeel2SimulinkRandom/1//m1','Gain',num2str(1./m1))
set_param('OpdrachtDeel2SimulinkRandom/1//m2','Gain',num2str(1./m2))
set_param('OpdrachtDeel2SimulinkRandom/c1//m1','Gain',num2str(c1./m1))
set_param('OpdrachtDeel2SimulinkRandom/c2//m1','Gain',num2str(c2./m1))
set_param('OpdrachtDeel2SimulinkRandom/c2//m2','Gain',num2str(c2./m2))
set_param('OpdrachtDeel2SimulinkRandom/k1//m1','Gain',num2str(k1./m1))
set_param('OpdrachtDeel2SimulinkRandom/k2//m1','Gain',num2str(k2./m1))
set_param('OpdrachtDeel2SimulinkRandom/k2//m2','Gain',num2str(k2./m2))

[t,x,f]=sim('OpdrachtDeel2SimulinkRandom');

plot(t,x(:,[2]));
hold on;
xlabel('Tijd, in seconden')
ylabel('Verplaatsing, in m')


addpath(fullfile(getenv('AME'),'scripting','matlab','amesim'));
cd '.\Amesim'
Parameters = amegetp('OpdrachtDeel2AmesimRandom')
ameputp('OpdrachtDeel2AmesimRandom','MAS001 instance 1 mass [kg]',num2str(m2))
ameputp('OpdrachtDeel2AmesimRandom','MAS002 instance 1 mass [kg]',num2str(m1))
ameputp('OpdrachtDeel2AmesimRandom','SD0000A-1 instance 1 spring rate [N/m]',num2str(k2))
ameputp('OpdrachtDeel2AmesimRandom','SD0000A-1 instance 1 damper rating [N/(m/s)]',num2str(c2))
ameputp('OpdrachtDeel2AmesimRandom','SD0000A-2 instance 1 spring rate [N/m]',num2str(k1))
ameputp('OpdrachtDeel2AmesimRandom','SD0000A-2 instance 1 damper rating [N/(m/s)]',num2str(c1))
amerun('OpdrachtDeel2AmesimRandom',0,100);
[R,S] = ameloadt('OpdrachtDeel2AmesimRandom');
time  = amegetvar(R,S,'time [s]');
x_amesim = amegetvar(R,S,'MAS001_1 displacement port 1 [m]');
plot(time,-x_amesim,'--');
xlabel('Tijd, in seconden');
ylabel('Positie van de massa, in m')
legend('x_1','x_2');
hold off;