m1=100;m2=400;
k1=3e5;k2=1e5;
c1=1e3;c2=200;
 
open_system('OpdrachtDeel1Simulink')
names = find_system('OpdrachtDeel1Simulink')
set_param('OpdrachtDeel1Simulink/1//m1','Gain',num2str(1./m1))
set_param('OpdrachtDeel1Simulink/1//m2','Gain',num2str(1./m2))
set_param('OpdrachtDeel1Simulink/c1//m1','Gain',num2str(c1./m1))
set_param('OpdrachtDeel1Simulink/c2//m1','Gain',num2str(c2./m1))
set_param('OpdrachtDeel1Simulink/c2//m2','Gain',num2str(c2./m2))
set_param('OpdrachtDeel1Simulink/k1//m1','Gain',num2str(k1./m1))
set_param('OpdrachtDeel1Simulink/k2//m1','Gain',num2str(k2./m1))
set_param('OpdrachtDeel1Simulink/k2//m2','Gain',num2str(k2./m2))

[t,x,f]=sim('OpdrachtDeel1Simulink');
N = size(x,1);

omega = [0:(length(t)-1)]./max(t)*2*pi;
figure(2);plot(omega,db(fft(x(:,[1 2]))./(N./2)) );
xlabel('Pulsatie, in radialen/seconde');
ylabel('Amplitude, in db');
legend('x_1','x_2');
 
figure(1);plot(t,x(:,[1 2]));
xlabel('Tijd, in seconden')
ylabel('Verplaatsing, in m')
legend('x_1','x_2');
