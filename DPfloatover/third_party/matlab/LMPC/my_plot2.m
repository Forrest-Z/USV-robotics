
figure(1)
plot(Xout(:,1),Xout(:,2),'b',x_real(:,1),x_real(:,2),'r')
xlabel('$x$(m)','interpreter','latex')
ylabel('$y$(m)','interpreter','latex')
legend('Ŀ��켣','ʵ���켣')
axis equal

t=T:T:N*T;
figure(2)
subplot(2,1,1)
plot(t,Xout(:,1),'b',t,x_real(:,1),'r')
xlabel('$t$(s)','interpreter','latex')
ylabel('$x$(m)','interpreter','latex')
legend('Ŀ��켣','ʵ���켣')
subplot(2,1,2)
plot(t,Xout(:,2),'b',t,x_real(:,2),'r')
xlabel('$t$(s)','interpreter','latex')
ylabel('$y$(m)','interpreter','latex')
legend('Ŀ��켣','ʵ���켣')

figure(3)
subplot(2,1,1)
stairs(t,u_real(:,1))
xlabel('$t$(s)','interpreter','latex')
ylabel('$\tau_u$(N)','interpreter','latex')
subplot(2,1,2)
stairs(t,u_real(:,2))
xlabel('$t$(s)','interpreter','latex')
ylabel('$\tau_r$(Nm)','interpreter','latex')

figure(4)
subplot(2,1,1)
stairs(t,delta_u_real(:,1))
xlabel('$t$(s)','interpreter','latex')
ylabel('$\Delta\tau_u$(N)','interpreter','latex')
subplot(2,1,2)
stairs(t,delta_u_real(:,2))
xlabel('$t$(s)','interpreter','latex')
ylabel('$\Delta\tau_r$(Nm)','interpreter','latex')