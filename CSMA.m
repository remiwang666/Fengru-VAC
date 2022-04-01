function loss = CSMA(n,q,m,W,init_pc,init_pin,N)

t=1:1:N;
pcL=zeros(1,N);
pinL=zeros(1,N);

pcL(1)=init_pc;
pinL(1)=init_pin;

for i=2:N
    pin_=(1+q*(1-pcL(i-1)^m)/(1-pcL(i-1))*(1+(W-1)/2/(1-pcL(i-1))))^(-1);
    pc_=1-(1-q*pin_*(1-pcL(i-1)^m)/(1-pcL(i-1)))^(n-1);
    pcL(i)=pc_;
    pinL(i)=pin_;
end
%{
plot(t,pcL,'r');
hold on;
plot(t,pinL,'b');
grid on
legend('pc','pin')
hold off;
%}
limit_pc=pcL(end);
limit_pin=pinL(end);
loss=limit_pc^m;