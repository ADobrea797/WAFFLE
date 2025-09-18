function [X_samples]=MCMC_fit(Data,indices,circuitstring,parameter_bounds,Nmc,likelihood,circuitparameters)

Nbi=floor(Nmc/3);
Nr=Nmc-Nbi;
freq=Data(indices,1);
M=length(indices);

Y=Data(indices,2:3);
N=size(parameter_bounds,2);
if likelihood=='G'
X_samples=zeros(N+1,Nr);
end

if isempty(circuitparameters)
    circuitparameters=mean(parameter_bounds,1);
end

Z=computecircuit(circuitparameters,circuitstring,freq);

sigma2=mean(mean((Y-Z).^2));

X=[circuitparameters sigma2]';
s=0.1*abs(log(circuitparameters));
V=rand(N,100)<0.5;

u=1;
for m_compt = 1:Nmc
    if mod(m_compt,50)==0
        m_compt
    end
            accept=zeros(N,1);
    for n=1:N
        x=log(X(n));
        x_cand=exp(x+sqrt(s(n))*randn);
        if x_cand>parameter_bounds(1,n) && x_cand<parameter_bounds(2,n)
            X_cand=X;
            X_cand(n)=x_cand;
            Z_cand=computecircuit(X_cand(1:end-1)',circuitstring,freq);
            rho=-0.5*(sum(sum((Y-Z_cand).^2))-sum(sum((Y-Z).^2)));
            if rand<exp(rho)
                X=X_cand;
                Z=Z_cand;
                accept(n)=1;
              
            end

        end
    end
    V=[V(:,2:end) accept];
    
    
    [s V]=update_s(V,s);

    if likelihood=='G'
        sigma2=min(1/gamrnd(M/2+0.01,1./(sum(sum((Y-Z).^2))+0.01)),1e8);
        X(end)=sigma2;
    end

    if m_compt>Nbi
        X_samples(:,u)=X;
        u=u+1;


    if mod(m_compt,200)==0
        figure(10)
for i=1:N+1
subplot(N+1,1,i)
plot(X_samples(i,1:u-1))
end
pause(0.01)
    end
    end



end 





