function [X_samples]=MCMC_fit(Data,indices,circuitstring,parameter_bounds,Nmc,circuitparameters)

Nbi=floor(Nmc/3); % Number of burn-in terations of the MCMC sampler
Nr=Nmc-Nbi; % Number of iterations after burn-in
freq=Data(indices,1); %Input frequencies
Nfreq=length(indices); % Number of input frequencies

Y=Data(indices,2:3);
N=size(parameter_bounds,2);
X_samples=zeros(N+1,Nr); %Initialise matrix for the Markov chain (N parameters + noise variance: N+1 parameters in total)

if isempty(circuitparameters)
    circuitparameters=mean(parameter_bounds,1); % In case the parameters are not initialized
end

Z=computecircuit(circuitparameters,circuitstring,freq); % References computecircuit function by Jean-Luc Dellis (2025). Zfit (https://uk.mathworks.com/matlabcentral/fileexchange/19460-zfit), MATLAB Central File Exchange.

sigma2=mean(mean((Y-Z).^2)); % Compute initial MSE (i.e., estimated noise variance)

X=[circuitparameters sigma2]';
s=0.1*abs(log(circuitparameters)); % variance of proposal distribution for the Metropolis-Hastings steps
V=rand(N,100)<0.5; % Sliding window for the monitoring the acceptance rate and tune the variance of the Gaussian proposals

u=1;
for m_compt = 1:Nmc
    if mod(m_compt,50)==0
        m_compt
    end
    accept=zeros(N,1); % Reset the acceptance flags for the new iteration
    for n=1:N
        % Gaussian proposal (in log-space)
        x=log(X(n));
        x_cand=exp(x+sqrt(s(n))*randn);
        if x_cand>parameter_bounds(1,n) && x_cand<parameter_bounds(2,n)
            X_cand=X;
            X_cand(n)=x_cand;
            Z_cand=computecircuit(X_cand(1:end-1)',circuitstring,freq);
            rho=-0.5*(sum(sum((Y-Z_cand).^2))-sum(sum((Y-Z).^2))); %Local acceptance ratio. 
            % We implicitely assume a uniform prior for the log-parameters 
            % and the prior terms do not appear in the acceptance ratio.  
            if rand<exp(rho) % If sample accepted
                X=X_cand;
                Z=Z_cand;
                accept(n)=1;
              
            end

        end
    end
    V=[V(:,2:end) accept]; % Update sliding window of the acceptance flags
    
    
    [s V]=update_s(V,s); % Udjust proposal of the variance (should in principle be only done during burn-in)
    

    sigma2=min(1/gamrnd(Nfreq/2+0.01,1./(sum(sum((Y-Z).^2))+0.01)),1e8);
    % Sample noise variance from inverse-gamma conditional distribution
    X(end)=sigma2;

    if m_compt>Nbi
        X_samples(:,u)=X;
        u=u+1;

        
        if mod(m_compt,200)==0 % Interim display for chain monitoring
            figure(10)
            for i=1:N+1
                subplot(N+1,1,i)
                plot(X_samples(i,1:u-1))
            end
            pause(0.01)
        end
    end

end 





