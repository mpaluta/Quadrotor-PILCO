function mu0 = randomize_mu(mu_max)

mu0=zeros(length(mu_max),1);
for i =1:length(mu_max)
    bound = mu_max(i);
    mu0(i) = -bound + 2*bound*rand();
end

end