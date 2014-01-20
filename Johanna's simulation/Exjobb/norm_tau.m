function norm_tau = norm_tau(TAU)
norm_tau = zeros(1,length(TAU));
for i=1:length(TAU)
    norm_tau(i)=norm(TAU(:,i));
end
    %}