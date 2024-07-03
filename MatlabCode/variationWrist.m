%% Function to evaluate the wirst variation
function [W_error] = variationWrist(W_new,W_old,E_new,E_old)
    W_error = 200*abs(W_new-W_old) + 50*abs(E_new-E_old);
end