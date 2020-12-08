function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration? --> Iterate over states except the terminal state
global TERMINAL_STATE_INDEX

% initilizations
J_opt       = zeros(K,1);
costJnew    = zeros(K,1);
u_opt_ind   = 5*ones(1,K);
J_opt_previous = ones(K,1);

% Iterate over states except the terminal state
statesIndex = 1:K;
statesIndex(TERMINAL_STATE_INDEX) = [];

% Termination criteria
% Iterate between Stage 1 and 2 until Jµh+1(i) = Jµh(i)
iterations=0;
while(~isequal(J_opt_previous,J_opt))

    J_opt_previous = J_opt;  
    
    for s=1:length(statesIndex)
        i = statesIndex(s);
        P2(i,:) = P(i,:,u_opt_ind(i));
        G2(i,1) = G(i,u_opt_ind(i));
    end
    J_opt=(eye(size(J_opt,1))-P2)\G2;
    
    for s=1:length(statesIndex)
        i = statesIndex(s);
        [costJnew(i), u_opt_ind(i)] = min( G(i,:) + J_opt'*squeeze(P(i,:,:)) );
    end

    iterations=iterations+1;
end 

% The final touch
u_opt_ind(TERMINAL_STATE_INDEX) = HOVER;
u_opt_ind  = u_opt_ind.';
fprintf('    Number of iterations: %f\n',iterations);


end
