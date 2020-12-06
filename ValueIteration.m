function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ? --> Iterate over states except the terminal state
global TERMINAL_STATE_INDEX

% initilizations
J_opt       = zeros(K,1);
costJnew    = zeros(K,1);
u_opt_ind   = zeros(1,K);

% inf causes problems
G(G==inf) = 10e10;

% Iterate over states except the terminal state
statesIndex = 1:K;
statesIndex(TERMINAL_STATE_INDEX) = [];

% Termination criteria
% Jk(i)-Jk+1(i) < termination_threshold 
termination_threshold = 10e-5;
costDiff = 1;

while(costDiff >= termination_threshold)
    for s=1:length(statesIndex)
        i = statesIndex(s);
        [costJnew(i), u_opt_ind(i)] = min( G(i,:) + J_opt'*squeeze(P(i,:,:)));
    end
    costDiff = norm(costJnew - J_opt);
    J_opt = costJnew;    
end 

% The final touch
u_opt_ind(TERMINAL_STATE_INDEX) = HOVER;
u_opt_ind  = u_opt_ind.';

end