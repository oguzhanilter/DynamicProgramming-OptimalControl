function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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
% iteration ? --> % Iterate over states except the terminal state
global TERMINAL_STATE_INDEX

% initilizations
f           = -1*ones(K-1,1);
u_opt_ind   = ones(1,K-1);

% inf causes problems
G(G==inf) = 10e10;

% Exclude the terminal state
G(TERMINAL_STATE_INDEX,:) = [];
P(TERMINAL_STATE_INDEX,:,:) = [];
P(:,TERMINAL_STATE_INDEX,:) = [];

% Transform the problem to standart form of linear programming
I = eye(K-1); A = []; b = [];
for i = 1:size(P,3)
    A = [A; I - P(:,:,i)];
    b = [b; G(:,i)];
end

% MAGIC!
J_opt = linprog(f,A,b);

% The same algorithm from PI and LP to find optimal input
for i=1:K-1
    [~, u_opt_ind(i)] = min( G(i,:) + J_opt'*squeeze(P(i,:,:)) );
end

% Add the terminal state to J and U
J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1, :) ; 0 ; J_opt(TERMINAL_STATE_INDEX:end, :) ];
u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX-1)  HOVER  u_opt_ind(TERMINAL_STATE_INDEX:end) ];

% The final touch
u_opt_ind = u_opt_ind';

end

