function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

    global GAMMA R P_WIND
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K TERMINAL_STATE_INDEX

    % K     : # of states
    % GAMMA : shooter gamma factor
    % R     : shooter range

    % bounds of the map
    % bounds(1) = M ; bounds(2) = N ; 
    bounds = size(map);

    % Intented movement of inputs
    %         NORTH   SOUTH    EAST    WEST     HOVER
    inputs = [[0 1] ; [0 -1] ; [1 0] ; [-1 0] ; [0 0]];

    % Matrix of Probability of being shot on every cell.
    % For the step 4 of evoluation of the system. 
    P_BeingShot = ComputeProbabilityOfBeingShot(map);

    % Position of trees
    [trees_m, trees_n] = ind2sub(size(map), find(map==TREE));
    trees = [trees_m trees_n];

    % Position of the base station
    % Index of the base state 
    % The drone ends up at this state if it crashes. 
    [base_m, base_n] = ind2sub(size(map), find(map==BASE));
    baseIndex = find(ismember(stateSpace, [base_m,base_n,0],'rows'));

    % The state [pickUp_m,pickUp_n,0] will never be visited. 
    % If the previous state does not have package (0) and comes to the pickup 
    % cell in the same time step, the package will be obtained (1). 

    % Probability Matrix inilization
    P = zeros(K,K,size(inputs,1));
    % From terminal state the only possible transition 
    P(TERMINAL_STATE_INDEX, TERMINAL_STATE_INDEX, :) = ones(1,length(inputs));

    % The entry P(i, j, l) represents the transition probability
    % from state i to state j if control input l is applied.
    % There are maximum 6 possible states that the drone might arrive.
    % Intended cell, cells at NORTH, SOUTH, EAST, WEST of the intended cell and
    % the basestation if it crashed. 
    for l = 1:size(inputs,1)
        for i = 1:K

            current_cell = stateSpace(i,1:2);
            intended_cell= current_cell + inputs(l,:);
            inputValid   = CheckInputValidity(intended_cell, trees, bounds);

            % Do the calculation only if the input is valid and the current
            % state is not the terminal state
            if inputValid && i ~= TERMINAL_STATE_INDEX 

                % possibleCells except the intended cell
                possibleCells = FindPossibleCells(intended_cell, inputs, trees, bounds);
                numberOfPossibleCells = size(possibleCells,1);

                % Movements that might result crash due to wind    
                numberOfCrashMovements = 4 - numberOfPossibleCells;

                % Add the intended cell to the possibleCells in order to 
                % calculate possibleStates 
                possibleCells = [intended_cell ; possibleCells];                                            
                possibleStates = FindPossibleStates(stateSpace(i,3),possibleCells, map, stateSpace); 

                % Calculate Probabilities.
                % p_intended = 1 - p_wind - (1 - p_wind)*p_beingShot
                p_crash = (1-P_WIND)*P_BeingShot(intended_cell(1), intended_cell(2));
                P(i,possibleStates(1),l) = 1 - P_WIND - p_crash;

                % p_possible = p_wind*0.25 - (p_wind*0.25)*p_beingShot
                for p = 1:numberOfPossibleCells
                    p_shot = P_WIND*0.25*P_BeingShot(possibleCells(p+1));
                    P(i,possibleStates(p+1),l) =  P_WIND*0.25 - p_shot;
                    p_crash = p_crash + p_shot;
                end

                % p_base = sum(p_beingShot)+ numberOfCrashMovements*p_wind*0.25
                p_crash = p_crash + numberOfCrashMovements*P_WIND*0.25;

                % The base cell might have been visited 
                P(i,baseIndex,l) = P(i,baseIndex,l) + p_crash;  

            end 
        end
    end

end

%%
function possibleStates = FindPossibleStates(package, possibleCells, map, stateSpace)

% FindPossibleStates finds the possible states that the drone might end up.

%   Input arguments:
%       possibleCells:
%           A (F x 2) marix containing every possible cells that the drone  
%           might arrive. The first cell is the intended cell.
%           (f, 1) = possibleCell_m ;(f, 2) = possibleCell_n
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       possibleStates:
%           A (1 x S) marix containing index of every possible states  
%           that the drone might end up.

global PICK_UP
possibleStates = zeros(1,size(possibleCells,1));
possibleStates_packages = zeros(size(possibleCells,1),1) + package;

% Only at pick-up station, the state transition should be [. . 0] --> [. . 1]
if package == 0
    [pickUp_m, pickUp_n] = ind2sub(size(map), find(map==PICK_UP));
    pickUpIndex = find(ismember(possibleCells, [pickUp_m,pickUp_n],'rows'));
    possibleStates_packages(pickUpIndex) = 1;
end

for s = 1:size(possibleCells,1)
    possibleState = [possibleCells(s,1),possibleCells(s,2),possibleStates_packages(s)];
    possibleStates(s) = find(ismember(stateSpace, possibleState, 'rows'));
end


end

%%
function possibleCells = FindPossibleCells(intended_cell, inputs, trees, bounds)
% FindPossibleCells finds the possible cells that the drone might arrive.

%   Input arguments:
%       intended_cell:
%           A (1 x 2) array containing the position of the intended cell on
%           the map
%
%       inputs: 
%            A (1 x L) array containing the position changes by the inputs
%
%       trees:
%           A (t x 2) matrix containing the position of the trees on the
%           map. trees(i,1) = tree_i_m, trees(i,1) = tree_i_n
%
%       bounds:
%           A (1 x 2) array containing the bounds of the map in the form of
%           [bound_M, bound_N]
%
%   Output arguments:
%       possibleCells:
%           A (F x 2) marix containing every possible cells that the drone  
%           might arrive. (except the intended cell)
%           (f, 1) = possibleCell_m ;(f, 2) = possibleCell_n

possibleCells = [];
% Effect of the wind a.k.a possible cells except intended cell.
for i = 1:4
    
    possibleCell = intended_cell + inputs(i,:);
    if CheckInputValidity(possibleCell, trees, bounds)
        possibleCells = [possibleCells; possibleCell];
    end 

end

end

%%
function  inputValid = CheckInputValidity(intended_cell, trees, bounds)
% CheckInputValidity checks the validity of the input. The input is not
% valid if the intended cell is out of the bounds of the worlds and if it
% contains a tree on it.

%   Input arguments:
%       intended_cell:
%           A (1 x 2) array containing the position of the intended cell on
%           the map

%       trees:
%           A (t x 2) matrix containing the position of the trees on the
%           map. trees(i,1) = tree_i_m, trees(i,1) = tree_i_n

%       bounds:
%           A (1 x 2) array containing the bounds of the map in the form of
%           [bound_M, bound_N]

%   Output arguments:
%       inputValidity:
%           A (M x N)-matrix containing the probabilities of being shot at
%           a cell. 


% check if the intended cell is inside of the bounds of the world 
bound_upper = intended_cell > bounds;
bound_lower = intended_cell < [1 1];

% check if the intended cell does not contain a tree
isOnTree = ismember(trees, intended_cell,'rows');

if any(bound_upper) || any(bound_lower) || any(isOnTree)
    inputValid = false;
else
    inputValid = true;
end

    

end

%%
function P_BeingShot = ComputeProbabilityOfBeingShot(map)

% ComputeProbabilityOfBeingShot computes the probabiity of being shot by the
% specified shooter at the specified cell.

%   Input arguments:
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE

%   Output arguments:
%       P_BeingShot:
%           A (M x N)-matrix containing the probabilities of being shot at
%           a cell. 

global SHOOTER

% Size of the map
[map_M, map_N] = size(map);
% Position of shooters
[shooter_M,shooter_N] = ind2sub(size(map), find(map==SHOOTER));
numberOfShooters = length(shooter_M);


P_BeingShot = zeros(map_M, map_N);
for m = 1:map_M
    for n = 1:map_N
        for s = 1:numberOfShooters
            P_BeingShot(m,n) = P_BeingShot(m,n)+ ...
                ProbabilityOfBeingShot([m,n],[shooter_M(s), shooter_N(s)]);               
        end
    end
end

end

%%
function p_BeingShot = ProbabilityOfBeingShot(pos, pos_Shooter)
% ProbabilityOfBeingShot computes the probabiity of being shot by the
% specified shooter at the specified cell.

%   Input arguments:
%       pos:
%           Position of the cell
%
%       pos_Shooter:
%           Position of the specific shooter
%
%   Output arguments:
%       p_shot:
%           A float number of the probability

global GAMMA R
% GAMMA : shooter gamma factor
% R     : shooter range

d = abs(pos(1)-pos_Shooter(1)) + abs(pos(2)-pos_Shooter(2));

if(d > R )
    p_BeingShot = 0;
else
    p_BeingShot = GAMMA / (d + 1);
end

end
