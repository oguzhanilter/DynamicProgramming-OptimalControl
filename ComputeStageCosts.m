function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    % bounds of the map
    bounds = size(map);
        
    % Intented movement of inputs
    %         NORTH   SOUTH    EAST    WEST     HOVER
    inputs = [[0 1] ; [0 -1] ; [1 0] ; [-1 0] ; [0 0]];
    
    p_windEffect = 0.25;

    % Matrix of Probability of being shot on every cell.
    P_BeingShot = ComputeProbabilityOfBeingShot(map);
      
    % Position of trees
    [trees_m, trees_n] = ind2sub(size(map), find(map==TREE));
    trees = [trees_m trees_n];
    
    % From terminal state the only possible transition 
    G = Inf(K,size(inputs,1));
    % From terminal state every cost is 0
    G(TERMINAL_STATE_INDEX, :) = zeros(1,length(inputs));
    
    states = 1:K;
    states(TERMINAL_STATE_INDEX) = [];
    for s = 1:length(states)
        i = states(s);
        for l = 1:size(inputs,1)
            
            current_cell = stateSpace(i,1:2);
            intended_cell= current_cell + inputs(l,:);
            inputValid   = CheckInputValidity(intended_cell, trees, bounds);
            
            if(inputValid)
                
                G(i,l) = 1;
                
                % possibleCells except the intended cell
                possibleCells = FindPossibleCells(intended_cell, inputs, trees, bounds);
                numberOfPossibleCells = size(possibleCells,1);

                % Movements that might result crash due to wind    
                numberOfCrashMovements = 4 - numberOfPossibleCells;
                
                % There is -1 --> (NC-1) because if it gets shots the total
                % cost is 10 but at the beginning we set G(i,l) = 1 we need
                % to subtract it. 
                
                for p = 1:numberOfPossibleCells
                    cell = possibleCells(p,:);
                    p_shot = p_windEffect*P_WIND*P_BeingShot(cell(1),cell(2));
                    G(i,l) = G(i,l) +  (Nc-1)*p_shot;
                end
                
                G(i,l) = G(i,l) .... 
                + (1-P_WIND)*(Nc-1)*P_BeingShot(intended_cell(1), intended_cell(2)) ...
                + numberOfCrashMovements*(Nc - 1)*P_WIND*p_windEffect;
       
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

%d = abs(pos(1)-pos_Shooter(1)) + abs(pos(2)-pos_Shooter(2));
d = norm(pos-pos_Shooter,1);


if(d > R )
    p_BeingShot = 0;
else
    p_BeingShot = GAMMA / (d + 1);
end

end


