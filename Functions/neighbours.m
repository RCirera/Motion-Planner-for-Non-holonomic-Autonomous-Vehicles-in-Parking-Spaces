function [neighbours]=neighbours(X,i,order)

% The function neighbours computes the neighbours of a node.
%
% The syntax is [neighbours]=neighbours(X,i,order).
%
% The function outputs are:
%   - neighbours: vector containing the IDs of the neighbours of the
%   current node.
%   
% The function arguments are:
%   - X: matrix containing the x coordinates of every node. The nodes are
%   organized in the matrix as in the plane.
%   - i: Integer, ID of the node for which the neighbours are being
%   determined.
%   - order: Integer, maximum amount of one-node steps that can be taken
%   between nodes that are still considered neighbours. Diagonal steps are
%   considered one-node steps.


%% Determine the neighbours

% Create a matrix with entries equal to its indices
numNodes=numel(X);
nodeVec=1:1:numNodes;
nodeMat=reshape(nodeVec,size(X));

% Find the row and column of the node with index i
[nRow,nCol]=find(nodeMat==i);

% Create a set of possible neighbors ans discard the neighbors with illegal 
% indices
nnRows = [max(1,nRow-order):1:min(size(nodeMat,1),nRow+order)];
nnCols = [max(1,nCol-order):1:min(size(nodeMat,2),nCol+order)];
neighbours=nodeMat(nnRows(1:end),nnCols(1:end));
neighbours=setdiff(neighbours,i);

end
