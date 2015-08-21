% Copyright (c) 2015 Niall McLaughlin, CSIT, Queen's University Belfast, UK
% Contact: nmclaughlin02@qub.ac.uk
% If you use this code please cite:
% "Enhancing Linear Programming with Motion Modeling for Multi-target Tracking",
% N McLaughlin, J Martinez Del Rincon, P Miller, 
% IEEE Winter Conference on Applications of Computer Vision (WACV), 2015 
% 
% This software is licensed for research and non-commercial use only.
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

%use the detections and linking graph to set up the constraints
function [costs,constraintsLeq,constraintsLegRHS,constraintsEq,constraintsEqRHS,lb,ub,linksOffset,exitOffset,entryOffset]...
    = buildConstraints(detections,nTotalEdges,linkGraph,linkIndexGraph,costs)

    %fill in the cost of the entry / exit nodes - all zero
    %cost(size(detections,1) + 1:size(detections,1) * 2) = 0;
    linksOffset = size(detections,1) * 3;
    exitOffset = size(detections,1) * 2;
    entryOffset = size(detections,1);
    nConstraint = nTotalEdges;

    constraintsLeq = sparse(size(detections,1)*2,nConstraint);
    constraintsLegRHS = zeros(1,size(detections,1)*2);
    lEqIndex = 1;

    constraintsEq = sparse(size(detections,1)*2 + 3,nConstraint);
    constraintsEqRHS = zeros(1,size(detections,1)*2 + 3);
    eqIndex = 1;

    %for each detection
    %entry link - link from source to detection
    %exit link - link from detection to sink
    for d = 1:size(detections,1)
    
        %get the enty and exit link indicies
        out = find(linkGraph(d,:) ~= -1);
        in = find(linkGraph(:,d) ~= -1);

        %add the costs for the links into and out of this detection
        if ~isempty(out)
            for edgeOut = out
                linkIndx = linkIndexGraph(d,edgeOut);
                costs(linksOffset + linkIndx) = linkGraph(d,edgeOut);
            end
        end

        if ~isempty(in)
            for edgeIn = in
                linkIndx = linkIndexGraph(edgeIn,d);
                costs(linksOffset + linkIndx) = linkGraph(edgeIn,d);
            end
        end

        %add constraints
        %flow conservation

        %exit link + inner <= 1
        constraint = zeros(1,nConstraint);
        constraint(d) = 1;
        constraint(exitOffset + d) = 1;
        constraintsLeq(lEqIndex,:) = constraint;
        constraintsLegRHS(lEqIndex) = 1;
        lEqIndex = lEqIndex + 1;

        %entry link + inner <= 1
        constraint = zeros(1,nConstraint);
        constraint(d) = 1;
        constraint(entryOffset + d) = 1;
        constraintsLeq(lEqIndex,:) = constraint;
        constraintsLegRHS(lEqIndex) = 1;
        lEqIndex = lEqIndex + 1;

        %entry link + inner == sum out
        if ~isempty(out)

            constraint = zeros(1,nConstraint);

            constraint(entryOffset + d) = 1;
            constraint(d) = 1;

            for edgeOut = out
                linkIndx = linkIndexGraph(d,edgeOut);
                constraint(linksOffset + linkIndx) = -1;
            end

            constraintsEq(eqIndex,:) = constraint;
            constraintsEqRHS(eqIndex) = 0;
            eqIndex = eqIndex + 1;
            
        else

            constraint = zeros(1,nConstraint);

            constraint(entryOffset + d) = 1;
            constraint(d) = 1;

            constraintsEq(eqIndex,:) = constraint;
            constraintsEqRHS(eqIndex) = 0;
            eqIndex = eqIndex + 1;

        end

        %exit link + inner == sum in
        if ~isempty(in)

            constraint = zeros(1,nConstraint);

            constraint(exitOffset + d) = 1;
            constraint(d) = 1;

            for edgeIn = in
                linkIndx = linkIndexGraph(edgeIn,d);
                constraint(linksOffset + linkIndx) = -1;
            end

            constraintsEq(eqIndex,:) = constraint;
            constraintsEqRHS(eqIndex) = 0;
            eqIndex = eqIndex + 1;
            
        else

            constraint = zeros(1,nConstraint);

            constraint(exitOffset + d) = 1;
            constraint(d) = 1;

            constraintsEq(eqIndex,:) = constraint;
            constraintsEqRHS(eqIndex) = 0;
            eqIndex = eqIndex + 1;
            
        end

    end

    %upper bound and lower bounds for all links
    %constrain all edges to be less >= 0 && <= 1
    lb = zeros(1,nConstraint);
    ub = ones(1,nConstraint);

    %Flow constraints - we should search for the best flow to find the optimal
    %tracking solution

    %flow in must equal flow out
    %flow from detections to sink == flow
    constraint = zeros(1,nConstraint);
    for d = 1:size(detections,1)
        constraint(exitOffset + d) = 1;
        constraint(entryOffset + d) = -1;
    end
    constraintsEq(eqIndex,:) = constraint;
    constraintsEqRHS(eqIndex) = 0;
    eqIndex = eqIndex + 1;

    %flow from source to detections == number of tracklets (roughly == people)
    flow = -1;
    constraint = zeros(1,nConstraint);
    for d = 1:size(detections,1)
        constraint(entryOffset + d) = 1;
    end
    constraintsEq(eqIndex,:) = constraint;
    constraintsEqRHS(eqIndex) = flow;
    eqIndex = eqIndex + 1;

    %flow from detections to sink == flow
    constraint = zeros(1,nConstraint);
    for d = 1:size(detections,1)
        constraint(exitOffset + d) = 1;
    end
    constraintsEq(eqIndex,:) = constraint;
    constraintsEqRHS(eqIndex) = flow;
    eqIndex = eqIndex + 1;
    
end