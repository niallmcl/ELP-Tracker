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


% Minimum cost network-flow tracker
% 
% This code is for a modified version of the tracker descibed in
% Enhancing Linear Programming with Motion Modeling for Multi-target Tracking
% N McLaughlin, J Martinez Del Rincon, P Miller - (WACV), 2015
% 
% The tracking solution is computed over several iterations. 
% In the first iteration we link detections based on distance, time and appearance.
% In later iterations we link tracklets based on motion cost, assuming a linear motion model.

function trackerOutputCSV = doTracking(seqData,trackerConf)

    % get detections into desired format (we adjust input cols from MOT chall. slightly)
    % the detections matrix holds a detection in each row with the format:
    % frame-num, detection-confidence, x, y, w, h,   
    detections = csvread(seqData.inputDetections);
    detections(:,2) = detections(:,7);
    detections(:,2) = detections(:,2) ./ max(detections(:,2));
    detections = detections(:,1:6);   
    detections(:,3:6) = round(detections(:,3:6));
    detections = [detections zeros(size(detections,1),1)];
    
    %for each detection, extract its appearance model for later use
    appearanceModels = getDetsAppearanceModels(detections,seqData.inputVideoFile);
    
    %use NMS to remove detections overlapped by >= 0.5
    if trackerConf.doDetectionPreProcessing
        keepAfterNMS =  nmsFilterDetections(detections,trackerConf.nmsThreshold);
        detections = detections(keepAfterNMS == 1,:);           
        appearanceModels = appearanceModels(keepAfterNMS == 1,:);
    end

    tracklets = {};
    startPoints = [];
    endPoints = [];
    trackerOutputCSV = [];
    globalTrackNumber = 1;
    
    % we can split long sequences into windows by changing the windowLen,
    % but the results are better if we just process all frames at once
    windowLen = seqData.seqLen;    
    for startFrame = 1:windowLen:seqData.seqLen
                
        endFrame = startFrame + windowLen-1;
        disp(['endFrame:' int2str(endFrame)]);
        
        for iters = 1:trackerConf.maxIters
                                   
            %On the first iteration build a simple linking graph using only unitary costs to
            %associate detections. On subsequent iterations, extend the existing graph
            %to include new linking costs based on tracklet motion
            maxFrameGapBetweenTracklets = trackerConf.maxFrameJumpMultiplier * (iters-1);
            
            if iters == 1
                %in the first iteration - link inidividual detections
                disp('Building link graph - simple');
                [linkGraph,linkIndexGraph,nTotalLinks,detectionsSorted,appearanceModelsSorted] = ...
                    buildDetectionsGraphSimple(detections,startFrame,endFrame,trackerConf.maxFrameJump,trackerConf.heightsPerSec,trackerConf.appearanceThreshold,seqData.seqFPS,appearanceModels);
            else
                %in later iterations - link tracklets over greater distances
                disp('Building link graph - tracklets');
                [linkGraph,linkIndexGraph,nTotalLinks,detectionsSorted] = ...
                    buildDetectionsGraphTracklets(linkGraph,linkIndexGraph,detectionsSorted,tracklets,startPoints,endPoints,startFrame,endFrame,maxFrameGapBetweenTracklets,trackerConf.linkEnergyThreshold,trackerConf.heightsPerSec,seqData.seqFPS,appearanceModelsSorted);
            end
                        
            %costs is a vector holding costs in the following order [detections entry exit links]
            nTotalEdges = size(detectionsSorted,1)*3 + nTotalLinks;
            costs = zeros(1,nTotalEdges);
            %add the cost of detections, which are negative and propto the detector confidence
            for i = 1:size(detectionsSorted,1)
                costs(i) = -detectionsSorted(i,2);
            end
            
            %use the linkGraph to build matrices that enforce tracking
            %constraints related to number of edges connecting with each
            %detection node
            disp('Building Constraint Matrices');
            [costs,constraintsLeq,constraintsLegRHS,constraintsEq,constraintsEqRHS,lb,ub,linksOffset,exitOffset,entryOffset] = ...
                buildConstraints(detectionsSorted,nTotalEdges,linkGraph,linkIndexGraph,costs);
            
            %find tracking solution using network flow, given the
            %constraint matrices and cost vector from above
            disp('Optimising Network Flow');
            maxFlow = 401;
            linearProg = @(flow) minFunc(flow,costs,constraintsLeq,constraintsLegRHS,constraintsEq,constraintsEqRHS,lb,ub);
            [minFlow,x,nEvals,solnRecord] = fibSearch(linearProg,maxFlow,1);
            
            %using the min-cost network flow solution contained in 'x',
            %recover the tracklets for use in later iterations
            disp('Recovering Tracklets');
            [tracklets,startPoints,endPoints] = recoverTracklets(x,entryOffset,exitOffset,linksOffset,linkGraph,linkIndexGraph);
        end
        
        % remove very short trackets as these may be false positives
        if trackerConf.doFilterTracklets            
            shortTracklets = zeros(1,numel(tracklets));
            for t = 1:numel(tracklets)
                if (tracklets{t}(end) - tracklets{t}(1)) < seqData.seqFPS
                    shortTracklets(t) = 1;
                end
            end
            tracklets = tracklets(shortTracklets==0);            
        end
        
        [csvFrame,globalTrackNumber] = buildTrackerOutput(tracklets,detectionsSorted,globalTrackNumber,seqData.seqFPS);
        trackerOutputCSV = [trackerOutputCSV; csvFrame];
        
    end
    csvwrite(seqData.CSVOutputFile,trackerOutputCSV);    
end