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


%return a linking graph - indicates which detections are linked together
%linkGraph(i,j) = cost of linking detection i with detection j
%linkIndexGraph = index refering to the edge between detections i and j -
%                 zero if no link is possible
%nTotalLinks = total number of linkso

function [linkGraph,linkIndexGraph,nTotalLinks,detections] = buildDetectionsGraphTracklets( ...
    linkGraph,linkIndexGraph,detections,tracklets,startPoints,endPoints,startFrame,endFrame,maxFrameGapBetweenTracklets,linkEnergyThreshold,heightsPerSec,fps,appModels)

    nTotalDetections = size(detections,1);
    prevFrameDetections = [];
    nPrevFrameDetections = 0;
    
    %This structure holds the tracklet associated with detection in
    %tracklets
    sourceTracklets = zeros(1,nTotalDetections);
    sourceInds = zeros(1,nTotalDetections);
    for t = 1:numel(tracklets)
        thisTracklet = tracklets{t};
        sourceTracklets(thisTracklet) = t;
        sourceInds(thisTracklet) = 1:numel(thisTracklet);
    end
    
    linkIndex = max(linkIndexGraph(:));
    
    for f = startFrame:endFrame

        thisFrameDetections = detections(find(detections(:,1) == f),:);
        nThisFrameDetections = size(thisFrameDetections,1);

        if nPrevFrameDetections > 0
            
            %find the distances
            for i = 1:nPrevFrameDetections
                isStartPoint = sum((startPoints - prevFrameDetections(i,8)) == 0);
                
                for j = 1:nThisFrameDetections
                    
                    %is this a startPoint or endPoint                    
                    isEndPoint = sum((endPoints - thisFrameDetections(j,8)) == 0);
                                        
                    t = thisFrameDetections(j,1) - prevFrameDetections(i,1);
                    
                    %check if these detections are members of tracklets
                    trackletPrev = sourceTracklets(prevFrameDetections(i,8));
                    trackletThis = sourceTracklets(thisFrameDetections(j,8));
                    
                    %the indicies of this detection within the tracklets
                    trackletPrevInds = sourceInds(prevFrameDetections(i,8));
                    trackletThisInds = sourceInds(thisFrameDetections(j,8));
                    
                    %don't recompute the costs for links that already exist
                    alreadyLinked = linkGraph(prevFrameDetections(i,8),thisFrameDetections(j,8)) ~= -1;
                   
                    if isStartPoint && isEndPoint && t > 0 && t <= maxFrameGapBetweenTracklets ...
                        && trackletPrev ~= trackletThis && trackletPrev ~= 0 && trackletThis ~= 0
                                                                                   
                        distBetweenDets = sqrt(sum((thisFrameDetections(j,3:4) - prevFrameDetections(i,3:4)).^2));
                        sourceTrackletLength = numel(tracklets{trackletPrev});
                        targetTrackletLength = numel(tracklets{trackletThis});

                        maxSpeed = (min([thisFrameDetections(j,6) prevFrameDetections(i,6)]) * heightsPerSec) / fps;
                        DistanceThreshold = maxSpeed * t;

                        if distBetweenDets < DistanceThreshold && t <= max([sourceTrackletLength,targetTrackletLength])

                            %find the "energy" associated with this link between tracklets
                            linkEnergy = findTrackletLinkEnergyByRegression(detections,tracklets,trackletPrev,trackletThis,trackletPrevInds,trackletThisInds,fps);
                            costAppearance = 1 - findTrackletAppearanceSimilarity(detections,tracklets,trackletPrev,trackletThis,appModels);

                            if linkEnergy < linkEnergyThreshold
    
                                costLinkEnergy = 1 - exp(1).^-((linkEnergy / linkEnergyThreshold)^2);
                                costTime = 1 - exp(1).^-(((t-1) / maxFrameGapBetweenTracklets)^2);
                                
                                if alreadyLinked == false
                                    linkGraph(prevFrameDetections(i,8),thisFrameDetections(j,8)) = (costTime + costLinkEnergy + costAppearance);
                                else
                                    linkGraph(prevFrameDetections(i,8),thisFrameDetections(j,8)) = min([(costTime + costLinkEnergy + costAppearance) linkGraph(prevFrameDetections(i,8),thisFrameDetections(j,8))]);
                                end
                                if alreadyLinked == false
                                    linkIndexGraph(prevFrameDetections(i,8),thisFrameDetections(j,8)) = linkIndex;
                                    linkIndex = linkIndex + 1;
                                end
                            end

                        end
                    end
                end
            end
        end
                
        prevFrameDetections = [prevFrameDetections; thisFrameDetections];
        inds = find((f - prevFrameDetections(:,1)) < maxFrameGapBetweenTracklets);
        prevFrameDetections = prevFrameDetections(inds,:);
        nPrevFrameDetections = size(prevFrameDetections,1);
    end

    nTotalLinks = sum(linkGraph(:) ~= -1);
end