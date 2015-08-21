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


%find the energy cost of a proposed link between tracklets
%use regression to project the trackets forwards and backwards, then
%measure the deviation from the opposite tracklet
function linkEnergy = findTrackletLinkEnergyByRegression(detections,tracklets,trackletPrev,trackletThis,trackletPrevInds,trackletThisInds,fps)

    sourceTrackletDetInds = tracklets{trackletPrev};
    targetTrackletDetInds = tracklets{trackletThis};
    
    sourceTracklet = detections(sourceTrackletDetInds,:);
    targetTracklet = detections(targetTrackletDetInds,:);
       
    %fit a regression line through (the last 2 seconds) of each tracklet in order to predict
    %forwards and backwards - ideally I should be using
    %robust-regression to avoid outliers...
    
%     tmeForward = sourceTracklet(:,1);
%     forwardX = sourceTracklet(:,3);
%     forwardY = sourceTracklet(:,4);    
%     pforwardX = polyfit(tmeForward,forwardX,1); % x_t = t*pX(1) + pX(2)
%     pforwardY = polyfit(tmeForward,forwardY,1); % y_t = t*pY(1) + pY(2)
%     
%     tmeBackward = targetTracklet(:,1);
%     backwardX = targetTracklet(:,3);
%     backwardY = targetTracklet(:,4);    
%     pbackwardX = polyfit(tmeBackward,backwardX,1); % x_t = t*pX(1) + pX(2)
%     pbackwardY = polyfit(tmeBackward,backwardY,1); % y_t = t*pY(1) + pY(2)   
            
    j = size(sourceTracklet,1);
    while j > 1 && (sourceTracklet(end,1) - sourceTracklet(j,1)) < (fps*2)
        j = j - 1;
    end
    forwardInds = j:size(sourceTracklet,1);    
    
    tmeForward = sourceTracklet(forwardInds,1);
    forwardX = sourceTracklet(forwardInds,3);
    forwardY = sourceTracklet(forwardInds,4);    
    pforwardX = polyfit(tmeForward,forwardX,1); % x_t = t*pX(1) + pX(2)
    pforwardY = polyfit(tmeForward,forwardY,1); % y_t = t*pY(1) + pY(2)
    
    j = 1;
    while j < size(targetTracklet,1) && (targetTracklet(j,1) - targetTracklet(1,1)) < (fps*2)
        j = j + 1;
    end
    backwardInds = 1:j;
    
    tmeBackward = targetTracklet(backwardInds,1);
    backwardX = targetTracklet(backwardInds,3);
    backwardY = targetTracklet(backwardInds,4);    
    pbackwardX = polyfit(tmeBackward,backwardX,1); % x_t = t*pX(1) + pX(2)
    pbackwardY = polyfit(tmeBackward,backwardY,1); % y_t = t*pY(1) + pY(2)
            
    %scatter3(sourceTracklet(:,3),sourceTracklet(:,4),sourceTracklet(:,1));
    %hold on;
    %scatter3(targetTracklet(:,3),targetTracklet(:,4),targetTracklet(:,1));
    
    gapStart = sourceTracklet(trackletPrevInds,:);
    gapEnd = targetTracklet(trackletThisInds,:);    
    deltaTime = gapEnd(1) - gapStart(1);    
    
    %predict the gap forward in time
    %compare with the target tracklet actual positions
    energyForward = 0;
    forwardCount = 0;
    gapStartTime = tmeBackward(1);
    for t = 1:fps
        %predictedPos = [gapEnd(3:4) + (gapVel * gapSpeed * t),gapEnd(1) + t];
        
        predictedPos = [pforwardX(1)*(gapStartTime + t) + pforwardX(2),pforwardY(1)*(gapStartTime + t) + pforwardY(2)];
        %scatter3(predictedPos(1),predictedPos(2),gapStartTime + t);
                
        if (trackletThisInds + t) <= size(targetTracklet,1)
            %measure difference in position
            e = sqrt(sum((predictedPos(1:2) - targetTracklet(trackletThisInds + t,3:4)).^2));
            energyForward = energyForward + e;
            forwardCount = forwardCount + 1;
        else
            break;
        end
    end
    
    %predict the gap backwards in time
    %compare with the source tracklet actual positions
    energyBackward = 0;
    backwardCount = 0;
    gapStartTime = tmeForward(end);
    for t = 1:fps
        %predictedPos = [gapStart(3:4) + (-gapVel * gapSpeed * t),gapStart(1) - t];
        
        predictedPos = [pbackwardX(1)*(gapStartTime - t) + pbackwardX(2),pbackwardY(1)*(gapStartTime - t) + pbackwardY(2)];
        %scatter3(predictedPos(1),predictedPos(2),gapStartTime - t);
        
        if (trackletPrevInds - t) > 0
            %measure difference in position
            e = sqrt(sum((predictedPos(1:2) - sourceTracklet(trackletPrevInds - t,3:4)).^2));
            energyBackward = energyBackward + e;
            backwardCount = backwardCount + 1;
        else
            break;
        end
    end
    %hold off;
    
    linkEnergy = energyForward / forwardCount + energyBackward / backwardCount;       
end