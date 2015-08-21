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


%Given the current linking solution - get all the tracklets, which are
%represented by lists of detections.
function [tracklets,startPoints,endPoints] = recoverTracklets(x,entryOffset,exitOffset,linksOffset,linkGraph,linkIndexGraph)

    startPoints = (find(round(x(entryOffset : exitOffset)) == 1) - 1)';    
    tracklets = {};
    
    %draw the links between detections
    for d = startPoints
        
        currTracklet = d;        
        curr = d;
        
        %get the enty and exit link indicies
        out = find(linkGraph(curr,:) ~= -1);
        while ~isempty(out)
            %check if these links are activated
            madeLink = false;
            for edgeOut = out
                linkIndx = linkIndexGraph(curr,edgeOut);
                if round(x(linksOffset + linkIndx))                    
                    %draw the link
                    %scatter3(detections(d,4),detections(d,3),detections(d,1),20,[0 1 0],'fill','o');
                    %plot3([detections(curr,4) detections(edgeOut,4)],[detections(curr,3) detections(edgeOut,3)],[detections(curr,1) detections(edgeOut,1)],'color',c);
                    %hold on;
                    madeLink = true;
                    currTracklet = [currTracklet edgeOut];
                    break;
                end
            end
            
            if madeLink            
                out = find(linkGraph(edgeOut,:) ~= -1);
                curr = edgeOut;
            else
                out = [];
            end            
        end
        tracklets = [tracklets {currTracklet}];        
    end
    
    startPoints = zeros(1,numel(tracklets));
    endPoints = zeros(1,numel(tracklets));
    for t = 1:numel(tracklets)
        tmp = tracklets{t};
        startPoints(t) = tmp(end);%tmp(1);
        endPoints(t) = tmp(1);%tmp(end);
    end
    
end