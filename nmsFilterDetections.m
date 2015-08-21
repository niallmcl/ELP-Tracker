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


%remove detections that are overlapping by >= nmsThreshold
function toKeep = nmsFilterDetections(detections,nmsThreshold)

    nFrames = max(detections(:,1));
    toKeep = zeros(1,size(detections,1));
        
    for f = 0:nFrames
        
        detsThisFrame = detections(detections(:,1) == f,:);

        %place the detections onto the scene one by one
        %if we overlap with an exisitng detection by > threshold - don't
        %place on the scene

        [~,order] = sort(detsThisFrame(:,2),'descend');
        detsThisFrame = detsThisFrame(order,:);
        
        inScene = zeros(1,size(detsThisFrame,1));
        inScene(1) = 1;
        
        %for each detection place in scene and check overlap with all
        %others already in scene
        for d = 2:size(detsThisFrame,1) %place            
            discard = false;
            for z = 1:size(detsThisFrame,1) %test
                if z~=d && inScene(z)
                    if getOverlap(detsThisFrame(z,:),detsThisFrame(d,:)) > nmsThreshold
                        discard = true;
                        break;
                    end
                end
            end
            inScene(d) = ~discard;
        end
        toKeep(detections(:,1) == f) = inScene;  
    end

end

%get the overlap between two detecitons
function overlap = getOverlap(a,b)    
    inter = rectint([a(3:4) a(5) a(6)],[b(3:4) b(5) b(6)]);
    union = (a(5) * a(6)) + (b(5) * b(6)) - inter;
    overlap = inter / union;
end