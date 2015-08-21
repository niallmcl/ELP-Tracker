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


%take two tracklets and compare the appearances of all the detections
function appearanceSimilarity = findTrackletAppearanceSimilarity(detections,tracklets,trackletPrev,trackletThis,appModels)

    sourceTrackletDetInds = tracklets{trackletPrev};
    targetTrackletDetInds = tracklets{trackletThis};
    
    sourceTracklet = detections(sourceTrackletDetInds,:);
    targetTracklet = detections(targetTrackletDetInds,:);
        
    sourceLen = size(sourceTracklet,1);
    targetLen = size(targetTracklet,1);
    
    distMat = zeros(sourceLen,targetLen);
    
    for i = 1:sourceLen
        for j = 1:targetLen
            detNumSrc = sourceTracklet(i,8);
            detNumTrg = targetTracklet(j,8);

            distMat(i,j) = sum(sqrt(appModels(detNumSrc,:) .* appModels(detNumTrg,:)));            
        end
    end
    
    appearanceSimilarity = mean(distMat(:));    
end