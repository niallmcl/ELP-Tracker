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


%Take the list of detections and the folder containing a file for every
%frame.
%Return a list of appearance models, one for each detection

function appModels = getDetsAppearanceModels(detections,imgDir)
    
    nDets = size(detections,1);
    appModels = zeros(nDets,1000);
    oldFrame = -1;
    img = 0;
    for d = 1:nDets
        
        frame = detections(d,1);
        pos = detections(d,3:4);
        w = detections(d,5);
        h = detections(d,6);
        
        %only read a new image if we need to == ++faster!
        if frame ~= oldFrame 
            imgFilename = sprintf('%.6d.jpg', frame);            
            img = imread(fullfile(imgDir,imgFilename));
        end
        oldFrame = frame;
        
        pxls = getPixels(img,pos(1),pos(2),w,h);
        
        nHistogramBins = 10;
        nHorizontalSegments = 1;
        h = hist3D(pxls,nHistogramBins,nHorizontalSegments);
        h = h ./ sum(h);
        appModels(d,:) = h';        
    end
end

