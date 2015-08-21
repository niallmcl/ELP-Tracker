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


%Given a box specified by (x,y) (w,h) return the pixels 
function pxls = getPixels(img,x,y,w,h)

    minc = x;
    maxc = x + w;
    minr = y;
    maxr = y + h;
    
    minr = keepInRange(minr,1,size(img,1));
    maxr = keepInRange(maxr,1,size(img,1));
    minc = keepInRange(minc,1,size(img,2));
    maxc = keepInRange(maxc,1,size(img,2));

    if maxr > minr && maxc > minc    
        pxls = img(minr:maxr,minc:maxc,:);
    else
        pxls = [];
    end

end

%If the number val is 
%< minV - minV
%> maxV - maxV
function val = keepInRange(val,minV,maxV)

    if val < minV
        val = minV;
    end
    if val > maxV
        val = maxV;
    end
    
end