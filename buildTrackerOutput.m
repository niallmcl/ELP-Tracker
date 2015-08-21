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


%take the series of detections making up a tracklet. 
%Linearly interpolate between missed detections
function [csvFinal,globalTrackNumber] = buildTrackerOutput(tracklets,detections,globalTrackNumber,fps)

    csvFinal = [];

    for t = 1:numel(tracklets)        
        trk = tracklets{t};
        csvTrk = [];
        
        for d = 1:numel(trk)
            
            tme = detections(trk(d),1);            
            pos = detections(trk(d),3:4);
            sze = detections(trk(d),5:6);
            
            csvTrk = [csvTrk; tme globalTrackNumber pos sze];
            
            %if time to next detection > 1 then interpolate            
            if d < numel(trk) && (detections(trk(d+1),1) - tme) > 1
                
               gapLength = detections(trk(d+1),1) - tme;                              
               
               endPos = detections(trk(d+1),3:4);
               endSze = detections(trk(d+1),5:6);               
               
               for step = 1:gapLength-1
                   
                   w = step / gapLength;
                   
                   interPos = pos.*(1-w) + endPos.*w;
                   interSze = sze.*(1-w) + endSze.*w;
                   
                   csvTrk = [csvTrk; (tme + step) globalTrackNumber interPos interSze];                                      
               end                
            end                            
        end
        
        for c = 3:6
            csvTrk(:,c) = smooth(csvTrk(:,c),'moving',fps);
        end
        
        %padding to get into same format as MOTChallenge required output
        csvTrk = [csvTrk (ones(size(csvTrk,1),4) * -1)];        
        csvFinal = [csvFinal; csvTrk];        
        
        globalTrackNumber = globalTrackNumber + 1;
    end

end