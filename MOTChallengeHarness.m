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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MOT Challenge tracker harness
% http://motchallenge.net/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear vars

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE - MODIFY FOLLOWING  TWO LINES TO POINT TO DEVKIT & SEQUENCES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
devkitRoot = fullfile('..','devkit','devkit');     %get the devkit from the MOT challenge website for tracker evaluation purposes
trackerResourcesRoot = fullfile('..','2DMOT2015'); %set to the directory containing the images and detections for MOT challenge 

addpath(genpath(devkitRoot));        
uniqueRunNum = 1;            
doTraining = true;    

if doTraining
    rootDir = fullfile(trackerResourcesRoot,'train');
    seqDirs = {'ADL-Rundle-6','ADL-Rundle-8','ETH-Bahnhof','ETH-Pedcross2','ETH-Sunnyday','KITTI-13','KITTI-17','PETS09-S2L1','TUD-Campus','TUD-Stadtmitte','Venice-2'};
    seqLens = [525,654,1000,840,354,340,145,795,71,179,600];
    seqFPS = [30,30,14,14,14,10,10,7,25,25,30];
else
    rootDir = fullfile(trackerResourcesRoot,'test');
    seqDirs = {'ADL-Rundle-1','ADL-Rundle-3','AVG-TownCentre','ETH-Crossing','ETH-Jelmoli','ETH-Linthescher','KITTI-16','KITTI-19','PETS09-S2L2','TUD-Crossing','Venice-1'};
    seqLens = [500,625,450,219,440,1194,209,1059,436,201,450];
    seqFPS = [30,30,3,14,14,14,10,10,7,25,30];   
end

%make a directory to hold the output from the tracker
resultsDir = fullfile('.','tracker output',['resultsRun_' int2str(uniqueRunNum)]);
if  ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

for seqNum = 1:numel(seqDirs)

    seqData.rootDir = rootDir;
    seqData.seqDir = seqDirs{seqNum};
    seqData.seqLen = seqLens(seqNum);
    seqData.seqFPS = seqFPS(seqNum);        
    seqData.inputVideoFile = fullfile(rootDir,seqDirs{seqNum},'img1');        
    seqData.inputDetections = fullfile(rootDir,seqDirs{seqNum},'det','det.txt');    
    seqData.CSVOutputFile = fullfile(resultsDir,[seqDirs{seqNum} '.txt']);

    % The following parameters control the tracker behaviour
    
    trackerConf.maxIters = 2;                       % the 1st iteration links detections using unary costs, later iterations link tracklets using motion costs
    trackerConf.linkEnergyThreshold = 100;          % maximum 'energy', measured by average deviation in pixels from linear tracklet motion, allowed when linking tracklets
    trackerConf.heightsPerSec = 3;                  % distance threshold used when linking detections. Relative to detection height which means it scales well with different video resolution / distance from the camera
    trackerConf.appearanceThreshold = 0.1;          % appearance similarity threshold to prevent detections with very different apppearance from linking
    trackerConf.doDetectionPreProcessing = true;    % apply NMS to detections before doing tracking
    trackerConf.nmsThreshold = 0.5;                 % threshold used by NMS
    trackerConf.doFilterTracklets = true;           % remove short tracklets as these may be false positives
    trackerConf.maxFrameJump = 2;                   % max frame gap between detections allowed in first iteration
    trackerConf.maxFrameJumpMultiplier = 7;         % max frame gap * (nIterations - 1) between tracklets allowed in iterations > 1

    doTracking(seqData,trackerConf);
end

%use the MOT challenge script to evaluate the tracker on all sequences
benchmarkDir = [rootDir '/'];
allMets = evaluateTracking(fullfile(devkitRoot,'seqmaps','c2-train.txt'), resultsDir, benchmarkDir);  
