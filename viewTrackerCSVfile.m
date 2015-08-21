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

%view CSV files

function viewTrackerCSVfile()

    rootDir = 'C:\Users\3047122\Documents\Test Data\2DMOT2015\train\';
    seqDirs = {'ADL-Rundle-6','ADL-Rundle-8','ETH-Bahnhof','ETH-Pedcross2','ETH-Sunnyday','KITTI-13','KITTI-17','PETS09-S2L1','TUD-Campus','TUD-Stadtmitte','Venice-2'};
    seqLens = [525,654,1000,840,354,340,145,795,71,179,600];
    seqFPS = [30,30,14,14,14,10,10,7,25,25,30];
    
%     rootDir = 'C:\Users\3047122\Documents\Research\Test Data\2DMOT2015\test\';
%     seqDirs = {'ADL-Rundle-1','ADL-Rundle-3','AVG-TownCentre','ETH-Crossing','ETH-Jelmoli','ETH-Linthescher','KITTI-16','KITTI-19','PETS09-S2L2','TUD-Crossing','Venice-1'};
%     seqLens = [500,625,450,219,440,1194,209,1059,436,201,450];
%     seqFPS = [30,30,3,14,14,14,10,10,7,25,30];
    
    uniqueRunNum = 1;
    resultsDir = ['./tracker output/resultsRunTest' int2str(uniqueRunNum) '/'];
    
    videoOutputDir = './tracker video output/';
    
    for seqNum = 1:11
        
        disp(seqNum);
    
        videoOutputFile = [videoOutputDir seqDirs{seqNum} '.avi'];
        outputVideo = VideoWriter(videoOutputFile);
        outputVideo.FrameRate = seqFPS(seqNum);
        open(outputVideo);    

        inputVideoFile = [rootDir seqDirs{seqNum} '/img1/'];
        inputDetections = [rootDir seqDirs{seqNum} '/det/det.txt'];
        CSVOutputFile = [resultsDir seqDirs{seqNum} '.txt'];
        groundTruthFile = [rootDir seqDirs{seqNum} '/gt/gt.txt'];

        groundTruth = csvread(groundTruthFile);
        tracks = csvread(CSVOutputFile);
        detections = csvread(inputDetections);

        %generate a random colour for each track
        nColours = numel(unique(tracks(:,2)));
        trackColours = floor((rand(nColours,3) * 0.5 + 0.5) * 255);

        for frame = 1:seqLens(seqNum)

            imgTracks = imread([inputVideoFile,genPetsFilename(frame)]);
            imgDets = imgTracks;
            imgGt = imgTracks;

            %draw the ground-truth
            gtInds = find(groundTruth(:,1) == frame);
            for i = 1:numel(gtInds)
                bbox = [groundTruth(gtInds(i),3) groundTruth(gtInds(i),4) groundTruth(gtInds(i),5) groundTruth(gtInds(i),6)];
                imgGt = insertObjectAnnotation(imgGt, 'Rectangle', bbox, 'obj1','color', [0 255 0]);
            end

            %draw the tracks
            trInds = find(tracks(:,1) == frame);
            for i = 1:numel(trInds)
                bbox = [tracks(trInds(i),3) tracks(trInds(i),4) tracks(trInds(i),5) tracks(trInds(i),6)];
                trColor = trackColours(tracks(trInds(i),2),:);
                imgTracks = insertObjectAnnotation(imgTracks, 'Rectangle', bbox, ['Track ' num2str(tracks(trInds(i),2))] ,'color', trColor);
            end

            %draw the detections


            %draw the tracks
            trInds = find(detections(:,1) == frame);
            for i = 1:numel(trInds)
                bbox = [detections(trInds(i),3) detections(trInds(i),4) detections(trInds(i),5) detections(trInds(i),6)];
                imgDets = insertObjectAnnotation(imgDets, 'Rectangle', bbox ,'a','color', [0 255 0]);
            end

            %draw the detections
            %subplot(2,2,1);
            %imagesc(imgTracks);
            writeVideo(outputVideo,imgTracks);
    %         subplot(2,2,2);
    %         imagesc(imgDets);
    %         subplot(2,2,3);
    %         imagesc(imgGt);

            %drawnow;
        end
        close(outputVideo);
    end

end

function filename = genPetsFilename(f)

    % read the next frame
    frameNumInStr = num2str(f);
    maxZeros = 6;
    numZeros = maxZeros - numel(frameNumInStr);
    zerosStr = '';
    for zc = 1:numZeros
        zerosStr = [zerosStr '0'];
    end
    filename = [zerosStr frameNumInStr '.jpg'];    

end