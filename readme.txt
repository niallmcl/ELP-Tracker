Minimum cost network-flow tracker

This code is for a modified version of the tracker descibed in  "Enhancing
Linear Programming with Motion Modeling for Multi-target Tracking", N
McLaughlin, J Martinez Del Rincon, P Miller - (WACV), 2015

The tracking solution is computed over several iterations.  
- In the first iteration we link detections based on distance, time and appearance. 
- In later iterations we link tracklets based on motion cost, assuming a linear motion model.

To Run the code:

A harness is provided to run the tracker on all sequences of the MOTChallenge http://motchallenge.net/

MOTChallengeHarness.m

This file must be modified to point to the MOTChallenge devkit, tracker images, and detections, which are  aviable from the MOTChallenge website.

The tracker can also be used on other sequences by providing a directory of images and a detections file in the same format as the MOTChallenge i.e. sequence images in format 000001.jpg and detections in a CSV file with each row in the format: frame-num, -1, x, y, w, h, detection-confidence

