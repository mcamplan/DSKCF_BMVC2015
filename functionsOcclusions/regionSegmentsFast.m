%REGIONSEGMENTSFAST function for extracting target candidates from the occluded area
%
%REGIONSEGMENTSFAST.m this function analyzes the segmented occluding area
%and extract meaningful target candidates. For more information about how
%DSKCF handles occlusions see [1].
%
%
%  INPUT:
%  - labelMatrix   image containing pixels' label corresponding to the
%  segmentation
%  - imageCoordinateOffset  bounding box containing the occluding area n
%  the format [topLeftX, topLeftY, bottomRightX, bottomRightY] read as
%  [columnIndexTopLeft, rowIndexTopLeft, columnIndexBottomRight,
%  rowIndexBottomRight]
%
%  OUTPUT - tarListFull Matrix that contains (in each column) target
%  candidates' bounding box in the format [topLeftX, topLeftY,
%  bottomRightX, bottomRightY] read as [columnIndexTopLeft,
%   rowIndexTopLeft, columnIndexBottomRight, rowIndexBottomRight]
%
% See also TARGETSEARCHDSKCF
%
%
%  [1] M. Camplani, S. Hannuna, D. Damen, M. Mirmehdi, A. Paiment, L. Tao,
%   T. burghard. Robust Real-time RGB-D Tracking with Depth Scaling
%   Kernelised Correlation Filters and Occlusion Handling, BMVC 2015
%
%
%  University of Bristol
%  Massimo Camplani and Sion Hannuna
%
%  massimo.camplani@bristol.ac.uk
%  hannuna@compsci.bristol.ac.uk

function [ tarListFull, areaVector ] = regionSegmentsFast(labelMatrix, imageCoordinateOffset)
tarListFull=[];


tarBBProp=regionprops(labelMatrix,'BoundingBox','Area');
areaVector=cat(1, tarBBProp.Area);

for i=1:length(areaVector)
    
    tmpBB=tarBBProp(i).BoundingBox;
    %use extrema points.....
    tmpBB=ceil([tmpBB(1), tmpBB(2),tmpBB(1)+tmpBB(3),tmpBB(2)+tmpBB(4)]);
    %and recenter to the entire image coordinate
    tmpBB([1 3])=tmpBB([1 3])+imageCoordinateOffset(1);
    tmpBB([2 4])=tmpBB([2 4])+imageCoordinateOffset(2);
    
    tarListFull=[tarListFull, tmpBB'];
end

end


