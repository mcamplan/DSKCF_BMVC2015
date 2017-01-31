function [pos,trackerDSKCF_struct,trackerDSKCF_structOccluder,scaleDSKCF_struct,...
    DSKCFparameters_Occluder]=singleFrameDSKCF(firstFrame,pos,frameCurr,...
    trackerDSKCF_struct,DSKCFparameters,scaleDSKCF_struct,...
    trackerDSKCF_structOccluder,DSKCFparameters_Occluder)
% SINGLEFRAMEDSKCF.m is the core function of DS-KCF tracker
%
%
%   SINGLEFRAMEDSKCF is the core function of DS-KCF tracker (for more
%   details see [1]). It applies the DS-KCF tracker to a single frame of
%   the sequence. This function is based on several data structures where
%   input and output data is stored.Please note that data structures are
%   supposed to be initialized as in wrapperDSKCF and runDSKCF.m test
%   script.
%
%   INPUT:
%   - firstFrame   boolean flag that marks the first frame of the
%   sequence
%   - frameCurr    first frame data structure (see WRAPPERDSKCF)
%   - trackerDSKCF_struct  DS-KCF tracker data structure (see WRAPPERDSKCF,
%   INITDSKCFTRACKER)
%   - DSKCFparameters, DSKCFparameters_Occluder    parameters structures
%   (see WRAPPERDSKCF)
%   - scaleDSKCF_struct    first frame data structure (see
%   wrapperDSKCF,INITDSKCFPARAM)
%   -trackerDSKCF_structOccluder DS-KCF tracker data structure for the
%   occluder
%   -pos previous position of the DS-KCF tracker pos=[y x] where x is the
%   column and y is the row index
%
%
%   OUTPUT
%   -pos updated position of the DS-KCF tracker pos=[y x] where x is the
%   column and y is the row index
%   -trackerDSKCF_struct,trackerDSKCF_structOccluder updated data
%   structures for the trackers
%   -scaleDSKCF_struct updated scale data structure
%   -DSKCFparameters_Occluder updated parameter structure for the occluder
%
%  See also wrapperDSKCF, CHECKOCCLUSIONSDSKCF_NOISEMODEL,
%  CHECKOCCLUSIONSDSKCF_SECONDPLANE, ENLARGEBB, OCCLUDINGOBJECTSEGDSKCF,
%  TARGETSEARCHDSKCF, GETSCALEFACTORSTRUCT, FROMBBTOCENTRALPOINT,
%  FROMCENTRALPOINTTOBB, GAUSSIAN_SHAPED_LABELS, GET_SUBWINDOW,
%  MAXRESPONSEDEPTHWEIGHTDSKCF, MODELUPDATEDSKCF, RESETDSKCFTRACKERINFO,
%  ROIFROMBB,SINGLEFRAMEDSKCF_OCCLUDER
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

im=frameCurr.gray;
depth=frameCurr.depth;
noData=frameCurr.depthNoData;
depth16Bit=frameCurr.depth16Bit;

%hard coded threshold for DS-KCF algorithm see [1] for more details
confInterval1=0.4;
confInterval2=0.15;
confValue3=0.14;

currentScale=scaleDSKCF_struct.i;

%clean target struct.....
trackerDSKCF_struct=resetDSKCFtrackerInfo(trackerDSKCF_struct);

%This is not the first frame we can start tracking!!!!!!!!!!
if(firstFrame==false)
    
    %no occlusion case
    if ~trackerDSKCF_struct.previousTarget.underOcclusion,
        
        %track the object and then check for occlusions
        patch = get_subwindow(im, pos, scaleDSKCF_struct.windows_sizes(currentScale).window_sz);
        patch_depth = get_subwindow(depth, pos, scaleDSKCF_struct.windows_sizes(currentScale).window_sz);
        
        %calculate response of the DS-KCF tracker
        [response, maxResponse,pos]=maxResponseDepthWeightDSKCF(...
            patch,patch_depth,depth16Bit, DSKCFparameters.features,DSKCFparameters.kernel,...
            pos,DSKCFparameters.cell_size, scaleDSKCF_struct.cos_windows(currentScale).cos_window,...
            trackerDSKCF_struct.model_xf,trackerDSKCF_struct.model_alphaf,...
            trackerDSKCF_struct.model_xDf,trackerDSKCF_struct.model_alphaDf,...
            trackerDSKCF_struct.previousTarget.meanDepthObj,...
            trackerDSKCF_struct.previousTarget.stdDepthObj);
        
        %update tracker struct, new position etc
        trackerDSKCF_struct.currentTarget.posX=pos(2);
        trackerDSKCF_struct.currentTarget.posY=pos(1);
        
        trackerDSKCF_struct.currentTarget.bb=fromCentralPointToBB...
            (trackerDSKCF_struct.currentTarget.posX,trackerDSKCF_struct.currentTarget.posY,...
            trackerDSKCF_struct.currentTarget.w,trackerDSKCF_struct.currentTarget.h,...
            size(im,2),size(im,1));
        
        trackerDSKCF_struct.currentTarget.conf=max(response(:));%use this one, discard the weight...
        
        %segment the depth data inside the tracked region
        [p, trackerDSKCF_struct.currentTarget.meanDepthObj,...
            trackerDSKCF_struct.currentTarget.stdDepthObj,estimatedDepth,...
            estimatedSTD,minIndexReduced,trackerDSKCF_struct.currentTarget.LabelRegions,...
            trackerDSKCF_struct.currentTarget.Centers,trackerDSKCF_struct.currentTarget.regionIndex,...
            trackerDSKCF_struct.currentTarget.LUT] = checkOcclusionsDSKCF_noiseModel(...
            depth16Bit,noData,trackerDSKCF_struct, trackerDSKCF_struct.currentTarget.bb);
        
        %occlusion condition
        trackerDSKCF_struct.currentTarget.underOcclusion = abs(p)>0.35 && trackerDSKCF_struct.currentTarget.conf<confInterval1;
        
        if ~trackerDSKCF_struct.currentTarget.underOcclusion
            %eventually correct the frameCurr.targetDepthFast
            if(trackerDSKCF_struct.currentTarget.meanDepthObj~=estimatedDepth && minIndexReduced==1)
                trackerDSKCF_struct.currentTarget.meanDepthObj=estimatedDepth;
                trackerDSKCF_struct.currentTarget.stdDepthObj=estimatedSTD;
            end
        end
        
        %IF UNDER OCCLUSION START TO SEGMENT OCCLUDER AND OCCLUDED
        %OBJECTS
        if trackerDSKCF_struct.currentTarget.underOcclusion,
            % initialize occlusion
            [tmpOccBB] = occludingObjectSegDSKCF(depth16Bit,trackerDSKCF_struct);
            
            if (isempty(tmpOccBB) | isnan(tmpOccBB) )
                trackerDSKCF_struct.currentTarget.underOcclusion = 0;
            else
                if trackerDSKCF_struct.currentTarget.conf <0.15,
                    trackerDSKCF_struct.currentTarget.bb = [];
                end
                trackerDSKCF_struct.currentTarget.occBB = tmpOccBB;
            end
            
            %occlusion detected......is time to fill the struct
            %for the occluder we not use any change of scale or other
            %occlusion detector!!!! so now you have to re-init it,
            %according to the new patch size etc etc
            
            if (isempty(tmpOccBB)==false)
                %assign the occluding bb to the occluder data struct
                trackerDSKCF_structOccluder.previousTarget.bb=...
                    tmpOccBB;
                
                [trackerDSKCF_structOccluder.previousTarget.posX,...
                    trackerDSKCF_structOccluder.previousTarget.posY...
                    trackerDSKCF_structOccluder.previousTarget.w,...
                    trackerDSKCF_structOccluder.previousTarget.h]...
                    =fromBBtoCentralPoint(tmpOccBB);
                
                trackerDSKCF_structOccluder.target_sz=...
                    [trackerDSKCF_structOccluder.previousTarget.h, ...
                    trackerDSKCF_structOccluder.previousTarget.w];
                
                trackerDSKCF_structOccluder.window_sz = floor(...
                    trackerDSKCF_structOccluder.target_sz *...
                    (1 + DSKCFparameters_Occluder.padding));
                
                trackerDSKCF_structOccluder.output_sigma = sqrt(prod(...
                    trackerDSKCF_structOccluder.target_sz)) * ...
                    DSKCFparameters_Occluder.output_sigma_factor / ...
                    DSKCFparameters_Occluder.cell_size;
                
                trackerDSKCF_structOccluder.yf = fft2(gaussian_shaped_labels(...
                    trackerDSKCF_structOccluder.output_sigma, floor(...
                    trackerDSKCF_structOccluder.window_sz / DSKCFparameters_Occluder.cell_size)));
                
                %store pre-computed cosine window
                trackerDSKCF_structOccluder.cos_window = hann(size(...
                    trackerDSKCF_structOccluder.yf,1)) * hann(size(...
                    trackerDSKCF_structOccluder.yf,2))';
                
                [trackerDSKCF_structOccluder]=singleFrameDSKCF_occluder(1,im,...
                    depth,trackerDSKCF_structOccluder,DSKCFparameters_Occluder);
                
                %update target size in the current object tracker
                trackerDSKCF_structOccluder.currentTarget=...
                    trackerDSKCF_structOccluder.previousTarget;
            end
        end
        
    else
        %PREVIOUS FRAME UNDER OCCLUSION....
        
        %track the occluded object!!!
        [trackerDSKCF_structOccluder,occludedPos]=singleFrameDSKCF_occluder(0,im,...
            depth,trackerDSKCF_structOccluder,DSKCFparameters_Occluder);
        
        %update occluder previous position for tracking...
        trackerDSKCF_structOccluder.previousTarget.posX=...
            trackerDSKCF_structOccluder.currentTarget.posX;
        trackerDSKCF_structOccluder.previousTarget.posY=...
            trackerDSKCF_structOccluder.currentTarget.posY;
        
        
        if isempty(trackerDSKCF_structOccluder.currentTarget.bb),
            trackerDSKCF_structOccluder.currentTarget.bb = ...
                trackerDSKCF_structOccluder.previousTarget.bb;
        end
        %then update the previous
        trackerDSKCF_structOccluder.previousTarget.bb=...
            trackerDSKCF_structOccluder.currentTarget.bb;
        
        %update occluder bb in the main target
        trackerDSKCF_struct.currentTarget.occBB=trackerDSKCF_structOccluder.currentTarget.bb;
        
        
        %enlarge searching reagion...
        if(isempty(trackerDSKCF_struct.previousTarget.bb)==false)
            extremaBB=[min([trackerDSKCF_struct.currentTarget.occBB(1),...
                trackerDSKCF_struct.previousTarget.occBB(1),...
                trackerDSKCF_struct.previousTarget.bb(1)]),...
                min([trackerDSKCF_struct.currentTarget.occBB(2),...
                trackerDSKCF_struct.previousTarget.occBB(2),...
                trackerDSKCF_struct.previousTarget.bb(2)]),...
                max([trackerDSKCF_struct.currentTarget.occBB(3),...
                trackerDSKCF_struct.previousTarget.occBB(3),...
                trackerDSKCF_struct.previousTarget.bb(3)]),...
                max([trackerDSKCF_struct.currentTarget.occBB(4),...
                trackerDSKCF_struct.previousTarget.occBB(4),...
                trackerDSKCF_struct.previousTarget.bb(4)])];
            
            extremaBB=[max(extremaBB(1),1),max(extremaBB(2),1),...
                min(extremaBB(3),size(im,2)),min(extremaBB(4),size(im,1))];
        else
            extremaBB=[min(trackerDSKCF_struct.currentTarget.occBB(1),...
                trackerDSKCF_struct.previousTarget.occBB(1)),...
                min(trackerDSKCF_struct.currentTarget.occBB(2),...
                trackerDSKCF_struct.previousTarget.occBB(2)),...
                max(trackerDSKCF_struct.currentTarget.occBB(3),...
                trackerDSKCF_struct.previousTarget.occBB(3)),...
                max(trackerDSKCF_struct.currentTarget.occBB(4),...
                trackerDSKCF_struct.previousTarget.occBB(4))];
            extremaBB=[max(extremaBB(1),1),max(extremaBB(2),1),...
                min(extremaBB(3),size(im,2)),min(extremaBB(4),size(im,1))];
            
        end
        
        %bbIn=framePrev.occBB;
        bbIn=extremaBB;
        bbIn=enlargeBB(bbIn ,0.05,size(noData));
        
        %extract the target roi, from the depth and the nodata mask
        front_depth=roiFromBB(depth16Bit,bbIn);
        depthNoData=roiFromBB(noData,bbIn);
        
        %in case of bounding box with no depth data....
        if(sum(sum(depthNoData)')==size(front_depth,1)*size(front_depth,2))
            trackerDSKCF_struct.currentTarget.LabelRegions=depthNoData>10000000;
            trackerDSKCF_struct.currentTarget.Centers=1000000;
            trackerDSKCF_struct.currentTarget.LUT=[];
            %eventually segment the current occluding area
        else
            %note, we are saving the segmentation of the occluding region,
            %in the target struct rather thant the other one!!!!!
            [trackerDSKCF_struct.currentTarget.LabelRegions,...
                trackerDSKCF_struct.currentTarget.Centers,...
                trackerDSKCF_struct.currentTarget.LUT,~,~,~]=...
                fastDepthSegmentationDSKCF_noiseModel(front_depth,3,depthNoData,...
                1,[-1 -1 -1],1,trackerDSKCF_struct.currentTarget.meanDepthObj,...
                trackerDSKCF_struct.currentTarget.stdDepthObj,[2.3,0.00055,0.00000235]);
            
        end
        
        %filter out very small regions
        tmpProp=regionprops(trackerDSKCF_struct.currentTarget.LabelRegions,'Area');
        areaList= cat(1, tmpProp.Area);
        minArea=scaleDSKCF_struct.target_sz(currentScale).target_sz(2)*...
            scaleDSKCF_struct.target_sz(currentScale).target_sz(1)*0.05;
        
        areaSmallIndex=areaList<minArea;
        
        %exclude the small area index setting a super high depth!!!!!!!!
        %it will never be used
        trackerDSKCF_struct.currentTarget.Centers(:,areaSmallIndex)= 1000000;
        
        [dummyVal,trackerDSKCF_struct.currentTarget.regionIndex]=...
            min(trackerDSKCF_struct.currentTarget.Centers);
        
        %very bad segmentation, full of small regions. Set up a flag....
        if(dummyVal==1000000)
            trackerDSKCF_struct.currentTarget.regionIndex=6666666;
        end
        
        %search for target's candidates in the search region.....
        [tarBB, segmentedOccBB, targetList, targetIndex,occmask] =  ...
            targetSearchDSKCF(bbIn, trackerDSKCF_struct, DSKCFparameters,...
            im,depth,depth16Bit,scaleDSKCF_struct,confValue3);
        
        if(isempty(segmentedOccBB)==false)
            extremaBB=[min([trackerDSKCF_struct.currentTarget.occBB(1),...
                segmentedOccBB(1)]),min([trackerDSKCF_struct.currentTarget.occBB(2),...
                segmentedOccBB(2)]),max([trackerDSKCF_struct.currentTarget.occBB(3),...
                segmentedOccBB(3)]),max([trackerDSKCF_struct.currentTarget.occBB(4),...
                segmentedOccBB(4)])];
            
            trackerDSKCF_struct.currentTarget.occBB=[max(extremaBB(1),1),...
                max(extremaBB(2),1),min(extremaBB(3),size(im,2)),...
                min(extremaBB(4),size(im,1))]';
        end
        
        if(isempty(trackerDSKCF_struct.currentTarget.occBB))
            trackerDSKCF_struct.currentTarget.occBB=trackerDSKCF_struct.previousTarget.occBB;
        end
        
        %THESE CANDIDATES....CAME FROM THE SEGMENTATIONS....YOU NEED TO
        %RESIZE THEM WITH THE REQUIRED WINDOW...
        centerNew=[];
        if(~isempty(tarBB))
            
            widthtarBB=tarBB(3)-tarBB(1);
            heighttarBB=tarBB(4)-tarBB(2);
            centerNew=floor([tarBB(1)+widthtarBB/2 tarBB(2)+heighttarBB/2]);
            
            tarBB(1:2)=centerNew - scaleDSKCF_struct.target_sz(scaleDSKCF_struct.i).target_sz([2 1])/2;
            tarBB(3:4)=centerNew + scaleDSKCF_struct.target_sz(scaleDSKCF_struct.i).target_sz([2 1])/2;
            
        end
        
        
        % calculate detection and segmentation consistency
        trackerDSKCF_struct.currentTarget.bb = tarBB;
        trackerDSKCF_struct.currentTarget.conf = targetList.Conf_class(targetIndex);
        
        
        
        %%Kill some strange respons on the occluding mask
        occmask = imfill(occmask,'holes');
        tmpMask=repmat(0,size(depth));
        tmpMask(bbIn(2):bbIn(4),bbIn(1):bbIn(3))=occmask;
        
        if(isempty(centerNew)==false)
            
            %re-assign new position to the target, even if the target is
            %not valid, you need this just for visualization or checking
            %the algorithm....
            [trackerDSKCF_struct.currentTarget.posX,...
                trackerDSKCF_struct.currentTarget.posY...
                trackerDSKCF_struct.currentTarget.w,...
                trackerDSKCF_struct.currentTarget.h]...
                =fromBBtoCentralPoint(trackerDSKCF_struct.currentTarget.bb);
            %update tracker struct, new position etc
            pos(2)=trackerDSKCF_struct.currentTarget.posX;
            pos(1)=trackerDSKCF_struct.currentTarget.posY;
            
            if(tmpMask(centerNew(2),centerNew(1))==1)
                trackerDSKCF_struct.currentTarget.conf=0;
            end
        end
        
        % check recovery
        trackerDSKCF_struct.currentTarget.underOcclusion =1;
        if ~isempty(trackerDSKCF_struct.currentTarget.bb)
            [p, TMPtargetDepth,TMPtargetStd,TMPLabelRegions,TMPCenters,...
                TMPregionIndex,TMPLUT,secondPlaneDepth,secondPlaneDepthStd] ...
                = checkOcclusionsDSKCF_secondPlane(depth16Bit,...
                noData,trackerDSKCF_struct, trackerDSKCF_struct.currentTarget.bb);
            
            if trackerDSKCF_struct.currentTarget.conf > confInterval1/2 && p<0.35,
                trackerDSKCF_struct.currentTarget.underOcclusion =0;
            end
            
            if ~trackerDSKCF_struct.currentTarget.underOcclusion,
                tmpWeight = max(0,min(1,trackerDSKCF_struct.currentTarget.conf));
                
            else
                tmpWeight = 0;
            end
            
            
            if(isempty(secondPlaneDepth)==false)
                TMPtargetDepth=secondPlaneDepth;
                TMPtargetStd=secondPlaneDepthStd;
                tmpWeight=tmpWeight*2.5;
                if(tmpWeight>1)
                    tmpWeight=0.95;
                end
                
            end
            
            trackerDSKCF_struct.currentTarget.meanDepthObj = tmpWeight * TMPtargetDepth + (1-tmpWeight) * trackerDSKCF_struct.currentTarget.meanDepthObj;
            trackerDSKCF_struct.currentTarget.stdDepthObj = tmpWeight * TMPtargetStd + (1-tmpWeight) * trackerDSKCF_struct.currentTarget.stdDepthObj;
            
        else
            trackerDSKCF_struct.currentTarget.meanDepthObj = trackerDSKCF_struct.previousTarget.meanDepthObj;
            trackerDSKCF_struct.currentTarget.stdDepthObj = trackerDSKCF_struct.previousTarget.stdDepthObj;
        end
    end
    
    
end

%IF UNDER OCCLUSION DON'T UPDATE.....
if(trackerDSKCF_struct.previousTarget.underOcclusion==false && trackerDSKCF_struct.currentTarget.underOcclusion==false)
    
    %check for scale change
    if(firstFrame==false)
        scaleDSKCF_struct=getScaleFactorStruct(trackerDSKCF_struct.currentTarget.meanDepthObj,scaleDSKCF_struct);
    end
    
    %obtain a subwindow for training at newly estimated target position
    patch = get_subwindow(im, pos, scaleDSKCF_struct.windows_sizes(scaleDSKCF_struct.i).window_sz);
    patch_depth = get_subwindow(depth, pos, scaleDSKCF_struct.windows_sizes(scaleDSKCF_struct.i).window_sz);
    
    %update the model
    [trackerDSKCF_struct.model_alphaf, trackerDSKCF_struct.model_alphaDf, ...
        trackerDSKCF_struct.model_xf, trackerDSKCF_struct.model_xDf]=...
        modelUpdateDSKCF(firstFrame,patch,patch_depth,DSKCFparameters.features,...
        DSKCFparameters.cell_size,scaleDSKCF_struct.cos_windows(scaleDSKCF_struct.i).cos_window,...
        DSKCFparameters.kernel,scaleDSKCF_struct.yfs(scaleDSKCF_struct.i).yf,...
        DSKCFparameters.lambda,trackerDSKCF_struct.model_alphaf, ...
        trackerDSKCF_struct.model_alphaDf,trackerDSKCF_struct.model_xf,...
        trackerDSKCF_struct.model_xDf,scaleDSKCF_struct.updated,DSKCFparameters.interp_factor);
    
    %if scale changed you must change tracker size information
    if(scaleDSKCF_struct.updated)
        %[newH,newW]= ;
        trackerDSKCF_struct.currentTarget.h=scaleDSKCF_struct.target_sz(scaleDSKCF_struct.i).target_sz(1);
        trackerDSKCF_struct.currentTarget.w=scaleDSKCF_struct.target_sz(scaleDSKCF_struct.i).target_sz(2);
    end
else
    %%THERE IS AN OCCLUSION.....NOW WHAT TO DO.....? DON'T UPDATE MODEL....
    %UPDATE ONLY POSITION!!!!!
    
    % TO BE CHECKEDD!!!!!
    if(isempty(trackerDSKCF_struct.currentTarget.bb))
        pos=[];
    else
        pos=[trackerDSKCF_struct.currentTarget.posY,trackerDSKCF_struct.currentTarget.posX];
    end
end

end