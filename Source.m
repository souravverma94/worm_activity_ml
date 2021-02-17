%% Assumptions
%       1. The worms do not move fast enough to move outside of a bounding
%       box between frames in the video.


%% Debugging Panel
clear all;
show_binarized=false;


%% User Interface
filename='Day 15.mov';
%% Create an Output Video
global avi;
avi = VideoWriter('Output_Video_Day15.avi');
open(avi);
global cnt;
cnt=0;

%% Convert video to readable frames
frames=VideoReader(filename);


%% User Inputted Crop Box
% ~~~~~~~~~~ User Inputed Crop Box ~~~~~~~~~~
bb = [0 0; 0 0; 0 0; 0 0];
figure;
curr_II=readFrame(frames);
imshow(curr_II,[]);
for ii=1:4
    [x, y] = ginput(1);
    bb(ii,1) = x;
    bb(ii,2) = y;
    curr_II = insertShape(curr_II,'filledcircle',[x y 7],'LineWidth',1,'Color','red');
    imshow(curr_II,[]);
    drawnow;
end
[~,mins] = mink(bb,2);
[~,maxs] = maxk(bb,2);
if bb(mins(1,1),2) < bb(mins(2,1),2)
    tl = bb(mins(1,1),:);
    bl = bb(mins(2,1),:);
else
    tl = bb(mins(2,1),:);
    bl = bb(mins(1,1),:);
end
if bb(maxs(1,1),2) < bb(maxs(2,1),2)
    tr = bb(maxs(1,1),:);
    br = bb(maxs(2,1),:);
else
    tr = bb(maxs(2,1),:);
    br = bb(maxs(1,1),:);
end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%% Frame Loop
frames=VideoReader(filename);
frameCount=1; metadata=[0 0];
while hasFrame(frames)% && frameCount<80
    curr_I=rgb2gray(readFrame(frames));
    if hasFrame(frames) % prevent fail on last frame
    curr_I=rgb2gray(readFrame(frames));
    end
    curr_bin_I=binarize_I(curr_I,tl,bl,tr);
    objs_curr_bin_I=object_detection(curr_bin_I);
    objs_curr_bin_I=classify_objects(objs_curr_bin_I);
    if frameCount==1       
        for ii=1:length(objs_curr_bin_I)
            superstructure(ii)=regionprop2structure(objs_curr_bin_I(ii));
        end
    else
        for ii=1:length(objs_curr_bin_I)
            struct=regionprop2structure(objs_curr_bin_I(ii));
            struct.centroid=objs_curr_bin_I(ii).Centroid;
            struct.movement=compare_BBs_2D(struct.framecoords,last_bin_I,curr_bin_I);
            index=findMatch(superstructure,struct,0,0);
            if index==-1
                jj=length(superstructure)+1;
                superstructure=add2super(superstructure);
                superstructure=attach2super(superstructure,jj,struct);
                index=length(superstructure);
            else
                superstructure=attach2super(superstructure,index,struct);
            end
        end
        superstructure=checkMissing(superstructure);
    end
    
    if frameCount~=1
        display_worms(superstructure,curr_I,tl);
    end
    last_bin_I=curr_bin_I;
    last_objs=objs_curr_bin_I;
    last_super=superstructure;
    frameCount=frameCount+1;
    
    % debugging
    if show_binarized==true
        figure;
        imshow(curr_bin_I);
        title('Binarized');
    end
end
close(avi);

%% Binarize Image Ahead of Object Detection
function [bin_I]=binarize_I(I,tl,bl,tr)
[r,c]=size(I);
%I = imcrop(I, [590,0,c-750,r]); %% For Video 15 [610, 170, 1105, 630]
%I = imcrop(I, [380,0,c-380,r]); %% For Video 4 [463, 90, 1410, 839]
I = imcrop(I, [tl(1),tl(2),tr(1)-tl(1),bl(2)-tl(2)]);
I = imadjust(I,[],[],10);
bin_I = imbinarize(I);
bin_I = bwareafilt(bin_I,[40 300]);
end


%% Object Detection
function [objects_array]=object_detection(bin_I)
objects_array=regionprops(bin_I);
end


%% Only give the worms from the objects detected
function [worms_only]=classify_objects(blobs)
worms_only=blobs;  % currently, do not classify anything
% we will need something here! (because of the day 15 video)
end


%% 
function [quant]=compare_BBs_2D(BB_last,last_bin_I,bin_I)
% may add padding around bounding box for more space to work with !!! TODO
loc=round(BB_last);         % get location of bounding box
x=loc(1); y=loc(2); w=loc(3); h=loc(4); % put location in variables
BB_bin_I=imcrop(bin_I,[x y w h]);       % crop current image
BB_last_I=imcrop(last_bin_I,[x y w h]); % crop last image
diff_I=xor(BB_last_I,BB_bin_I);         % find the differences
quant=sum(sum(diff_I));                 % count the differences
end


%% Display worms and movements on screen
function []=display_worms(superstruct,I,tl)
imshow(I);

for ii=1:length(superstruct)
    loc=superstruct(ii).framecoords;
    color=superstruct(ii).color;
    rectangle('Position', [loc(1)+tl(1), loc(2)+tl(2), loc(3), loc(4)],...
      'EdgeColor',color, 'LineWidth', 1)
    text(10, 20, 'Total Movement: '+string(round(sum([superstruct.avgmovement]),2)),...
      'FontSize', 14, 'Color','black','FontName','Courier');
    text(10, 60, '  Avg Movement: '+string(round(sum([superstruct.avgmovement])/length(superstruct),2)),...
      'FontSize', 14, 'Color','black','FontName','Courier');
end
pause(.001)
global avi;
fig = getframe(gcf);
writeVideo(avi,fig);
fig = getframe(gcf);
writeVideo(avi,fig);
end

function [structure]=worm_type()
structure.movement=0;
structure.linmovement=0;
structure.avgmovement=0;
structure.framecoords=0;
structure.centroid=[0 0];
structure.lastcentroid=0;
structure.color='blue';
structure.updated=false;
structure.counts=1;
structure.missingcount=0;
end

function [structure]=regionprop2structure(rp)
structure=worm_type();
structure.framecoords=rp.BoundingBox;
structure.centroid=rp.Centroid;
end

function [superstructure]=checkMissing(supstructure)
for ii=1:length(supstructure)
    if supstructure(ii).updated==false
       supstructure(ii).missingcount=supstructure(ii).missingcount+1; 
    end
end
supstructure=cleanUp(supstructure);
superstructure=resetStructure(supstructure);
end

function [superstructure]=resetStructure(supstructure)
for ii=1:length(supstructure)
   supstructure(ii).updated=false;
end
superstructure=supstructure;
end

function [superstructure]=cleanUp(supstructure)
newstructure=supstructure;
delCounter=1;
toBeDeleted=[];
for ii=1:length(supstructure)
    if supstructure(ii).missingcount>2
        toBeDeleted(delCounter)=ii;
        delCounter=delCounter+1;
    end
end
if length(toBeDeleted)>=1
    for ii=fliplr(1:length(toBeDeleted))
       newstructure(toBeDeleted(ii))=[];
    end
end
superstructure=newstructure;
end

function [supstructure]=attach2super(supstructure,index,new_structure)
supstructure(index).movement=supstructure(index).movement+new_structure.movement;
supstructure(index).framecoords=new_structure.framecoords;
supstructure(index).lastcentroid=supstructure(index).centroid;
supstructure(index).centroid=new_structure.centroid;
[newColor,rollingAvg]=getNewColor(supstructure(index).color,supstructure(index).counts,supstructure(index).avgmovement,new_structure.movement,supstructure(index).linmovement);
supstructure(index).color=newColor;
supstructure(index).avgmovement=rollingAvg;
supstructure(index).linmovement=linearMovement(supstructure(index).centroid,supstructure(index).lastcentroid);
supstructure(index).counts=supstructure(index).counts+1;
supstructure(index).updated=true;
supstructure(index).missingcount=0;
end

function [superstructure]=add2super(supstructure)
index=length(supstructure)+1;
superstructure=supstructure;
superstructure(index)=worm_type();
end

function [index]=findMatch(supstructure,new_structure,padx,pady)
for ii=1:length(supstructure)
   xvec1=supstructure(ii).framecoords(1);
   xvec2=xvec1+supstructure(ii).framecoords(3);
   yvec1=supstructure(ii).framecoords(2);
   yvec2=yvec1+supstructure(ii).framecoords(4);
   cen=new_structure.centroid;
   cenx=cen(1);
   ceny=cen(2);
   if (cenx<=(xvec2+padx)&&cenx>=(xvec1-padx))&&(ceny<=(yvec2+pady)&&ceny>=(yvec1-pady))
      index=ii;
      return;
   end
end
index=-1;
end

function [linmovement]=linearMovement(centroid,lastcentroid)
p1x=lastcentroid(1);
p1y=lastcentroid(2);
p2x=centroid(1);
p2y=centroid(2);
if p1x==0&&p1y==0||p2x==0&&p2y==0
    linmovement=0;
    return;
end
linmovement=sqrt((p2x-p1x)^2+(p2y-p1y)^2);
end

function [newColor,m]=getNewColor(color,counts,avgmovement,new_movement,linmovement)
m=rollAvg(counts,avgmovement,.3*new_movement+linmovement);
k=[2 4 6 8];
if m>=0&&m<=k(1)
newColor='blue';
elseif m>k(1)&&m<=k(2)
newColor='green';
elseif m>k(2)&&m<=k(3)
newColor='yellow';
elseif m>k(3)&&m<=k(4)
newColor='#EDB120';
else
newColor='red';
end
end

function [average]=rollAvg(counts,avgmovement,new_movement)
average=avgmovement+(new_movement-avgmovement)/counts;
end