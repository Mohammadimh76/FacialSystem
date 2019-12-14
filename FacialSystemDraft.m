function varargout = FacialSystemDraft(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @FacialSystemDraft_OpeningFcn, ...
    'gui_OutputFcn',  @FacialSystemDraft_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function FacialSystemDraft_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

guidata(hObject, handles);

function varargout = FacialSystemDraft_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

% --- Executes on button press in LoadImage.
function LoadImage_Callback(hObject, eventdata, handles)
global I
Image=imgetfile();
I=imread(Image);
axes(handles.axes1);
imshow(I);
set(handles.text1,'string','');

% --- Executes on button press in Emotion.
function Emotion_Callback(hObject, eventdata, handles)
global I
imshow(I); hold on; warning('off')% Shows the loaded image
% Uses the Viola-Jones algorithm to detect mouth
detector = vision.CascadeObjectDetector('Mouth');
for TH=10:40:500                      % The detector can detect multiple parts
    detector.MergeThreshold = TH;     % however only one is the right part for
    BB=step(detector,I);              % mouth so the detection is made on increasing
    if size(BB,1)==1,break,end        % steps for accuracy till only one part is detected
end
% Draw a red rectangle around detected mouth area
rectangle('Position',BB,'LineWidth',2,'LineStyle','-','EdgeColor','r');
mouth=imcrop(I,BB); % Croping the whole image to mouth only
% Detect corner points
points = detectHarrisFeatures(rgb2gray(mouth)); points=points.Location;
points(:,1)=BB(1)+points(:,1); points(:,2)=BB(2)+points(:,2);
plot(points(:,1),points(:,2),'y.') % Showing corner points on image in yellow
P = polyfit(points(:,1),points(:,2),2); % Fiting the points to a parapola
% Plotting the fitted parapola on image
x = linspace(BB(1),BB(1)+BB(3),30); y=polyval(P,x); plot(x,y)
if P(1)<0 % Parapola is upward
    set(handles.text1,'string','Smiling');
else % Parapola is invertered
    set(handles.text1,'string','Frowning');
end
hold off; warning('on')

% --- Executes on button press in EyeColour.
function EyeColour_Callback(hObject, eventdata, handles)
global I
imshow(I);
% Create detector of eyes pair
EyeDetect = vision.CascadeObjectDetector('EyePairBig');
% Changing accuracy in steps to detect one part only
for TH=0:2:20
    EyeDetect.MergeThreshold = TH;
    BB=step(EyeDetect,I);
    if size(BB,1)==1,break,end
end
%Draw Blue rectangle around eyes pair
rectangle('Position',BB,'LineWidth',2,'LineStyle','-','EdgeColor','b');
Eyes=imcrop(I,BB); % Crop image to small image of two eyes
% Create detector of each eye and search smaller image of eyes pair
EyeDetect = vision.CascadeObjectDetector('LeftEye');
% Changing accuracy in steps to detect two part only
for TH=0:1:10 
    EyeDetect.MergeThreshold = TH;
    B=step(EyeDetect,Eyes);
    if size(B,1)==2,break,end
end
% B1 and B2 are smaller rectangles ofr each eye
B1=B(1,:); B1(1)=B1(1)+BB(1); B1(2)=B1(2)+BB(2); 
%Draw yellow rectangle around eye
rectangle('Position',B1,'LineWidth',0.1,'LineStyle','-','EdgeColor','y');
B1(1)=B1(1)+0.3*B1(3); B1(2)=B1(2)+0.3*B1(4);
B1(3)=0.4*B1(3); B1(4)=0.4*B1(4);
Leye=imcrop(I,B1); % Crop image for image of one eye
B2=B(end,:); B2(1)=B2(1)+BB(1); B2(2)=B2(2)+BB(2);
%Draw yellow rectangle around eye
rectangle('Position',B2,'LineWidth',0.1,'LineStyle','-','EdgeColor','y');
B2(1)=B2(1)+0.3*B2(3); B2(2)=B2(2)+0.3*B2(4);
B2(3)=0.4*B2(3); B2(4)=0.4*B2(4);
Reye=imcrop(I,B2); % Crop image for image of the other eye
% Transferring the two eyes image matrices into two long vectors
tmp1=reshape(Reye(:,:,1),[1,numel(Reye(:,:,1))]);
tmp1=[tmp1 reshape(Leye(:,:,1),[1,numel(Leye(:,:,1))])];
tmp2=reshape(Reye(:,:,2),[1,numel(Reye(:,:,2))]);
tmp2=[tmp2 reshape(Leye(:,:,2),[1,numel(Leye(:,:,2))])];
tmp3=reshape(Reye(:,:,3),[1,numel(Reye(:,:,3))]);
tmp3=[tmp3 reshape(Leye(:,:,3),[1,numel(Leye(:,:,3))])];
% Detect color from eyes images by looking for most repeated
% pixel color in long vectors created using mode function
RGB = [0 0 0]; % This vector will contain eyes color in RGB format
RGB(1)=double(mode(tmp1));
RGB(2)=double(mode(tmp2));
RGB(3)=double(mode(tmp3));
% Transferring RGB format into a color name
if RGB(1)< (255/4) && RGB(2)< (255/4) && RGB(3)< (255/4)
    s='Black';
else
    RGB=(RGB == max(RGB));
    if RGB(1), s='Brown';
    elseif RGB(2), s='Green';
    else s='Blue'; 
    end
end
% Showing the color name as a result
set(handles.text1,'string',s);

% --- Executes on button press in Greyscale.
function Greyscale_Callback(hObject, eventdata, handles)
global I
A=rgb2gray(I);
imshow(A);
s=('Greyscale');
set(handles.text1,'string',s);
% --- Executes on button press in HSV.
function HSV_Callback(hObject, eventdata, handles)
% hObject    handle to HSV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I
A = rgb2hsv(I);
imagesc(A);
imshow(A);
s=('Hue, Saturation, Value');
set(handles.text1,'string',s);

% --- Executes on button press in Lab.
function Lab_Callback(hObject, eventdata, handles)
% hObject    handle to Lab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I
A = rgb2lab(I);
imagesc(A);
imshow(A);
s=('Lightness & Color Dimensions');
set(handles.text1,'string',s);


% --- Executes on button press in Original.
function Original_Callback(hObject, eventdata, handles)
% hObject    handle to Original (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global I
imshow(I);
s=('Original');
set(handles.text1,'string',s);
