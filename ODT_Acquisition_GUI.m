function varargout = ODT_Acquisition_GUI(varargin)
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @controlPanel_Erlangen_ximea_OpeningFcn, ...
    'gui_OutputFcn',  @controlPanel_Erlangen_ximea_OutputFcn, ...
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

function controlPanel_Erlangen_ximea_OpeningFcn(hObject, eventdata, handles, varargin)
global res; global NA;global lambda;global folderFlag;global timeLapseNum;
global mtbid; global rootZeiss; global defaultPosition; global connection;
folderFlag=0;
timeLapseNum=0;

NA=1.2;lambda=0.532;
res=5.5/(64.8968); 
warning('off')
handles.output = hObject;
g=gpuDevice(1);
reset(g);

imaqreset
set(handles.pixelSize,'userdata',5.5);
set(handles.NAscan,'userdata',NA);
set(handles.Wavelength,'userdata',532);
set(handles.radius,'userdata',0.25);
set(handles.durationTime,'userdata',30);
set(handles.stepNum,'userdata',30);
set(handles.exposureControl,'userdata',1); % ExposureTime (ms)
set(handles.mag,'userdata',5.5/res);
set(handles.stopBtn,'userdata',1);
set(handles.gainControl,'userdata',6);
set(handles.brightnessControl,'userdata',0);
set(handles.frameNumControl,'userdata',150); % FrameRate (Hz)
set(handles.fluoGain,'userdata',6);
set(handles.fluoNumFrame,'userdata',1);

set(handles.pixelSize,'String',get(handles.pixelSize,'userdata'));
set(handles.NAscan,'String',get(handles.NAscan,'userdata'));
set(handles.Wavelength,'String',get(handles.Wavelength,'userdata'));
set(handles.radius,'String',get(handles.radius,'userdata'));
set(handles.durationTime,'String',get(handles.durationTime,'userdata'));
set(handles.stepNum,'String',get(handles.stepNum,'userdata'));
set(handles.exposureControl,'String',get(handles.exposureControl,'userdata'));
set(handles.mag,'String',get(handles.mag,'userdata'));
set(handles.gainControl,'String',get(handles.gainControl,'userdata'));
set(handles.brightnessControl,'String',get(handles.brightnessControl,'userdata'));
set(handles.imageFinder,'sliderstep',[1/150 10/150]);
set(handles.frameNumControl,'String',get(handles.frameNumControl,'userdata'));
set(handles.fluoNumFrame,'string',get(handles.fluoNumFrame,'userdata'));

set(handles.timeLapseSelected,'value',0);
set(handles.totalNumTimeLapse,'userdata',1);
set(handles.lapseDurationTime,'userdata',0);
set(handles.numLapse,'userdata',1);
set(handles.totalNumTimeLapse,'string','1');
set(handles.lapseDurationTime,'string',get(handles.lapseDurationTime,'userdata'));
set(handles.numLapse,'string',get(handles.numLapse,'userdata'));
set(handles.minPhase,'string',num2str(-3));
set(handles.maxPhase,'string',num2str(3));
set(handles.fluoExp,'string','900');
set(handles.rExposure,'string','900');
set(handles.gExposure,'string','900');
set(handles.bExposure,'string','900');
set(handles.laserFlag,'userdata',1);
set(handles.fluoGain,'string',num2str(get(handles.fluoGain,'userdata')));
set(handles.xPos,'userdata',zeros(1,3));
set(handles.lateralDist,'string','40');
set(handles.AxialDist,'string','0.5')

vid = videoinput('gentl', 1, 'Mono8');
set(handles.startPreview,'userdata',vid);
cameraInitial(hObject, eventdata, handles)

cl = NET.addAssembly('MTBApi');
import ZEISS.MTB.Api.*
connection = ZEISS.MTB.Api.MTBConnection();
mtbid = connection.Login('en');
rootZeiss = connection.GetRoot(mtbid);
deviceCount = rootZeiss.GetDeviceCount();
device=rootZeiss.GetDevice(0);

reflectorChanger = rootZeiss.GetComponent(char("MTBReflectorChanger"));
reflectorChanger.SetPosition(mtbid,4,ZEISS.MTB.Api.MTBCmdSetModes.Default);

StageAxis=rootZeiss.GetComponent(char("MTBStage"));
FocusAxis=rootZeiss.GetComponent(char("MTBFocus"));
defaultPosition=zeros(1,3);
[defaultX,defaultY]=StageAxis.GetPosition("µm");
defaultPosition(1:2)=[defaultX,defaultY];
defaultPosition(3)=FocusAxis.GetPosition("µm");
set(handles.xPos,'string','0');
set(handles.yPos,'string','0');
set(handles.zPos,'string','0');

set(handles.posList,'data',[]);

guidata(hObject, handles);

function varargout = controlPanel_Erlangen_ximea_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

function cameraInitial(hObject, eventdata, handles)
vid=get(handles.startPreview,'userdata');
vid.ROIPosition = [428 430 1188 1188];
src=getselectedsource(vid);

src.ExposureTime = 1000;
src.AcquisitionTimingMode = 'FrameRate';
src.AcquisitionFrameRate = 150;

function connectGalvo_Callback(hObject, eventdata, handles)
global session;
set(handles.galvoStatus,'string','..Connecting');
session=daq.createSession('ni');
addAnalogOutputChannel(session,'Dev1',0:1, 'Voltage');
addDigitalChannel(session,'dev1', 'Port0/Line0:1', 'OutputOnly');
set(handles.galvoStatus,'string','Connected');
outputSingleScan(session,[0 0 0 1]);

function defaultPosition_Callback(hObject, eventdata, handles)
global session;
if get(hObject,'Value')==1
    outputSingleScan(session,[0 0 0 1]);
end
startPreview_Callback(hObject, eventdata, handles)

function galvoAlignment_Callback(hObject, eventdata, handles)
global session;
session.Rate=1000;
vid=get(handles.startPreview,'userdata');
vidRes = get(vid, 'ROIPosition');
imWidth = vidRes(3);
imHeight = vidRes(4);
nBands = get(vid, 'NumberOfBands');
axes(handles.previewPanel);
hImage = imagesc( zeros(imHeight, imWidth, nBands) );
preview(vid, hImage);
% preview(vid);
r=get(handles.radius,'userdata');
stepNum=get(handles.stepNum,'userdata');
durationTime=get(handles.durationTime,'userdata');
theta=linspace(0,360,stepNum);
x=r.*cos(theta*pi/180);
y=r.*sin(theta*pi/180);
axes(handles.galvoPlot);
plot(x,y,'or'),axis image

repNum=durationTime;
scanVolt2=zeros(length(x)*repNum,4);
for kk=1:stepNum
    scanVolt2((kk-1)*repNum+1:(kk-1)*repNum+repNum,1:2)=repmat([x(kk),y(kk)],[repNum 1]);
    scanVolt2((kk-1)*repNum+3,3)=1;
end
scanVolt2(:,4)=1;
for n=1:10000
    queueOutputData(session,scanVolt2)
    startBackground(session);
    pause(0.001*durationTime*(stepNum+5))
    if get(handles.stopBtn,'userdata')==0
        stoppreview(vid);
        break;
    end
end
set(handles.stopBtn,'UserData',1);
startPreview_Callback(hObject, eventdata, handles)

function stopBtn_Callback(hObject, eventdata, handles)
set(handles.stopBtn,'UserData',0);

function durationTime_Callback(hObject, eventdata, handles)
durationTime=str2double(get(hObject,'String'));
set(handles.durationTime,'userdata',durationTime);

function durationTime_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function stepNum_Callback(hObject, eventdata, handles)
stepNum=str2double(get(hObject,'String'));
set(handles.stepNum,'userdata',stepNum);

function stepNum_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function radius_Callback(hObject, eventdata, handles)
radius=str2double(get(hObject,'String'));
set(handles.radius,'userdata',radius);

function radius_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function scanBtn_Callback(hObject, eventdata, handles)
global scanVolt; global session;
global res; global NA;global lambda;global timeLapseNum;global timeLapseBatchNum;
session.Rate=1000;

stopPreview_Callback(hObject, eventdata, handles)
pause(0.2);
set(handles.currentNum,'string','0');
set(handles.acquisitionMsg,'string','Tomography Acquisition Processing');
vid=get(handles.startPreview,'userdata');
stoppreview(vid);
src=getselectedsource(vid);
src.Gain = get(handles.gainControl,'userdata');
src.AcquisitionFrameRate = get(handles.frameNumControl,'userdata');
src.ExposureTime = get(handles.exposureControl,'userdata')*1000;
src.Gain = get(handles.gainControl,'userdata');

stoppreview(vid);
set(handles.previewMsg,'string','Stop Preview');
drawnow
set(vid, 'FramesPerTrigger', 1);
vid.TriggerRepeat = Inf;
triggerconfig(vid, 'hardware', 'DeviceSpecific', 'DeviceSpecific');
src.TriggerMode = 'On';
src.TriggerActivation = 'RisingEdge';
src.TriggerSelector = 'FrameStart';
src.TriggerSource = 'Line1';
readJoystick_Callback(hObject, eventdata, handles)
positionVal=get(handles.xPos,'userdata');

if ~get(handles.timeLapseSelected,'value')
    start(vid);
    outputSingleScan(session,[0 0 0 1])
    outputSingleScan(session,scanVolt(1,:));
    pause(0.1);
    queueOutputData(session,scanVolt)
    startForeground(session);
    pause(0.75);
    set(handles.currentNum,'string','1');
    stepNum=get(handles.stepNum,'userdata');
    tomogMap=getdata(vid,stepNum);
    tomogMap=squeeze(tomogMap(:,:,1,:));
    
    set(handles.scanBtn,'userdata',tomogMap);
    %
    outputSingleScan(session,[0 0 0 1])
    set(handles.laserFlag,'userdata',0);
    flushdata(vid)
    folderPath=get(handles.folderPath,'UserData');
    dateTimeVal=datetime;
    cd(folderPath);
    sampleList=dir('sample*_Tomog.mat');
    if isempty(sampleList)==1
        save('sample001_Tomog.mat','tomogMap','res','lambda','NA','dateTimeVal');
    else
        save(strcat('sample',sprintf('%03d',length(sampleList)+1),'_Tomog.mat'),'tomogMap','positionVal','res','lambda','NA','dateTimeVal');
    end
    
    axes(handles.tomogCheck);
    imagesc(squeeze(tomogMap(:,end/2,:))')
    
    axes(handles.tomogCheckProf);
    temp=mean(squeeze(mean(abs(tomogMap),1)));
    plot(temp,'or')
    ylim([0 255])
    camroll(-90)
    
    set(handles.acquisitionMsg,'string',strcat('Tomography Completed '));
    outputSingleScan(session,[0 0 0 1])
    
    stop(vid);
    triggerconfig(vid, 'immediate');
    vid.TriggerRepeat = 0;
else
    %
    delayTime=str2double(get(handles.lapseDurationTime,'userdata'));
    stepNum=get(handles.stepNum,'userdata');
    folderPath=get(handles.folderPath,'UserData');
    cd(folderPath);
    sampleList=dir('sample*_TimeLapse_001_Tomog.mat');
    for timeLapseNum=1:get(handles.numLapse,'userdata')
        tic
        start(vid);
        if get(handles.laserFlag,'userdata')==0
            outputSingleScan(session,[scanVolt(1,1) scanVolt(1,2) 0 1])
            set(handles.laserFlag,'userdata',1);
        end
%         pause(0.25)
        pause(get(handles.durationTime,'userdata')*0.001);
        queueOutputData(session,scanVolt)
        startForeground(session);
        %         pause(0.1);
        set(handles.currentNum,'string',num2str(timeLapseNum));
        drawnow
        outputSingleScan(session,[scanVolt(1,1) scanVolt(1,2) 0 1])
        set(handles.laserFlag,'userdata',0);
        
        tomogMap=getdata(vid,stepNum);
        tomogMap=squeeze(tomogMap(:,:,1,:));
        flushdata(vid)
        
        dateTimeVal=datetime;
        if isempty(sampleList)==1
            save(strcat('sample001','_TimeLapse_',sprintf('%03d',(timeLapseNum)),'_Tomog.mat'),'tomogMap','positionVal','dateTimeVal','res','lambda','NA');
            timeLapseBatchNum=1;
        else
            save(strcat('sample',sprintf('%03d',length(sampleList)+1),'_TimeLapse_',sprintf('%03d',(timeLapseNum)),'_Tomog.mat'),'tomogMap','positionVal','dateTimeVal','res','lambda','NA');
            timeLapseBatchNum=length(sampleList)+1;
        end
        
        channelId=[get(handles.bCheck,'value') get(handles.gCheck,'value') get(handles.rCheck,'value')];
        stop(vid);
        t=toc;
        pause(delayTime-t);
    end
    
    outputSingleScan(session,[0 0 0 0])
    
    axes(handles.tomogCheck);
    imagesc(squeeze(tomogMap(:,end/2,:))')
    
    axes(handles.tomogCheckProf);
    temp=mean(squeeze(mean(abs(tomogMap),1)));
    plot(temp,'or')
    ylim([0 255])
    camroll(-90)
    
    timeLapseNum=0;
    set(handles.acquisitionMsg,'string',strcat('Tomography Lapse Completed '));
    triggerconfig(vid, 'immediate');
    vid.TriggerRepeat = 0;
    src.TriggerMode = 'Off';
    
end

function fastTomography_Callback(hObject, eventdata, handles)
global scanVolt; global session;global res; global NA;global lambda;
session.Rate=1000;
set(handles.currentNum,'string','0');

set(handles.acquisitionMsg,'string','Tomography Acquisition Processing');
vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
src.Gain = get(handles.gainControl,'userdata');
stoppreview(vid);
set(handles.previewMsg,'string','Stop Preview');
drawnow
delayTime=str2double(get(handles.lapseDurationTime,'userdata'));
stepNum=get(handles.stepNum,'userdata');

vid.TriggerRepeat = stepNum*get(handles.numLapse,'userdata')-1;
vid.FramesPerTrigger = 1;
triggerconfig(vid, 'hardware', 'DeviceSpecific', 'DeviceSpecific');
src.TriggerMode = 'On';
src.TriggerActivation = 'RisingEdge';
src.TriggerSelector = 'FrameStart';
src.TriggerSource = 'Line1';

start(vid);

scanVolt2=scanVolt;
scanVolt2(end+1:delayTime*1000,:)=repmat(scanVolt(1,:),[delayTime*1000-size(scanVolt,1) 1]);
outputSingleScan(session,[scanVolt2(1,1:2), [0 1]]);
queueOutputData(session,repmat(scanVolt2,[get(handles.numLapse,'userdata') 1]))
startForeground(session);

pause(3);
set(handles.acquisitionMsg,'string',strcat('Acquisition Completed '));

outputSingleScan(session,[0 0 0 0])
% tomogramImageCheck_Callback(hObject, eventdata, handles);
tempBatch=getdata(vid,stepNum*get(handles.numLapse,'userdata'));
tempBatch=squeeze(tempBatch(:,:,1,:));

flushdata(vid)
folderPath=get(handles.folderPath,'UserData');

cd(folderPath);
sampleList=dir('sample*_fastScan_001_Tomog.mat');

for kk=1:get(handles.saveBtn,'userdata')
    tomogMap=tempBatch(:,:,((kk-1)*stepNum+1):((kk-1)*stepNum+stepNum));
    set(handles.currentNum,'string',num2str(kk));
    drawnow
    if isempty(sampleList)==1
        save(strcat('sample001','_fastScan_',sprintf('%03d',(kk)),'_Tomog.mat'),'tomogMap','delayTime','res','lambda','NA');
    else
        save(strcat('sample',sprintf('%03d',length(sampleList)+1),'_fastScan_',sprintf('%03d',(kk)),'_Tomog.mat'),'tomogMap','delayTime','res','lambda','NA');
    end
end

axes(handles.tomogCheck);
imagesc(squeeze(tomogMap(:,end/2,:))')

axes(handles.tomogCheckProf);
temp=mean(squeeze(mean(abs(tomogMap),1)));
plot(temp,'or')
ylim([0 255])
camroll(-90)

set(handles.scanBtn,'userdata',tomogMap);
set(handles.acquisitionMsg,'string',strcat('Tomography Completed '));

stop(vid);
triggerconfig(vid, 'immediate');
src.TriggerMode = 'Off';
vid.TriggerRepeat = 0;

function circularScanning_Callback(hObject, eventdata, handles)

function scanningSelection_SelectionChangedFcn(hObject, eventdata, handles)
global scanVolt;
radius=get(handles.radius,'userdata');
stepNum=get(handles.stepNum,'userdata');

switch hObject
    case handles.circularScanning
        theta=linspace(0,360,stepNum+1)';
        x=radius.*cos(theta*pi/180);
        y=radius.*sin(theta*pi/180);
        x=x(1:end-1);y=y(1:end-1);
    case handles.spiralScanning
        N3=round(stepNum/3);
        theta=linspace(2*pi,0,N3);
        theta=theta(2:end);
        r=sqrt(theta);
        x=r.*cos(theta)/sqrt(2*pi);
        y=r.*sin(theta)/sqrt(2*pi);
        
        theta=linspace(0,2*pi,N3);
        theta=theta(2:end-1);
        r=sqrt(theta);
        x=[x -r.*cos(theta)/sqrt(2*pi)];
        y=[y -r.*sin(theta)/sqrt(2*pi)];
        
        theta=linspace(0,2*pi,stepNum-2*N3+3);
        theta=linspace(0,theta(end-1),stepNum-2*N3+3);
        x=[x -cos(theta)];
        y=[y -sin(theta)];
        x=radius*x;
        y=radius*y;
end
axes(handles.galvoPlot);
plot(x,y,'or'),axis image
repNum=8;
scanVolt=zeros(length(x)*repNum,4);
for kk=1:stepNum
    scanVolt((kk-1)*repNum+1:(kk-1)*repNum+repNum,1:2)=repmat([x(kk),y(kk)],[repNum 1]);
    scanVolt((kk-1)*repNum+3,3)=1;
end
scanVolt(:,4)=1;
drawnow

function figure1_CloseRequestFcn(hObject, eventdata, handles)
global connection; global mtbid;
vid=get(handles.startPreview,'userdata');
delete(vid);
connection.Logout(mtbid);
delete(hObject);

function timeLapseSelected_Callback(hObject, eventdata, handles)

function lapseDurationTime_Callback(hObject, eventdata, handles)
set(handles.lapseDurationTime,'userdata',get(handles.lapseDurationTime,'string'));

function lapseDurationTime_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function numLapse_Callback(hObject, eventdata, handles)
set(handles.numLapse,'userdata',str2double(get(handles.numLapse,'string')));
set(handles.totalNumTimeLapse,'string',str2double(get(handles.numLapse,'string')));

function numLapse_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function folderPath_Callback(hObject, eventdata, handles)
global folderFlag;global folderPath;
if folderFlag==0
    folderPath = uigetdir('E:\Kyoo\002_Data','Select Folder');
    set(handles.folderPath,'UserData',folderPath);
    folderFlag=1;
else
    folderPath = uigetdir(folderPath,'Select Folder');
    set(handles.folderPath,'UserData',folderPath);
end
set(handles.folderPathMsg,'string',folderPath);

function imageFinder_Callback(hObject, eventdata, handles)
tomogMap=get(handles.scanBtn,'userdata');
[xx yy frame]=size(tomogMap);
sliderVal=get(hObject,'value');
sliderVal=ceil(frame*sliderVal);
if sliderVal==0
    sliderVal=1;
end
axes(handles.previewPanel);
imagesc(squeeze(tomogMap(:,:,sliderVal)),[0 max(max(max(tomogMap)))]),colormap('gray'), axis off, axis image
title(strcat('Frame # : ',num2str(sliderVal)))

function imageFinder_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function tomogramImageCheck_Callback(hObject, eventdata, handles)
tomogMap=get(handles.scanBtn,'userdata');
[xx yy frame]=size(tomogMap);

axes(handles.previewPanel);
for ii=1:frame
    imagesc(squeeze(tomogMap(:,:,ii)),[0 max(max(max(tomogMap)))]),colormap('gray'), axis off, axis image
    title(strcat('Frame # : ',num2str(ii)))
    pause(0.03)
end

function startPreview_Callback(hObject, eventdata, handles)
global res; global NA;global lambda;global mi;global mj;global session;
outputSingleScan(session,[0 0 0 1])

set(handles.stopPreview,'userdata',0);
vid=get(handles.startPreview,'userdata');
vidRes = get(vid, 'ROIPosition');
imWidth = vidRes(3);
imHeight = vidRes(4);
nBands = get(vid, 'NumberOfBands');
src=getselectedsource(vid);

src.AcquisitionTimingMode = 'FrameRate';
src.ExposureTime = get(handles.exposureControl,'userdata')*1000;
src.AcquisitionFrameRate = get(handles.frameNumControl,'userdata');
src.Gain = get(handles.gainControl,'userdata');
src.AcquisitionTimingMode = 'FrameRate';
triggerconfig(vid, 'manual');

try
    if (get(handles.phaseCheck,'value')~=1)&&(get(handles.fourierSpec,'value')~=1)&&(get(handles.DICcheck,'value')~=1)
        axes(handles.previewPanel);
        hImage = imagesc( zeros(imHeight, imWidth, nBands) );
        preview(vid, hImage);
    else
        start(vid)
        while 1
            if get(handles.stopPreview,'userdata')==1
                break;
            end
            drawnow;
            nBuf = getsnapshot(vid);
            if get(handles.phaseCheck,'value')==1
                f_bg=(get(handles.takeBg,'userdata'));
                nBuf=(nBuf);
                Fimg = fftshift(fft2(nBuf))/(size(nBuf,1).^2);
                
                r=round(size(nBuf,1)*res*NA/lambda);
                yr = r;
                c1mask = ~(mk_ellipse(r,yr,size(nBuf,1),size(nBuf,1)));
                c2mask=~c1mask;
                c3mask = circshift(c1mask,[mi mj]).*c2mask;
                Fimg=Fimg.*c3mask;
                Fimg = circshift(Fimg,-[mi mj]);
                Fimg = ifft2(ifftshift(Fimg))*(size(nBuf,1).^2);
                f = Fimg./f_bg;
                p = single(unwrap2(double(((((angle(f))))))));
                p= p-median(median(p));
                
                minPhase=str2double(get(handles.minPhase,'string'));
                maxPhase=str2double(get(handles.maxPhase,'string'));
                axes(handles.previewPanel)                imagesc(((1:size(nBuf,1))-size(nBuf,1)/2)*res,((1:size(nBuf,1))-size(nBuf,1)/2)*res,p,[minPhase maxPhase]),axis image,colormap('jet');
                xlabel('\mum')
                ylabel('\mum')
                set(gca,'xgrid', 'on', 'ygrid', 'on', 'gridlinestyle', '-', 'xcolor', 'k', 'ycolor', 'k');        % imagesc(log10(abs(Fimg))),axis image,colormap('jet')
                %%
            elseif get(handles.fourierSpec,'value')==1
                nBuf=(nBuf);
                Fimg = fftshift(fft2(nBuf))/(size(nBuf,1).^2);
                axes(handles.previewPanel)
                imagesc(log10(abs(Fimg))),axis image,colormap('jet')
                set(gca,'xgrid', 'on', 'ygrid', 'on', 'gridlinestyle', '-', 'xcolor', 'k', 'ycolor', 'k');        % imagesc(log10(abs(Fimg))),axis image,colormap('jet')
            elseif get(handles.DICcheck,'value')==1
                f_bg=(get(handles.takeBg,'userdata'));
                nBuf=(nBuf);
                Fimg = fftshift(fft2(nBuf))/(size(nBuf,1).^2);
                r=round(size(nBuf,1)*res*NA/lambda);
                yr = r;
                c1mask = ~(mk_ellipse(r,yr,size(nBuf,1),size(nBuf,1)));
                c2mask=~c1mask;
                c3mask = circshift(c1mask,[mi mj]).*c2mask;
                Fimg=Fimg.*c3mask;
                Fimg = circshift(Fimg,-[mi mj]);
                Fimg = ifft2(ifftshift(Fimg))*(size(nBuf,1).^2);
                f = Fimg./f_bg;
                f = (f(3:end-2,3:end-2));
                p = single(PhiShift(PhiShift((angle(f)))));
                p= p-median(median(p));
                p=p-circshift(p,[1 1]*round(lambda/2/NA/res));
                minPhase=str2double(get(handles.minPhase,'string'));
                maxPhase=str2double(get(handles.maxPhase,'string'));
                axes(handles.previewPanel)
                imagesc(((1:size(nBuf,1))-size(nBuf,1)/2)*res,((1:size(nBuf,1))-size(nBuf,1)/2)*res,p,[minPhase maxPhase]),axis image,colormap('gray');
                xlabel('\mum')
                ylabel('\mum')
                set(gca,'xgrid', 'on', 'ygrid', 'on', 'gridlinestyle', '-', 'xcolor', 'k', 'ycolor', 'k');        % imagesc(log10(abs(Fimg))),axis image,colormap('jet')
            else
            end
            
            if get(handles.stopPreview,'userdata')==1
                stop(vid)
                break;
            end
            drawnow
        end
        stop(vid)
    end
catch
    disp('Preview Error')
end

function angleCheck_Callback(hObject, eventdata, handles)
global session;global scanVolt;
outputSingleScan(session,[0 0 0 1])
set(handles.stopPreview,'userdata',0);
vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
src.ExposureTime = 1000*get(handles.exposureControl,'userdata');
src.Gain = get(handles.gainControl,'userdata');
durationTime=get(handles.durationTime,'userdata');
try
    while 1
        if get(handles.stopPreview,'userdata')==1
            stoppreview(vid);
            break;
        end
        for scanSeq=1:size(scanVolt,1)/10
            if get(handles.stopPreview,'userdata')==1
                stoppreview(vid);
                break;
            end
            outputSingleScan(session,[scanVolt(scanSeq*10,1:2) 0 1])
            set(handles.stopBtn,'userdata',scanVolt(scanSeq*10,1:2));
            drawnow;
            nBuf = getsnapshot(vid);
            nBuf=(nBuf);
            Fimg = fftshift(fft2(nBuf))/(size(nBuf,1).^2);
            axes(handles.previewPanel)
            imagesc(log10(abs(Fimg))),axis image,colormap('jet')
            set(gca,'xgrid', 'on', 'ygrid', 'on', 'gridlinestyle', '-', 'xcolor', 'k', 'ycolor', 'k');        % imagesc(log10(abs(Fimg))),axis image,colormap('jet')
            pause(durationTime*0.001)
        end
    end
catch
    disp('Preview Error')
end

function stopPreview_Callback(hObject, eventdata, handles)
global session;
vid=get(handles.startPreview,'userdata');
                stoppreview(vid);
if get(handles.laserFlag,'userdata')==1
    outputSingleScan(session,[0 0 0 1])
    set(handles.laserFlag,'userdata',0);
end
set(handles.stopPreview,'UserData',1);

function takeBg_Callback(hObject, eventdata, handles)
global res; global NA;global lambda;global mi;global mj;global session;
if get(handles.laserFlag,'userdata')==0
    set(handles.laserFlag,'userdata',1);
end
pause(0.2)
vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
src.Gain = get(handles.gainControl,'userdata');
stoppreview(vid);
nBuf = getsnapshot(vid);
nBuf=(nBuf);
Fimg = fftshift(fft2(nBuf))/(size(nBuf,1).^2);
[mi,mj]=find(Fimg==max(max(Fimg(:, round(size(nBuf,1)*0.05):round(size(nBuf,1)*0.45) ))));
mi=round(mi-size(nBuf,1)/2-1); mj=round(mj-size(nBuf,2)/2-1);
r=round(size(nBuf,1)*res*NA/lambda);
yr = r;
c1mask = ~(mk_ellipse(r,yr,size(nBuf,1),size(nBuf,1)));
c2mask=~c1mask;
c3mask = circshift(c1mask,[mi mj]).*c2mask;
Fimg=Fimg.*c3mask;
Fimg = circshift(Fimg,-[mi mj]);
Fimg = ifft2(ifftshift(Fimg))*(size(nBuf,1).^2);

set(handles.takeBg,'userdata',Fimg);
set(handles.previewMsg,'string','Taking Bg Completed');
startPreview_Callback(hObject, eventdata, handles)

outputSingleScan(session,[0 0 0 1])
set(handles.laserFlag,'userdata',0);

function phaseCheck_Callback(hObject, eventdata, handles)

function minPhase_Callback(hObject, eventdata, handles)

function minPhase_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function maxPhase_Callback(hObject, eventdata, handles)

function maxPhase_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function topographyBtn_Callback(hObject, eventdata, handles)
global session;global NA;global res;global lambda;
session.Rate=500;
set(handles.currentNum,'string','0');
set(handles.acquisitionMsg,'string','Topography Acquisition Processing');
vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
src.Gain = get(handles.gainControl,'userdata');
stoppreview(vid);
set(handles.previewMsg,'string','Stop Preview');
drawnow

triggerconfig(vid, 'immediate');
set(vid, 'FramesPerTrigger', str2double(get(handles.numTopo,'string')));
src.AcquisitionTimingMode = 'FrameRate';
src.AcquisitionFrameRate = str2double(get(handles.frequencyAcq,'string'));

outputSingleScan(session,[0 0 0 1])
start(vid);

pause(str2double(get(handles.numTopo,'string'))./(src.AcquisitionFrameRate)+1);

topogMap=getdata(vid,str2double(get(handles.numTopo,'string')));
topogMap=squeeze(topogMap(:,:,1,:));

flushdata(vid)
folderPath=get(handles.folderPath,'UserData');
cd(folderPath);
sampleList=dir('sample*_Topog.mat');
    dateTimeVal=datetime;
positionVal=get(handles.xPos,'userdata');
frameRate=str2double(get(handles.frequencyAcq,'string'));
exposureTime=get(handles.exposureControl,'userdata'); % ExposureTime (ms)
if isempty(sampleList)==1
    save('sample01_Topog.mat','topogMap','topogMap','dateTimeVal','NA','res','lambda','positionVal','frameRate','exposureTime');
elseif length(sampleList)<9
    save(strcat('sample0',num2str(length(sampleList)+1),'_Topog.mat'),'topogMap','dateTimeVal','NA','res','lambda','positionVal','frameRate','exposureTime');
else
    save(strcat('sample',num2str(length(sampleList)+1),'_Topog.mat'),'topogMap','dateTimeVal','NA','res','lambda','positionVal','frameRate','exposureTime');
end

set(handles.acquisitionMsg,'string',strcat('Topography Completed '));
stop(vid);
triggerconfig(vid, 'immediate');
set(vid, 'FramesPerTrigger', 1);
src.AcquisitionFrameRate= get(handles.frameNumControl,'userdata');

vid.TriggerRepeat = 0;

function frequencyAcq_Callback(hObject, eventdata, handles)

function frequencyAcq_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function numTopo_Callback(hObject, eventdata, handles)

function numTopo_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function fourierSpec_Callback(hObject, eventdata, handles)

function DICcheck_Callback(hObject, eventdata, handles)

function rCheck_Callback(hObject, eventdata, handles)
function gCheck_Callback(hObject, eventdata, handles)
function bCheck_Callback(hObject, eventdata, handles)

function rExposure_Callback(hObject, eventdata, handles)
function gExposure_Callback(hObject, eventdata, handles)
function bExposure_Callback(hObject, eventdata, handles)

function Bmove_Callback(hObject, eventdata, handles)
filterMove(1)

function Gmove_Callback(hObject, eventdata, handles)
filterMove(2)

function Rmove_Callback(hObject, eventdata, handles)
filterMove(3)

function filterMove(colorChannel)
global rootZeiss;global mtbid;
reflectorChanger = rootZeiss.GetComponent(char("MTBReflectorChanger"));
reflectorChanger.SetPosition(mtbid,colorChannel,ZEISS.MTB.Api.MTBCmdSetModes.Default);
pause(0.3)
reflectionShutter= rootZeiss.GetComponent(char("MTBRLShutter"));
if (reflectionShutter.ReadPosition~=2)&&(colorChannel<4)
    reflectionShutter.SetPosition(mtbid,2,ZEISS.MTB.Api.MTBCmdSetModes.Default);
end
HXPShutter=rootZeiss.GetComponent(char("MTBHXP120Shutter"));
if (HXPShutter.ReadPosition~=2)&&(colorChannel<4)
    HXPShutter.SetPosition(mtbid,2,ZEISS.MTB.Api.MTBCmdSetModes.Default);
end

function rExposure_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function gExposure_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function bExposure_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function fPreview_Callback(hObject, eventdata, handles)
global session;
outputSingleScan(session,[0.3 0.3 0 0])
vid=get(handles.startPreview,'userdata');
vidRes = get(vid, 'ROIPosition');
imWidth = vidRes(3);
imHeight = vidRes(4);
nBands = get(vid, 'NumberOfBands');
axes(handles.previewPanel);
hImage = imagesc( zeros(imHeight, imWidth, nBands),[0 255]);
src=getselectedsource(vid);
src.Gain=get(handles.fluoGain,'userdata');
src.ExposureTime = str2double(get(handles.fluoExp,'string'))*1000;
src.AcquisitionFrameRate = min(990/str2double(get(handles.fluoExp,'string')),get(handles.frameNumControl,'userdata'));
preview(vid, hImage);

function stopFpreview_Callback(hObject, eventdata, handles)
vid=get(handles.startPreview,'userdata');
stoppreview(vid);

function fluoExp_Callback(hObject, eventdata, handles)
vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
stoppreview(vid);
src.Gain=get(handles.fluoGain,'userdata');
src.ExposureTime = 1000 * str2double(get(handles.fluoExp,'string'));
src.AcquisitionFrameRate = 990/str2double(get(handles.fluoExp,'string'));

function fluoExp_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function fluorImg_Callback(hObject, eventdata, handles)
global session;global timeLapseNum;global timeLapseBatchNum;
outputSingleScan(session,[0.3 0.3 0 0])
pause(0.1)

vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
src.Gain=get(handles.fluoGain,'userdata');

stoppreview(vid);
set(handles.previewMsg,'string','Stop Preview');
drawnow

set(vid, 'FramesPerTrigger', 1);
triggerconfig(vid, 'immediate');

channelId=[get(handles.bCheck,'value') get(handles.gCheck,'value') get(handles.rCheck,'value')];
channelInd=find(channelId==1);
exposureId=[str2double(get(handles.bExposure,'string')) str2double(get(handles.gExposure,'string')) str2double(get(handles.rExposure,'string'))];

sizeVal=vid.ROIPosition;
sizeVal=sizeVal(3:4);

readJoystick_Callback(hObject, eventdata, handles)

if timeLapseNum==0
    fluoImg=zeros(sizeVal(1),sizeVal(2),sum(channelId));
    zPosTemp=str2double(get(handles.zPos,'string'));
    for kk=1:sum(channelId)
        filterMove(channelInd(kk));
        src.ExposureTime = 1000 * exposureId(channelInd(kk));
        src.AcquisitionFrameRate=min(990/exposureId(channelInd(kk)),150);
        for zStepVal=0
%             stageMove(handles,hObject,eventdata,3,zPosTemp+zStepVal*0.5,0);
            stageMove(handles,hObject,eventdata,3,zPosTemp,0);
            pause(0.5);
            start(vid);
            pause(exposureId(channelInd(kk))/1000*2);
            fluoImg(:,:,kk,zStepVal+1)=getdata(vid,1);
            flushdata(vid);
            stop(vid);
        end
    end
    filterMove(4)
else
    fluoImg=zeros(sizeVal(1),sizeVal(2),sum(channelId),3);
    zPosTemp=str2double(get(handles.zPos,'string'));
    for kk=1:sum(channelId)
        filterMove(channelInd(kk));
        src.AcquisitionFrameRate=min(990/exposureId(channelInd(kk)),150);
        src.ExposureTime = 1000 * exposureId(channelInd(kk));
        for zStepVal=0
            stageMove(handles,hObject,eventdata,3,zPosTemp+zStepVal*0.5,0);
            pause(0.5);
            start(vid);
            pause(exposureId(channelInd(kk))/1000*2);
            fluoImg(:,:,kk,zStepVal+1)=getdata(vid,1);
            flushdata(vid);
            stop(vid);
        end
    end
    stageMove(handles,hObject,eventdata,3,zPosTemp,0);
    filterMove(4)
end

dateTimeVal=datetime;
positionVal=get(handles.xPos,'userdata');
folderPath=get(handles.folderPath,'UserData');
if timeLapseNum==0
    set(handles.scanBtn,'userdata',fluoImg);
    cd(folderPath);
    sampleList=dir('sample*_Fluo.mat');
    if isempty(sampleList)==1
        save('sample001_Fluo.mat','fluoImg','channelId','exposureId','dateTimeVal','positionVal');
    else
        save(strcat('sample',sprintf('%03d',length(sampleList)+1),'_Fluo.mat'),'fluoImg','channelId','exposureId','dateTimeVal','positionVal');
    end
else
    set(handles.scanBtn,'userdata',fluoImg(:,:,:,2));
    cd(folderPath);
    save(strcat('sample',sprintf('%03d',timeLapseBatchNum),'_TimeLapse_',sprintf('%03d',(timeLapseNum)),'_Fluo.mat'),'fluoImg','channelId','exposureId','dateTimeVal','positionVal');
end

axes(handles.previewPanel)
imagesc(max(fluoImg,[],3)),axis image,colormap('gray')
set(handles.acquisitionMsg,'string',strcat('Fluorescence Completed '));


function closeShutter_Callback(hObject, eventdata, handles)
filterMove(4)

function laserFlag_Callback(hObject, eventdata, handles)
global session;
outputSingleScan(session,[0 0 0 1])
set(handles.laserFlag,'userdata',1);

function fluoGain_Callback(hObject, eventdata, handles)
set(handles.fluoGain,'userdata',str2double(get(handles.fluoGain,'string')));

function fluoGain_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function fluoTimeLapse_Callback(hObject, eventdata, handles)
global session;
vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
src.Gain=get(handles.fluoGain,'userdata');
stoppreview(vid);
set(handles.previewMsg,'string','Stop Preview');
drawnow

set(vid, 'FramesPerTrigger', get(handles.fluoNumFrame,'userdata'));
triggerconfig(vid, 'immediate');
exposureId=[str2double(get(handles.bExposure,'string')) str2double(get(handles.gExposure,'string')) str2double(get(handles.rExposure,'string'))];
channelId=[get(handles.bCheck,'value') get(handles.gCheck,'value') get(handles.rCheck,'value')];

src.Shutter=exposureId(channelInd(kk));
start(vid);
pause(0.5);

fluoImg=getdata(vid, get(handles.fluoNumFrame,'userdata'));
fluoImg=squeeze(fluoImg(:,:,1,:));
flushdata(vid)

filterMove(4)
folderPath=get(handles.folderPath,'UserData');
set(handles.scanBtn,'userdata',fluoImg);

cd(folderPath);
sampleList=dir('sample*_Fluo.mat');
if isempty(sampleList)==1
    save('sample001_Fluo.mat','fluoImg','channelId','exposureId');
else
    save(strcat('sample',sprintf('%03d',length(sampleList)+1),'_Fluo.mat'),'fluoImg','channelId','exposureId');
end
set(handles.acquisitionMsg,'string',strcat('Fluorescence Completed '));
stop(vid);
triggerconfig(vid, 'immediate');
set(vid, 'FramesPerTrigger', 1);
src.FrameRate= get(handles.frameNumControl,'userdata');
vid.TriggerRepeat = 0;

function fluoNumFrame_Callback(hObject, eventdata, handles)
set(handles.fluoNumFrame,'userdata',str2double(get(handles.fluoNumFrame,'string')));

function fluoNumFrame_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function xPos_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,2,str2double(get(handles.xPos,'string')),0);

function yPos_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,1,str2double(get(handles.yPos,'string')),0);

function zPos_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,3,str2double(get(handles.zPos,'string')),0);

function xPosUp_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,2,1);

function xPosDown_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,2,-1);

function yPosUp_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,1,1);

function yPosDown_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,1,-1);

function zPosUp_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,3,1);

function zPosDown_Callback(hObject, eventdata, handles)
stageMove(handles,hObject,eventdata,3,-1);

function stageMove(handles,hObject,eventdata,varargin)
global rootZeiss; global defaultPosition;
readJoystick_Callback(hObject, eventdata, handles)
StageAxis=rootZeiss.GetComponent(char("MTBStage"));
FocusAxis=rootZeiss.GetComponent(char("MTBFocus"));
positionVal=get(handles.xPos,'userdata');
if length(varargin)==2
    if varargin{1}~=3
        positionVal(varargin{1})=positionVal(varargin{1})+(varargin{2})*str2double(get(handles.lateralDist,'string'));
    else
        positionVal(3)=positionVal(3)+(varargin{2})*str2double(get(handles.AxialDist,'string'));
    end
elseif length(varargin)==3
    positionVal(varargin{1})=varargin{2};
else
    positionValtemp=varargin{1};
    positionVal=[positionValtemp(2),positionValtemp(1),positionValtemp(3)];
end

set(handles.xPos,'userdata',positionVal);
set(handles.xPos,'string',num2str(positionVal(2)))
set(handles.yPos,'string',num2str(positionVal(1)))
set(handles.zPos,'string',num2str(positionVal(3)))

StageAxis.SetPosition(defaultPosition(1)+positionVal(1),defaultPosition(2)+positionVal(2),"µm",ZEISS.MTB.Api.MTBCmdSetModes.Default);
FocusAxis.SetPosition(defaultPosition(3)+positionVal(3),"µm",ZEISS.MTB.Api.MTBCmdSetModes.Default);

function lateralDist_Callback(hObject, eventdata, handles)

function AxialDist_Callback(hObject, eventdata, handles)

function AxialDist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function xPos_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function yPos_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function zPos_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function lateralDist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function resetPos_Callback(hObject, eventdata, handles)
global defaultPosition;
positionVal=get(handles.xPos,'userdata');
positionArray=get(handles.posList,'data');
if isempty(positionArray)==0
    defaultPosition=[defaultPosition(1)+positionVal(1),defaultPosition(2)+positionVal(2),defaultPosition(3)+positionVal(3)];
    positionArray(:,1:3)=positionArray(:,1:3)+repmat([positionVal(2) positionVal(1) positionVal(3)],[size(positionArray,1) 1]);
    set(handles.posList,'data',positionArray);
    axes(handles.posPlot);
    plot(positionArray(:,1),positionArray(:,2),'ok')
    xlim([min(min(positionArray(:,1)*0.7,positionArray(:,1)*1.3)) max(max(positionArray(:,1)*0.7,positionArray(:,1)*1.3))])
    ylim([min(min(positionArray(:,2)*0.7,positionArray(:,2)*1.3)) max(max(positionArray(:,2)*0.7,positionArray(:,2)*1.3))])
else
    defaultPosition=[defaultPosition(1)+positionVal(1),defaultPosition(2)+positionVal(2),defaultPosition(3)+positionVal(3)];
end
set(handles.xPos,'userdata',[0 0 0]);
set(handles.xPos,'string',num2str(0))
set(handles.yPos,'string',num2str(0))
set(handles.zPos,'string',num2str(0))

function goBtn_Callback(hObject, eventdata, handles)
positionArray=get(handles.posList,'data');
posInterest=get(handles.deletePos,'userdata');
positionVal=positionArray(posInterest,1:3);
set(handles.bExposure,'string',num2str(positionArray(posInterest,4)));
set(handles.gExposure,'string',num2str(positionArray(posInterest,5)));
set(handles.rExposure,'string',num2str(positionArray(posInterest,6)));
stageMove(handles,hObject,eventdata,positionVal);

function deletePos_Callback(hObject, eventdata, handles)
positionArray=get(handles.posList,'data');
[posNum, dummy]=size(positionArray);
posInterest=get(handles.deletePos,'userdata');

if posNum==1
    positionArray=[];
    set(handles.posList,'data',positionArray);
    axes(handles.posPlot);
    plot(0,0,'ow');
else
    positionArray(posInterest,:)=[];
    set(handles.posList,'data',positionArray);
    axes(handles.posPlot);
    plot(positionArray(:,1),positionArray(:,2),'ok')
    xlim([min(min(positionArray(:,1)*0.7,positionArray(:,1)*1.3))-10 max(max(positionArray(:,1)*0.7,positionArray(:,1)*1.3))+10])
    ylim([min(min(positionArray(:,2)*0.7,positionArray(:,2)*1.3))-10 max(max(positionArray(:,2)*0.7,positionArray(:,2)*1.3))+10])
    
end

function savePosBtn_Callback(hObject, eventdata, handles)
positionVal=get(handles.xPos,'userdata');
positionVal=[positionVal(2), positionVal(1), positionVal(3)];
exposureVal=[str2double(get(handles.bExposure,'string')) str2double(get(handles.gExposure,'string')) str2double(get(handles.rExposure,'string'))];

positionArray=get(handles.posList,'data');
positionArray(end+1,:)=[positionVal, exposureVal];
set(handles.posList,'data',positionArray);
axes(handles.posPlot);
plot(positionArray(:,1),positionArray(:,2),'ok')
xlim([min(min(positionArray(:,1)*0.7,positionArray(:,1)*1.3))-10 max(max(positionArray(:,1)*0.7,positionArray(:,1)*1.3))+10])
ylim([min(min(positionArray(:,2)*0.7,positionArray(:,2)*1.3))-10 max(max(positionArray(:,2)*0.7,positionArray(:,2)*1.3))+10])

function posList_CellSelectionCallback(hObject, eventdata, handles)
coord=eventdata.Indices;
if numel(coord)~=0
    trapVal=coord(1);
    set(handles.deletePos,'userdata',trapVal);
end

function multiSampleScanBtn_Callback(hObject, eventdata, handles)
global res; global NA;global lambda;
global scanVolt; global session;
session.Rate=1000;
set(handles.currentNum,'string','0');
set(handles.acquisitionMsg,'string','Tomography Acquisition Processing');
vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);
src.Gain = get(handles.gainControl,'userdata');
stoppreview(vid);
set(handles.previewMsg,'string','Stop Preview');
drawnow
set(vid, 'FramesPerTrigger', 1);
vid.TriggerRepeat = Inf;
triggerconfig(vid, 'hardware', 'DeviceSpecific', 'DeviceSpecific');
src.TriggerMode = 'On';
src.TriggerActivation = 'RisingEdge';
src.TriggerSelector = 'FrameStart';
src.TriggerSource = 'Line1';

start(vid);
pause(0.5);

stepNum=get(handles.stepNum,'userdata');
positionArray=get(handles.posList,'data');
set(handles.totalNumTimeLapse,'string',num2str(size(positionArray,1)));

folderPath=get(handles.folderPath,'UserData');
cd(folderPath);
sampleList=dir('sample001_multiScan_*_Tomog.mat');

for timeLapseNum=1:size(positionArray,1)
    if get(handles.laserFlag,'userdata')==0
        outputSingleScan(session,[scanVolt(1,1) scanVolt(1,2) 0 1])
        set(handles.laserFlag,'userdata',1);
    end
    positionVal=positionArray(timeLapseNum,1:3);
    stageMove(handles,hObject,eventdata,positionVal);
    
    pause(0.5)
    pause(get(handles.durationTime,'userdata')*0.001);
    queueOutputData(session,scanVolt)
    startForeground(session);
    %         pause(0.1);
    set(handles.currentNum,'string',num2str(timeLapseNum));
    drawnow
    outputSingleScan(session,[scanVolt(1,1) scanVolt(1,2) 0 1])
    set(handles.laserFlag,'userdata',0);
    
    tomogMap=getdata(vid,stepNum);
    tomogMap=squeeze(tomogMap(:,:,1,:));
    flushdata(vid)
    if isempty(sampleList)==1
        save(strcat('sample',sprintf('%03d',timeLapseNum),'_multiScan_001_Tomog.mat'),'tomogMap','positionVal','res','lambda','NA');
    else
        save(strcat('sample',sprintf('%03d',timeLapseNum),'_multiScan_',sprintf('%03d',length(sampleList)+1),'_Tomog.mat'),'tomogMap','positionVal','res','lambda','NA');
    end
end
outputSingleScan(session,[0 0 0 1])

axes(handles.tomogCheck);
imagesc(squeeze(tomogMap(:,end/2,:))')

axes(handles.tomogCheckProf);
temp=mean(squeeze(mean(abs(tomogMap),1)));
plot(temp,'or')
ylim([0 255])
camroll(-90)
set(handles.acquisitionMsg,'string',strcat('Tomography Completed '));
stop(vid);
triggerconfig(vid, 'immediate');
src.TriggerMode = 'Off';
vid.TriggerRepeat = 0;

function meshScanBtn_Callback(hObject, eventdata, handles)
global res;global scanVolt; global session;global rootZeiss; global defaultPosition;
global NA;global lambda;
session.Rate=1000;
stepNum=get(handles.stepNum,'userdata');
positionArray=get(handles.posList,'data');

xVal=[min(positionArray(:,1)) max(positionArray(:,1))];
yVal=[min(positionArray(:,2)) max(positionArray(:,2))];

xList=xVal(1):(res*1024*0.8):xVal(2);
yList=yVal(1):(res*1024*0.8):yVal(2);

if (xVal(end)-xList(end))>(res*1024*0.45)
    xList(end+1)=xVal(end);
end
if (yVal(end)-yList(end))>(res*1024*0.45)
    yList(end+1)=yVal(end);
end
scanPosList=zeros(length(xList)*length(yList),2);
scanPosList(1:length(xList),1)=xList;
temp=padarray(scanPosList(1:length(xList),1),size(scanPosList,1),'symmetric','post');
scanPosList(:,1)=temp(1:size(scanPosList,1));
scanPosList(:,2)=repelem(yList,length(xList));

axes(handles.posPlot);
plot(scanPosList(:,1),scanPosList(:,2),'o-k')
xlim([min(min(scanPosList(:,1)*0.7,scanPosList(:,1)*1.3)) max(max(scanPosList(:,1)*0.7,scanPosList(:,1)*1.3))])
ylim([min(min(scanPosList(:,2)*0.7,scanPosList(:,2)*1.3)) max(max(scanPosList(:,2)*0.7,scanPosList(:,2)*1.3))])


set(handles.totalNumTimeLapse,'string',num2str(size(scanPosList,1)));
folderPath=get(handles.folderPath,'UserData');
cd(folderPath);
sampleList=dir('sample*_meshScan_001_Tomog.mat');

vid=get(handles.startPreview,'userdata');
src=getselectedsource(vid);

for timeLapseNum=1:size(scanPosList,1)
    positionVal=scanPosList(timeLapseNum,:);
    positionVal(3)=positionArray(1,3);
    stageMove(handles,hObject,eventdata,eventdata,positionVal);
    
    channelId=[get(handles.bCheck,'value') get(handles.gCheck,'value') get(handles.rCheck,'value')];
    if sum(channelId)>0
        outputSingleScan(session,[0 0 0 0])
        pause(0.1)
        
        src.Gain=get(handles.fluoGain,'userdata');
        stoppreview(vid);
        set(handles.previewMsg,'string','Stop Preview');
        drawnow
        
        set(vid, 'FramesPerTrigger', 1);
        triggerconfig(vid, 'immediate');
        
        channelInd=find(channelId==1);
        exposureId=[str2double(get(handles.bExposure,'string')) str2double(get(handles.gExposure,'string')) str2double(get(handles.rExposure,'string'))];
        
        sizeVal=vid.ROIPosition;
        sizeVal=sizeVal(3:4);
        fluoImg=zeros(sizeVal(1),sizeVal(2),sum(channelId));
        
        for kk=1:sum(channelId)
            filterMove(channelInd(kk));
            
            src.AcquisitionFrameRate = min(990/exposureId(channelInd(kk)),get(handles.frameNumControl,'userdata'));
            src.ExposureTime = 1000* exposureId(channelInd(kk));
            pause(0.5);
            start(vid);
            pause(exposureId(channelInd(kk))/1000*2);
            fluoImg(:,:,kk)=getdata(vid,1);
            flushdata(vid);
            stop(vid);
        end
        filterMove(4)
        folderPath=get(handles.folderPath,'UserData');
        set(handles.scanBtn,'userdata',fluoImg);
        
        cd(folderPath);
        if isempty(sampleList)==1
            save(strcat('sample001','_meshScan_',sprintf('%03d',(timeLapseNum)),'_Fluo.mat'),'fluoImg','channelId','exposureId','positionVal');
        else
            save(strcat('sample',sprintf('%03d',length(sampleList)+1),'_meshScan_',sprintf('%03d',(timeLapseNum)),'_Fluo.mat'),'fluoImg','channelId','exposureId','positionVal');
        end
    end
    
    StageAxis=rootZeiss.GetComponent(char("MTBStage"));
    FocusAxis=rootZeiss.GetComponent(char("MTBFocus"));
    
    [posX,posY]=StageAxis.GetPosition("µm");
    posZ=FocusAxis.GetPosition("µm");
    posX=posX-defaultPosition(1);
    posY=posY-defaultPosition(2);
    posZ=posZ-defaultPosition(3);
    positionReal=[posX,posY,posZ];
    
    if get(handles.laserFlag,'userdata')==0
        outputSingleScan(session,[scanVolt(1,1) scanVolt(1,2) 0 1])
        set(handles.laserFlag,'userdata',1);
    end
    
    outputSingleScan(session,[scanVolt(1,1) scanVolt(1,2) 0 1])
  
    set(handles.stopPreview,'userdata',0);
    src.AcquisitionFrameRate = get(handles.frameNumControl,'userdata');
    src.AcquisitionTimingMode = 'FrameRate';
    src.ExposureTime = 1000* get(handles.exposureControl,'userdata');
    src.Gain = get(handles.gainControl,'userdata');
    
    set(handles.currentNum,'string','0');
    set(handles.acquisitionMsg,'string','Tomography Acquisition Processing');
    
    stoppreview(vid);
    set(handles.previewMsg,'string','Stop Preview');
    drawnow
    set(vid, 'FramesPerTrigger', 1);
    vid.TriggerRepeat = Inf;
    triggerconfig(vid, 'hardware', 'DeviceSpecific', 'DeviceSpecific');
    src.TriggerMode = 'On';
    src.TriggerActivation = 'RisingEdge';
    src.TriggerSelector = 'FrameStart';
    src.TriggerSource = 'Line1';
    
    start(vid);
    pause(0.5);
    pause(get(handles.durationTime,'userdata')*0.001);
    queueOutputData(session,scanVolt)
    startForeground(session);
    set(handles.currentNum,'string',num2str(timeLapseNum));
    drawnow
    outputSingleScan(session,[scanVolt(1,1) scanVolt(1,2) 0 1])
    set(handles.laserFlag,'userdata',0);
    pause(0.5);
    tomogMap=getdata(vid,stepNum);
    tomogMap=squeeze(tomogMap(:,:,1,:));
    flushdata(vid)
    stop(vid);
    if isempty(sampleList)==1
        save(strcat('sample001','_meshScan_',sprintf('%03d',(timeLapseNum)),'_Tomog.mat'),'tomogMap','positionVal','positionReal','res','lambda','NA');
    else
        save(strcat('sample',sprintf('%03d',length(sampleList)+1),'_meshScan_',sprintf('%03d',(timeLapseNum)),'_Tomog.mat'),'tomogMap','positionVal','positionReal','res','lambda','NA');
    end
end
outputSingleScan(session,[0 0 0 1])

axes(handles.tomogCheck);
imagesc(squeeze(tomogMap(:,end/2,:))')

axes(handles.tomogCheckProf);
temp=mean(squeeze(mean(abs(tomogMap),1)));
plot(temp,'or')
ylim([0 255])
camroll(-90)
set(handles.acquisitionMsg,'string',strcat('Tomography Completed '));
triggerconfig(vid, 'immediate');
vid.TriggerRepeat = 0;
src.TriggerMode = 'Off';

function resetPosList_Callback(hObject, eventdata, handles)
set(handles.posList,'data',[]);
axes(handles.posPlot);
plot([0 0],'ow')

function readJoystick_Callback(hObject, eventdata, handles)
global rootZeiss; global defaultPosition;
StageAxis=rootZeiss.GetComponent(char("MTBStage"));
FocusAxis=rootZeiss.GetComponent(char("MTBFocus"));

[posX,posY]=StageAxis.GetPosition("µm");
posZ=FocusAxis.GetPosition("µm");
posX=posX-defaultPosition(1);
posY=posY-defaultPosition(2);
posZ=posZ-defaultPosition(3);
set(handles.xPos,'string',num2str(posY))
set(handles.yPos,'string',num2str(posX))
set(handles.zPos,'string',num2str(posZ))
positionVal=[posX posY posZ];
set(handles.xPos,'userdata',positionVal);

function mag_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function mag_Callback(hObject, eventdata, handles)
global res;
res=5.5/str2double(get(handles.mag,'string'));

function NAscan_Callback(hObject, eventdata, handles)
global NA;
NA=5.5/str2double(get(handles.NAscan,'string'));