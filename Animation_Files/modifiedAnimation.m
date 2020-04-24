
function modifiedAnimation()
    Vicon=evalin('base', 'Vicon');
    vQ=evalin('base','vQ');

    world = vrworld('DroneLab.wrl'); 

    open(world);

    fig = view(world, '-internal');
    vrdrawnow;

    drone = vrnode(world, 'crazyflie');

    global counter;
    counter=1;
    
%% build GUI
    f = figure;

    timeSlider = uicontrol('Parent',f,'Style','slider','Position',[81,90,419,10],'value',counter, 'min',1, 'max',length(Vicon), 'Callback', @sliderCallback);
    playbackSpeedSlider = uicontrol('Parent',f,'Style','slider','Position',[81,40,419,10],'value',counter, 'min',-9, 'max',9, 'Callback', @playbackCallback);
    bgcolor = f.Color;
    playButton = uicontrol('Parent', f, 'Style', 'pushbutton', 'Position', [81, 150, 100, 50], 'String', 'Play/Pause', 'Callback', @playButtonCallback);
    endButton = uicontrol('Parent', f, 'Style', 'pushbutton', 'Position', [81, 220, 100, 50], 'String', 'Stop', 'Callback', @stopButtonCallback);
    bl1 = uicontrol('Parent',f,'Style','text','Position',[50,90,23,23],...
                    'String','0','BackgroundColor',bgcolor);
    bl2 = uicontrol('Parent',f,'Style','text','Position',[500,90,23,23],...
                    'String',max(Vicon(:,1)),'BackgroundColor',bgcolor);
    playbackSpeedSlider.Value=0;

%% initialize playback vars
    drone.translation=[0 0 0];
    drone.rotation=[0 0 0 0];
    global run
    run=true;
    global playbackSpeed
    playbackSpeed=1;
    dataRate=100;
    global newFrame
    global play
    newFrame=false;
    play=true;
%% control animation playback
    while(run==true)   
        if (play==true || newFrame==true) %wait until timer has finished
           tic
           newFrame=false;
           drone.translation = [Vicon(counter,2) Vicon(counter,3) Vicon(counter,4)];
           drone.rotation = [vQ(counter,2), vQ(counter,3), vQ(counter,4), 2*acos(vQ(counter,1))];
           vrdrawnow;
           timeSlider.Value=counter;
           timer=toc;
           frames=ceil(dataRate*timer*playbackSpeed);
           counter=counter+frames;
           pause(frames/(dataRate*playbackSpeed)-toc);
           if (counter>length(Vicon))
               play=false;
               counter=length(Vicon);
           end
        else
            pause(.05);
        end
    end
    close(f);
    close(world)
end

function sliderCallback(hObject,eventdata)
    global counter
    global newFrame
    counter=round(hObject.Value);
    newFrame=true;
end

function playButtonCallback(hObject, eventdata)
    global play
    play=~play;
end

function playbackCallback(hObject, eventdata)
    global playbackSpeed
    tempVal=hObject.Value;
    if (tempVal>=0)
        playbackSpeed=tempVal+1;
    else
        playbackSpeed=1/(abs(tempVal)+1);
    end
end

function stopButtonCallback(hObject, eventdata)
    global run
    run=false;
end