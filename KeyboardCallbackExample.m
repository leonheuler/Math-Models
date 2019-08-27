% Creating the intial figure data
offset = 0;
x=linspace(0,10,1000); 
y=(x-offset).^2;
f = figure;
% Using guihandles to create the structure in which data can be saved
myhandles = guihandles(f); 
% Adding an offset field to the structure
myhandles.offset = 0;
% Save the structure
guidata(f,myhandles) 
plot(x,y);
% Set the KeyPressFcn Callback
set(f,'KeyPressFcn',{@myfun,1});
function myfun(src,event,rate)
    % Get the structure using guidata in the callback
    myhandles = guidata(src);
    % If right arrow is pressed
    if strcmp(event.Key,'rightarrow')
        %Modify offset in the strucuture with the rate you want
        myhandles.offset = myhandles.offset + rate;
        % Save the change to the structure
        guidata(src,myhandles) 
        % Get the line object which is the grand-child of the figure
        % window
        lineObject = src.Children.Children;
        % Modify the YData of the line object.
        x = lineObject.XData;
        lineObject.YData = (x-myhandles.offset).^2;
    end
        % If left arrow is pressed
    if strcmp(event.Key,'leftarrow')
        %Modify offset in the strucuture with the rate you want
        myhandles.offset = myhandles.offset - rate;
        % Save the change to the structure
        guidata(src,myhandles) 
        % Get the line object which is the grand-child of the figure
        % window
        lineObject = src.Children.Children;
        % Modify the YData of the line object.
        x = lineObject.XData;
        lineObject.YData = (x-myhandles.offset).^2;
    end
end