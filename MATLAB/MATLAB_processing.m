% NEW
clc
serialportlist("available") % find which ports are being open
% s = serialport("COM3", 115200, "StopBits", 1);
configureCallback(s,"byte",912,@readSerialData) % callback function setup
%


x = input("press 1 to start: ")

while x ~= 2    
A = [1 1 0 220 1 100 100 501 1 100 2 220 1 1 1 1];

array = [A(1,1:4); A(1,5:8); A(1,9:12); A(1,13:16)]

% colormap(jet(12))

b = bar3(array)
colorbar

for k = 1:length(b)
    zdata = b(k).ZData;
    b(k).CData = zdata;
    
    b(k).FaceColor = 'interp';
   
end
% if mod(x,10) == 0
%     drawnow
% end

x = x+1

end


function readSerialData(src,~)
    bar_array = [];
    for i = 1:16
        data = readline(src);
        if (strlength(data) == 1)
            data = readline(src);
        end
        disp(data);
        k = 0;
        k = strfind(data, "Zone");

        if(k == 13) % Zone location for current ranging loop
            b = data{1};
            zone = str2num(b(20:22));
            row = floor(zone/4) + 1;
            col = mod(zone, 4) + 1;
    %         disp(b(20:22)); % Zone
            bar_array(row, col) = str2num(b(49:53));
    %         disp(b(34:36)); % Status
    %         disp(b(49:53)); % Distance
        end
    end
    
    figure(1);
    graph = bar3(bar_array)
    colorbar
    caxis([0 4000]);
    axis([0 5 0 5 0 4000]);

    for i = 1:length(graph)
        zdata = graph(i).ZData;
        graph(i).CData = zdata;
        graph(i).FaceColor = 'interp';
    end
    drawnow
end