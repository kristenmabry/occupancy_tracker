% NEW
clc
serialportlist("available") % find which ports are being open
% s = serialport("COM5", 115200, "StopBits", 1);
configureCallback(s,"terminator",@readSerialData) % callback function setup
%

clc
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