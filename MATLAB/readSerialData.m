% function readSerialData(src,~)
%     data = readline(src);
%     disp(data);
%     k = 0;
%     k = strfind(data, "Zone");
%     
%     if(k == 13) % Zone location for current ranging loop
%         b = data{1};
%         disp(b(20:22)); % Zone
%         disp(b(34:36)); % Status
%         disp(b(49:53)); % Distance
%     end
% 
% end