function [Day,Month,Year,Hour,Min,Sec,Latdeg,Londeg,Altkm,Sun,M_nT,W,q] = import_STK(filename, startRow, endRow)
%IMPORTFILE Import numeric data from a text file as column vectors.
%   [DAY,MONTH,YEAR,TIME,LATDEG,LONDEG,ALTKM,SUNX,SUNY,SUNZ,MX,MY,MZ,WX,WY,WZ,Q1,Q2,Q3,Q4]
%   = IMPORTFILE(FILENAME) Reads data from text file FILENAME for the
%   default selection.
%
%   [DAY,MONTH,YEAR,TIME,LATDEG,LONDEG,ALTKM,SUNX,SUNY,SUNZ,MX,MY,MZ,WX,WY,WZ,Q1,Q2,Q3,Q4]
%   = IMPORTFILE(FILENAME, STARTROW, ENDROW) Reads data from rows STARTROW
%   through ENDROW of text file FILENAME.
%
% Example:
%   [Day,Month,Year,Time,Latdeg,Londeg,Altkm,Sunx,Suny,Sunz,Mx,My,Mz,Wx,Wy,Wz,q1,q2,q3,q4] = importfile('Cubesat AttitudeDetermination.txt',6, 54006);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2016/11/30 12:24:28

%% Initialize variables.
if nargin<=2
    startRow = 6;
    endRow = inf;
end

%% Format for each line of text:
%   column1: double (%f)
%	column2: text (%s)
%   column3: double (%f)
%	column4: text (%s)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
%   column13: double (%f)
%	column14: double (%f)
%   column15: double (%f)
%	column16: double (%f)
%   column17: double (%f)
%	column18: double (%f)
%   column19: double (%f)
%	column20: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%2f%4s%5f%13s%13f%13f%14f%15f%15f%15f%13f%13f%13f%15f%15f%15f%13f%13f%13f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
textscan(fileID, '%[^\n\r]', startRow(1)-1, 'WhiteSpace', '', 'ReturnOnError', false);
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', ',', 'WhiteSpace', '', 'EmptyValue' ,NaN,'ReturnOnError', false, 'EndOfLine', '\r\n');
for block=2:length(startRow)
    frewind(fileID);
    textscan(fileID, '%[^\n\r]', startRow(block)-1, 'WhiteSpace', '', 'ReturnOnError', false);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', '', 'WhiteSpace', '', 'EmptyValue' ,NaN,'ReturnOnError', false, 'EndOfLine', '\r\n');
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Remove white space around all cell columns.
dataArray{2} = strtrim(dataArray{2});
dataArray{4} = strtrim(dataArray{4});

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
% initialize data
N = length(dataArray{:, 1});
Month = zeros(N, 1);
Hour = zeros(N, 1);
Min = zeros(N, 1);
Sec = zeros(N, 1);

% Import array
Day = dataArray{:, 1};
month = dataArray{:, 2};
for i = 1:N % Loop to convert month data
    if strcmp(month{i, 1}, 'Jan')
        Month(i, 1) = 1;
    elseif strcmp(month{i, 1}, 'Feb')
        Month(i, 1) = 2;
    elseif strcmp(month{i, 1}, 'Mar')
        Month(i, 1) = 3;
    elseif strcmp(month{i, 1}, 'Apr')
        Month(i, 1) = 4;
    elseif strcmp(month{i, 1}, 'May')
        Month(i, 1) = 5;
    elseif strcmp(month{i, 1}, 'Jun')
        Month(i, 1) = 6;
    elseif strcmp(month{i, 1}, 'Jul')
        Month(i, 1) = 7;
    elseif strcmp(month{i, 1}, 'Aug')
        Month(i, 1) = 8;
    elseif strcmp(month{i, 1}, 'Sep')
        Month(i, 1) = 9;
    elseif strcmp(month{i, 1}, 'Oct')
        Month(i, 1) = 10;
    elseif strcmp(month{i, 1}, 'Nov')
        Month(i, 1) = 11;
    elseif strcmp(month{i, 1}, 'Dec')
        Month(i, 1) = 12;
    end
end
Year = dataArray{:, 3};
UTCG = dataArray{:, 4};
for i = 1:N % Loop to seperate time data
    Hour(i,1) = str2double(UTCG{i,1}(1:2));
    Min(i,1) = str2double(UTCG{i,1}(4:5));
    Sec(i,1) = str2double(UTCG{i,1}(7:end));
end

Latdeg = dataArray{:, 5};
Londeg = dataArray{:, 6};
Altkm = dataArray{:, 7};
% Create Sun vector
Sun = [dataArray{:, 8} dataArray{:, 9} dataArray{:, 10}];
% Create magnetic field vector
M_nT = [dataArray{:, 11} dataArray{:, 12} dataArray{:, 13}];
% Create angular velocity vector
W = [dataArray{:, 14} dataArray{:, 15} dataArray{:, 16}];
% Create quaternion vector
q = [dataArray{:, 20} dataArray{:, 17} dataArray{:, 18} dataArray{:, 19}];
