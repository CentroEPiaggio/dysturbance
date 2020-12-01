function results = ReadYaml(filePath)
%--------------------------------------------------------------------------
% This is a modified version of the function from Lloyd Russell 2017
% Modifications are made to convert the yaml structure to a matlab
% structure. Earlier version does not consider the original yaml structure
% and cancel all the variable with the same name.
% 
% Modifications are made by Simone Monteleone (2020) 
%--------------------------------------------------------------------------

% read file line by line
fid = fopen(filePath, 'r');
data = textscan(fid, '%s', 'delimiter', '\n', 'whitespace', '');
fclose(fid);

% remove empty lines
data = deblank(data{1});
data(cellfun('isempty', data)) = [];

% prepare final results structure
results = [];
inside = 0;
% parse the contents (line by line)
for i = 1:numel(data)
    
    % extract this line
    thisLine = data{i};
    
    % ignore if this line is a comment
    if strcmpi(thisLine(1), '#')
        continue
    end
    
    % find the seperator between key and value
    sepIndex = find(thisLine==':', 1, 'first');
    
    % get the key name (remove whitespace)
    if isspace(thisLine(1:2))
        inside = 1;
        if isspace(thisLine(2:4))
            inside = 2;
        end
    end
    
    key = strtrim(thisLine(1:sepIndex-1));
    
    % get the value, ignoring any comments (remove whitespace)
    value = strsplit(thisLine(sepIndex+1:end), '#');
    value = strtrim(value{1});
    
    % attempt to convert value to numeric type
    [convertedValue, success] = str2num(value);
    if success
        value = convertedValue;
    end
    
    % store the key and value in the results
    if inside == 0
        results.(key) = value;
        key_prec = key;
    elseif inside == 1
        results.(key_prec).(key) = value;
        key_prec_2 = key_prec;
        key_prec_1 = key;
    elseif inside == 2
        results.(key_prec_2).(key_prec_1).(key) = value;
    else 
        error("Error: yaml file structure is miss readed");
    end
    inside = 0;
end
