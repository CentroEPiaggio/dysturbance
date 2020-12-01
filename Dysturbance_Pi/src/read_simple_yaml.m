%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% read_simple_yaml.m
%
% Extract from a simple and 1 level yaml the data in it
%
% Anthony Remazeilles
% Copyright Tecnalia 2019
% Beerware license.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function data = read_simple_yaml(filename)

    fid = fopen(filename);

    spec = "%s %f";
    infile = textscan(fid, spec, 'Delimiter', ':');

    labels = infile{1};
    values = infile{2};
    [nitem, misc] = size(labels);

    for i=1:nitem
        label = labels{i};
        data.(label) = values(i);
    end
    fclose(fid);
end