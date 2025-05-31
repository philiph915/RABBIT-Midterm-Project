function setpath()
    base_path=pwd;
%     
%     addpath(genpath_nosvn([base_path,'/ATRIAS-3D_SymbolicCode']));
%     addpath(genpath_nosvn([base_path,'/ImpactModel']));
%     addpath(genpath_nosvn([base_path,'/LagrangeModel']));
%     addpath(genpath_nosvn([base_path,'/Controller']));
    
    addpath(genpath_nosvn(cd));  % add all subdirectories to path
end

function pathstr = genpath_nosvn(startpath)
    % generates a path string, then removes paths which contain "\.svn" (on
    % Linux) or "/.svn" (on Windows)
    pathcell = regexp(genpath(startpath), pathsep, 'split');
    isnotsvn = cellfun(@(x)isempty(x), strfind(pathcell, [filesep '.svn']));
    isnotempty = ~cellfun(@(x)isempty(x), pathcell);
    pathstr = sprintf(['%s' pathsep],pathcell{isnotsvn & isnotempty});
end
