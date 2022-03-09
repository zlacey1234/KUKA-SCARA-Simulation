function path_to_workspace = updatePaths()
path_to_workspace = cd;
cd ..
addpath rtb
addpath common
addpath smtb

updatePathsUtils();
cd(path_to_workspace)

end
