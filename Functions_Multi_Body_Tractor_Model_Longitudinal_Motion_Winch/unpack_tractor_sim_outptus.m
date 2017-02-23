function [tractor, controlArchitecture, inputMat, nTimeParam] = unpack_tractor_sim_outptus(j)

r = fetchOutputs(j);

tractor = r{1};
controlArchitecture = r{2};
inputMat = r{3};
nTimeParam = r{4};

end