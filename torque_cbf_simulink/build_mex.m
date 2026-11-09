function build_mex()
% BUILD_MEX Compiles the C-MEX S-Function for Torque CBF
% Automatically detects shared libraries from either pure CMake or colcon builds.

current_dir = fileparts(mfilename('fullpath'));
ws_dir = fileparts(current_dir);

inc_cbf_common = fullfile(ws_dir, 'cbf_common', 'include');
inc_torque_cbf = fullfile(ws_dir, 'torque_cbf_controller', 'include');
inc_eigen = '/usr/include/eigen3';

% Search for Pinocchio include directory (needed for torque_cbf headers)
pinocchio_inc_cands = {
    fullfile(fileparts(ws_dir), 'pinocchio_install', 'include'), ...
    '/opt/ros/jazzy/include', ...
    '/usr/local/include', ...
    '/usr/include', ...
    '/opt/openrobots/include'
};
inc_pinocchio = find_existing_dir(pinocchio_inc_cands, 'pinocchio');
if isempty(inc_pinocchio)
    inc_pinocchio = '/usr/include';
end

inc_pinocchio_dep = fullfile(inc_pinocchio, 'pinocchio', 'deprecated');
extra_inc = '';
if exist(inc_pinocchio_dep, 'dir') == 7
    extra_inc = sprintf('-I"%s"', inc_pinocchio_dep);
end

% Search for libtorque_cbf_controller.so
cand_torque = {
    fullfile(ws_dir, 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'torque_cbf', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'lib'), ...
    '/usr/local/lib', '/usr/lib'
};
lib_torque_cbf = find_lib_dir(cand_torque, 'libtorque_cbf_controller.so');
if isempty(lib_torque_cbf)
    error(['libtorque_cbf_controller.so not found! ' ...
           'Please build the C++ libraries first with "cmake --build <build_dir>" or "colcon build".']);
end

% Search for libcbf_common.so
cand_common = {
    fullfile(ws_dir, 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'cbf_common', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'lib'), ...
    '/usr/local/lib', '/usr/lib'
};
lib_cbf_common = find_lib_dir(cand_common, 'libcbf_common.so');
if isempty(lib_cbf_common)
    error(['libcbf_common.so not found! ' ...
           'Please build the C++ libraries first with "cmake --build <build_dir>" or "colcon build".']);
end

src_file = fullfile(current_dir, 'src', 'sfun_torque_cbf.cpp');

rpath_flags = sprintf('-Wl,-rpath,%s -Wl,-rpath,%s', lib_torque_cbf, lib_cbf_common);

mex_cmd = sprintf(['mex "%s" -I"%s" -I"%s" -I"%s" -I"%s" %s ' ...
                   '-L"%s" -L"%s" ' ...
                   '-ltorque_cbf_controller -lcbf_common ' ...
                   'LDFLAGS=''$LDFLAGS %s'' ' ...
                   '-R2018a -output sfun_torque_cbf'], ...
                   src_file, inc_cbf_common, inc_torque_cbf, inc_eigen, ...
                   inc_pinocchio, extra_inc, ...
                   lib_torque_cbf, lib_cbf_common, rpath_flags);

disp(['Executing: ' mex_cmd]);
eval(mex_cmd);
disp('Successfully compiled sfun_torque_cbf.mexa64 with embedded RPATH!');
end

function p = find_lib_dir(candidates, filename)
p = '';
for i = 1:length(candidates)
    full_path = fullfile(candidates{i}, filename);
    if exist(full_path, 'file') == 2
        p = candidates{i};
        return;
    end
end
end

function p = find_existing_dir(candidates, subfolder)
p = '';
for i = 1:length(candidates)
    if exist(fullfile(candidates{i}, subfolder), 'dir') == 7
        p = candidates{i};
        return;
    end
end
end
