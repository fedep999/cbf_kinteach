function build_mex()
% BUILD_MEX Compiles the C-MEX S-Function for the joint-space torque CBF (sfun_joint_torque_cbf).
% Alternative to the CMake build (which also produces the MEX); detects the shared libraries from a
% pure CMake or a colcon build.

current_dir = fileparts(mfilename('fullpath'));
ws_dir = fileparts(current_dir);

inc_cbf_common = fullfile(ws_dir, 'cbf_common', 'include');
inc_jtorque_cbf = fullfile(ws_dir, 'joint_torque_cbf_controller', 'include');
inc_eigen = '/usr/include/eigen3';

% Pinocchio include directory (needed by the controller header)
pinocchio_inc_cands = {
    fullfile(fileparts(ws_dir), 'pinocchio_install', 'include'), ...
    '/opt/ros/jazzy/include', '/usr/local/include', '/usr/include', '/opt/openrobots/include'
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

lib_cands = {
    fullfile(ws_dir, 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'jtorque_cbf', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'cbf_common', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'lib'), ...
    '/usr/local/lib', '/usr/lib'
};
lib_jtorque = find_lib_dir(lib_cands, 'libjoint_torque_cbf_controller.so');
if isempty(lib_jtorque)
    error(['libjoint_torque_cbf_controller.so not found! ' ...
           'Build the C++ libraries first with "cmake --build <build_dir>" or "colcon build".']);
end
lib_cbf_common = find_lib_dir(lib_cands, 'libcbf_common.so');
if isempty(lib_cbf_common)
    error(['libcbf_common.so not found! ' ...
           'Build the C++ libraries first with "cmake --build <build_dir>" or "colcon build".']);
end

src_file = fullfile(current_dir, 'src', 'sfun_joint_torque_cbf.cpp');
rpath_flags = sprintf('-Wl,-rpath,%s -Wl,-rpath,%s', lib_jtorque, lib_cbf_common);

mex_cmd = sprintf(['mex "%s" -I"%s" -I"%s" -I"%s" -I"%s" %s ' ...
                   '-L"%s" -L"%s" -ljoint_torque_cbf_controller -lcbf_common ' ...
                   'LDFLAGS=''$LDFLAGS %s'' ' ...
                   '-R2018a -outdir "%s" -output sfun_joint_torque_cbf'], ...
                   src_file, inc_cbf_common, inc_jtorque_cbf, inc_eigen, inc_pinocchio, extra_inc, ...
                   lib_jtorque, lib_cbf_common, rpath_flags, current_dir);

disp(['Executing: ' mex_cmd]);
eval(mex_cmd);
disp('Successfully compiled sfun_joint_torque_cbf.mexa64 with embedded RPATH!');
end

function p = find_lib_dir(candidates, filename)
p = '';
for i = 1:length(candidates)
    if exist(fullfile(candidates{i}, filename), 'file') == 2
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
