function build_mex()
% BUILD_MEX Compiles the C-MEX S-Function for Damping CBF
% Automatically detects shared libraries from either pure CMake or colcon builds.

current_dir = fileparts(mfilename('fullpath'));
ws_dir = fileparts(current_dir);

inc_cbf_common = fullfile(ws_dir, 'cbf_common', 'include');
inc_damping_cbf = fullfile(ws_dir, 'damping_cbf_controller', 'include');
inc_eigen = '/usr/include/eigen3';

% Search for libdamping_cbf_controller.so
cand_damping = {
    fullfile(ws_dir, 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'build', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'damping_cbf', 'lib'), ...
    fullfile(fileparts(ws_dir), 'install', 'lib'), ...
    '/usr/local/lib', '/usr/lib'
};
lib_damping_cbf = find_lib_dir(cand_damping, 'libdamping_cbf_controller.so');
if isempty(lib_damping_cbf)
    error(['libdamping_cbf_controller.so not found! ' ...
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

src_file = fullfile(current_dir, 'src', 'sfun_damping_cbf.cpp');

rpath_flags = sprintf('-Wl,-rpath,%s -Wl,-rpath,%s', lib_damping_cbf, lib_cbf_common);

mex_cmd = sprintf(['mex "%s" -I"%s" -I"%s" -I"%s" ' ...
                   '-L"%s" -L"%s" -ldamping_cbf_controller -lcbf_common ' ...
                   'LDFLAGS=''$LDFLAGS %s'' ' ...
                   '-R2018a -output sfun_damping_cbf'], ...
                   src_file, inc_cbf_common, inc_damping_cbf, inc_eigen, ...
                   lib_damping_cbf, lib_cbf_common, rpath_flags);

disp(['Executing: ' mex_cmd]);
eval(mex_cmd);
disp('Successfully compiled sfun_damping_cbf.mexa64 with embedded RPATH!');
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
