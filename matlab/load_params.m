function P = load_params(jsonfile)
%LOAD_PARAMS  Read params/params.json into a struct and derive convenience fields.
%
%   P = load_params()            reads <repo>/params/params.json
%   P = load_params(file)        reads a specific JSON file
%
% Derived fields:
%   P.mass.loaded / P.mass.unloaded  : m, cg (FRD, from airframe CG), J (3x3), r_thrust
%   P.act.<name>.delta_max (rad), rate_max (rad/s)
%   P.aero.lon.<name>, P.aero.lat.<name>  : plain numbers (source tags in P.aero.src)
%   P.meta.sha256                    : hash of params.yaml

    here = fileparts(mfilename('fullpath'));
    if nargin < 1
        jsonfile = fullfile(here, '..', 'params', 'params.json');
    end
    if ~exist(jsonfile, 'file')
        error('load_params:missing', ...
            'params.json not found. Run `make params` (python3 params/yaml2json.py) first.');
    end
    raw = jsondecode(fileread(jsonfile));

    P = struct();
    P.meta = raw.meta;
    P.env  = raw.environment;
    P.geom = raw.geometry;
    P.sign = raw.sign_conventions;
    P.speeds = raw.speeds;
    P.trim_ref = raw.trim_reference;
    P.design_points = raw.design_points;
    P.hydro = raw.hydro;
    P.specs = raw.control_specs;
    P.prop = raw.propulsion;
    P.raw = raw;

    % ---- aero: flatten {value, source} -> value, keep sources ----------------
    [P.aero.lon, P.aero.src.lon] = flatten_coeffs(raw.aero.longitudinal);
    [P.aero.lat, P.aero.src.lat] = flatten_coeffs(raw.aero.lateral);
    P.aero.CD0    = raw.aero.drag_polar.CD0;
    P.aero.K      = raw.aero.drag_polar.K;
    P.aero.CL_max = raw.aero.stall.CL_max;
    P.aero.blend_M = raw.aero.stall.blend_M;
    % blend centre solved so peak CL = CL_max (D-04)
    P.aero.alpha_stall_lin = (P.aero.CL_max - P.aero.lon.CL0) / P.aero.lon.CL_alpha;
    P.aero.alpha_stall = calibrate_stall(P.aero.lon.CL0, P.aero.lon.CL_alpha, ...
                                         P.aero.CL_max, P.aero.blend_M);
    P.aero.Mix = raw.aero.mixing.Mix;

    % ---- actuators: degrees -> radians --------------------------------------
    names = {'elevator', 'aileron', 'rudder', 'direct_lift'};
    for k = 1:numel(names)
        a = raw.actuators.(names{k});
        P.act.(names{k}).tau       = a.tau;
        P.act.(names{k}).delta_max = deg2rad(a.delta_max_deg);
        P.act.(names{k}).rate_max  = deg2rad(a.rate_max_deg_s);
    end
    P.act.throttle.tau = raw.actuators.throttle.tau;
    P.act.throttle.min = raw.actuators.throttle.min;
    P.act.throttle.max = raw.actuators.throttle.max;

    % ---- mass properties ----------------------------------------------------
    P.mass = mass_props(raw);

    % ---- trim references in radians -----------------------------------------
    for f = {'cruise_loaded', 'cruise_unloaded'}
        t = raw.trim_reference.(f{1});
        t.alpha = deg2rad(t.alpha_deg);
        t.de    = deg2rad(t.de_deg);
        t.gamma = deg2rad(t.gamma_deg);
        P.trim_ref.(f{1}) = t;
    end
end

function a_s = calibrate_stall(CL0, CLa, CLmax, M)
% Find the sigmoid centre a_s such that max_alpha CL_blend(alpha) = CLmax.
    al = linspace(0, deg2rad(40), 2001);
    function CLpk = peak(a_s)
        num = 1 + exp(-M*(al - a_s)) + exp(M*(al + a_s));
        den = (1 + exp(-M*(al - a_s))) .* (1 + exp(M*(al + a_s)));
        sg = num ./ den;
        CL = (1 - sg) .* (CL0 + CLa*al) + sg .* (2*sign(al).*sin(al).^2.*cos(al));
        CLpk = max(CL);
    end
    a_lin = (CLmax - CL0) / CLa;
    a_s = fzero(@(a) peak(a) - CLmax, [a_lin, a_lin + deg2rad(15)]);
end

function [vals, srcs] = flatten_coeffs(s)
    vals = struct(); srcs = struct();
    f = fieldnames(s);
    for k = 1:numel(f)
        e = s.(f{k});
        if isstruct(e)
            vals.(f{k}) = e.value;
            if isfield(e, 'source'), srcs.(f{k}) = e.source; else, srcs.(f{k}) = 'unknown'; end
        else
            vals.(f{k}) = e;
            srcs.(f{k}) = 'unknown';
        end
    end
end
