function x = make_state(varargin)
%MAKE_STATE  Build an eom state vector from named fields.
%   x = make_state('pos',[0 0 -100], 'vb',[20 0 2], 'euler',[0 0.1 0], ...
%                  'pqr',[0 0 0], 'act',[de da dr dT dL])
% Unspecified fields are zero; quaternion defaults to identity; dT to 0.
    Q = quat_utils();
    pos = [0 0 0]; vb = [0 0 0]; eul = [0 0 0]; pqr = [0 0 0]; act = [0 0 0 0 0];
    for k = 1:2:numel(varargin)
        switch varargin{k}
            case 'pos',   pos = varargin{k+1};
            case 'vb',    vb  = varargin{k+1};
            case 'euler', eul = varargin{k+1};
            case 'pqr',   pqr = varargin{k+1};
            case 'act',   act = varargin{k+1};
            otherwise, error('make_state:field', 'unknown field %s', varargin{k});
        end
    end
    quat = Q.from_euler(eul(1), eul(2), eul(3));
    x = [pos(:); vb(:); quat(:); pqr(:); act(:)];
end
