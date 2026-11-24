function [x,P,nis,dof] = corregir_estado(x,P,z,Q)
    idx = find(isfinite(z)); dof = numel(idx); nis = NaN;
    if isempty(idx), return; end
    H = eye(3); H = H(idx,:); V = Q(idx,idx);
    nu = z(idx)-H*x;
    a = find(idx==3); nu(a) = wrapToPi(nu(a));
    S = H*P*H'+V; K = (P*H')/S;
    nis = nu'*(S\nu);
    x = x+K*nu; x(3) = wrapToPi(x(3));
    A = eye(3)-K*H; P = A*P*A'+K*V*K'; P = (P+P')/2;
end
