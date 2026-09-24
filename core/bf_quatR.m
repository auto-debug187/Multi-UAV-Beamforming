function R=bf_quatR(q)
% Scalar-first quaternion; maps BODY FLU vectors into WORLD z-up vectors.
q=q(:)/max(norm(q),eps); w=q(1); v=q(2:4);
R=(w*w-v'*v)*eye(3)+2*(v*v')+2*w*bf_skew(v);
end
