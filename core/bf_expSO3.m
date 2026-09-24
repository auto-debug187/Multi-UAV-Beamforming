function R=bf_expSO3(v)
t=norm(v); S=bf_skew(v);
if t<1e-7
    R=eye(3)+(1-t^2/6)*S+(0.5-t^2/24)*(S*S);
else
    R=eye(3)+sin(t)/t*S+(1-cos(t))/t^2*(S*S);
end
end
