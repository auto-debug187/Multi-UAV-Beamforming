function A=bf_leftJacInv(v)
t=norm(v); S=bf_skew(v);
if t<1e-5, b=1/12+t^2/720;
else, b=(1-0.5*t/tan(0.5*t))/t^2; end
A=eye(3)-0.5*S+b*S*S;
end
