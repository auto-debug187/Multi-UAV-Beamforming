function [motorCmd,diagout]=bf_autopilot(a,x,cfg)
%BF_AUTOPILOT Acceleration -> thrust/attitude -> torques -> rotor speeds.
% All rotations are body-to-world. No formation attitude is sent to the body.
v=cfg.vehicle; raw=a;
a(3)=max(-cfg.control.maxVerticalAccel,min(cfg.control.maxVerticalAccel,a(3)));
ah=norm(a(1:2)); maxh=min(cfg.control.maxHorizontalAccel, ...
    max(0,v.g+a(3))*tan(cfg.control.maxTilt));
if ah>maxh, a(1:2)=a(1:2)*maxh/ah; end
F=v.mass*(a+[0;0;v.g]); b3=F/norm(F);
heading=[cos(cfg.yaw);sin(cfg.yaw);0]; b2=cross(b3,heading); b2=b2/norm(b2);
Rc=[cross(b2,b3) b2 b3]; R=bf_quatR(x(7:10)); w=x(11:13);
eR=0.5*bf_vee(Rc'*R-R'*Rc);
tau=-v.KR.*eR-v.Kw.*w+cross(w,v.I*w);
thrust=max(0,dot(F,R(:,3)));
fraw=v.Binv*[thrust;tau]; f=max(0,min(v.fMax,fraw));
motorCmd=sqrt(f/v.kT);
diagout=[double(norm(a-raw)>1e-9);double(any(abs(fraw-f)>1e-9)); ...
    thrust;acos(max(-1,min(1,R(3,3))))];
end
