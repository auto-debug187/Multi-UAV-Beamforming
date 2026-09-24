function dx=bf_plantRHS(t,x,motorCmd,cfg)
%BF_PLANTRHS Full 6DOF Newton-Euler plant + four first-order rotor speeds.
v=cfg.vehicle; vel=x(4:6); q=x(7:10); w=x(11:13);
rotor=max(0,min(v.omegaMax,x(14:17))); f=v.kT*rotor.^2;
wrench=v.B*f; R=bf_quatR(q);
air=zeros(3,1);
if cfg.wind.enabled
    air=cfg.wind.velocity+cfg.wind.gust.*[sin(0.7*t);sin(0.53*t+0.4);sin(0.31*t)];
end
forceDrag=-v.drag.*(vel-air);
acc=(R*[0;0;wrench(1)]+forceDrag)/v.mass-[0;0;v.g];
qdot=0.5*[-dot(q(2:4),w);q(1)*w+cross(q(2:4),w)];
qdot=qdot+(1-dot(q,q))*q; % norm stabilization, no Euler-angle singularities
wdot=v.I\(wrench(2:4)-cross(w,v.I*w)-v.angularDrag.*w);
cmd=max(0,min(v.omegaMax,motorCmd));
dx=[vel;acc;qdot;wdot;(cmd-x(14:17))/v.motorTau];
end
