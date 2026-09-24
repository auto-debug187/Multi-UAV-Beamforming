function [n,nd,ndd]=bf_unitDerivatives(r,rd,rdd)
L=norm(r); assert(L>1e-8,'Cannot normalize a near-zero direction.');
n=r/L; Ld=dot(n,rd);
nd=(rd-n*Ld)/L;
ndd=(rdd-n*dot(n,rdd))/L-2*Ld/L*nd-n*dot(nd,nd);
end
