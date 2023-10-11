function [K,S] = LQR(ode,q)
    AB = jacobian(rhs(ode),q);
    AB = vpa(subs(AB,q,[0,0,0,0,0,0,0,eps]))

end