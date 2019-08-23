function [ke_fun, pe_fun] = compute_ke_pe_fun(R,d,h,m)

  load ke_pe.mat

  ke_fun = matlabFunction(subs(ke))
  pe_fun = matlabFunction(subs(pe))

end
