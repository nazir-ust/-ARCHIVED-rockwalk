function [ke_value, pe_value] = compute_ke_pe_values(ke_fun, pe_fun, dqdt1, dqdt2, dqdt4, dqdt5, dqdt6, q4, q5, q6)

  ke_value = subs(ke_fun)
  pe_value = subs(pe_fun)

end
