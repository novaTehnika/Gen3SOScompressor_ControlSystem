function cost = loopCost(r)
% loopCost - scalar ranking for gain sweeps: peak error, overshoot and late
% ripple all count, and never settling is penalised.
  cost = r.peakErr + 2*r.overshoot + 20*r.ripple;
  if isnan(r.tSettle), cost = cost + 10; end
end
