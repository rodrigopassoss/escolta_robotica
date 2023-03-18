function result = angle_diff(angle1, angle2)
  result = mod(angle1 - angle2 + (angle1.^0).*pi, (angle1.^0).*2*pi) - (angle1.^0).*pi;
end