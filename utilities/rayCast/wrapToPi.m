function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end