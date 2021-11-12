function T = TDH(a,d,alpha,theta)

T=  [ cos(theta)                        -sin(theta)           0   a;
      cos(alpha)*sin(theta)   cos(alpha)*cos(theta) -sin(alpha)   0;
      sin(alpha)*sin(theta)   sin(alpha)*cos(theta)  cos(alpha)   d;
                          0                      0           0    1];