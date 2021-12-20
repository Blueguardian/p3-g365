function O=eulerZYX(T)
O(2)=atan2(-T(3,1),sqrt(T(1,1)^2+T(2,1)^2));
if (abs(abs(O(2))-90)<0.00001)
    O(1)=0;
    O(3)=O(2)/abs(O(2))*atan2(T(1,2),T(2,2));
else
    O(1)=atan2(T(2,1)/cos(O(2)),T(1,1)/cos(O(2)));
    O(3)=atan2(T(3,2)/cos(O(2)),T(3,3)/cos(O(2)));
end