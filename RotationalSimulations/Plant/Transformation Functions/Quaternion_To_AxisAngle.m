function axisAngle = Quaternion_To_AxisAngle(q)

    q = Get_Unit_Quaternion(q);
    qw = q(4);
    qx = q(1);
    qy = q(2);
    qz = q(3);
    
    theta = 2*acos(qw);

    if theta == 0 
        x = 1;
        y = 0;
        z = 0;
    else
        x = qx/(sin(theta/2));
        y = qy/(sin(theta/2));
        z = qz/(sin(theta/2));
    end
    
    %Vector comes first in our axis representation and angle comes last
    axisAngle = [x,y,z,theta];

end