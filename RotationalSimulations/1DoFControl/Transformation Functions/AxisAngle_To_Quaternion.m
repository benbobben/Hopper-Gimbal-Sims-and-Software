function q = AxisAngle_To_Quaternion(AxisAngle)
    
    theta = AxisAngle(4);
    x = AxisAngle(1);
    y = AxisAngle(2);
    z = AxisAngle(3);
    
    qw = cos(theta/2);
    qx = x * sin(theta/2);
    qy = y * sin(theta/2);
    qz = z * sin(theta/2);

    q = [qx,qy,qz,qw];
    
end