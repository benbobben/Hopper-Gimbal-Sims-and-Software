function unit_q =  Get_Unit_Quaternion(q)

    qw = q(4);
    qx = q(1);
    qy = q(2);
    qz = q(3);
    q_mag = qw^2 +qx^2 +qy^2 +qz^2;
    
    unit_q = q./q_mag;

end 