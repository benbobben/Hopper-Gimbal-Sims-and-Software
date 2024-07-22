function eul = Quaternion_To_Euler(q)
    
    qw = q(4);
    qx = q(1);
    qy = q(2);
    qz = q(3);
    q_mag = qw^2 +qx^2 +qy^2 +qz^2;
    
    q = q./q_mag;
    
    % Pre-allocate output
    eul = zeros(1, 3, 'like', q);
    
    % Cap all inputs to asin to 1, since values >1 produce complex
    % results
    % Since the quaternion is of unit length, this should never happen,
    % but some code generation configuration seem to hit this edge case
    % under some circumstances.
    aSinInput = -2*(qx.*qz-qw.*qy);
    mask1 = aSinInput >= 1 - 10*eps(class(qw));
    mask2 = aSinInput <= -1 + 10*eps(class(qw));
    aSinInput(mask1) = 1;
    aSinInput(mask2) = -1 ;
    mask = mask1 | mask2;
    
    eul = [ atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 ), ...
        asin( aSinInput ), ...
        atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 )];
    eul(mask, 1) = -sign(aSinInput(mask,1)).* 2 .* atan2(qx(mask,1), qw(mask,1));
    eul(mask, 3) = 0;
    
    % Check for complex numbers
    if ~isreal(eul)
        eul = real(eul);
    end
    
%     if nargout > 1
%         eul; % workaround for g2380983 (const folding heurstic issue during codegen)
%         eulAlt = robotics.core.internal.generateAlternateEulerAngles(eul, seq);
%     end
    
    
end



%%%%%%%%%
%% NOT FUNCTIONING 
% function eul = Quaternion_To_Euler(q)
%     q = Get_Unit_Quaternion(q);
% 
%     qw = q(4);
%     qx = q(1);
%     qy = q(2);
%     qz = q(3);
%     
%     % Pre-allocate output
%     eul = zeros( 1,3 );
%     
%     % roll (x-axis rotation)
%     roll_sin = 2 * (qx * qw + qy * qz);
%     roll_cos = 1 - 2 * (qx^2 + qy^2);
%     roll = atan2(roll_sin, roll_cos);
% 
%     % pitch (y-axis rotation)
%     pitch_sin = 2 * (qw * qy - qz * qx);
%     if abs(pitch_sin) >=1
%         if pitch_sin >= 0 
%             pitch = pi/2;
%         else
%             pitch = -pi/2;
%         end
%     else
%         pitch = asin(pitch_sin);
%     end
% 
%     % yaw (z-axis rotation)
%     yaw_sin = 2 * (qw * qz + qx * qy);
%     yaw_cos = 1 - 2 * (qy^2 + qz^2);
%     yaw = atan2(yaw_sin, yaw_cos);
%     
%     Euler = [roll,pitch,yaw];
% end






% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Equations from https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
% function eul = Quaternion_To_Euler(q)
% %   q = Get_Unit_Quaternion(q);
% 
%     qw = q(4);
%     qx = q(1);
%     qy = q(2);
%     qz = q(3);
%     
%     % Pre-allocate output
%     eul = zeros( 1,3 );
%     
%     % Cap all inputs to asin to 1, since values >1 produce complex
%     % results
%     % Since the quaternion is of unit length, this should never happen,
%     % but some code generation configuration seem to hit this edge case
%     % under some circumstances.
% 
% 
%     aSinInput = 2*(qx*qw-qy*qz);
% 
%     pitch = asin(aSinInput);
% 
%     if pitch == pi/2
%         roll = 0;
%         yaw = -2*tan2(qx,qw);
%     elseif pitch == -pi/2 
%         roll = 0;
%         yaw = 2*tan2(qx,qw);
%     else
%         roll = atan2( 2*(qw*qx+qy*qz), qw^2 - qx^2 - qy^2 + qz^2 );
%         yaw = atan2(2*(qw*qz + qx * qy), qw^2+qx^2-qy^2-qz^2);
%     end
%     
%     eul = [roll,pitch,yaw];
% 
%     
% end
