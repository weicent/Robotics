function flag = circle_line_intersection(A, B, C, R)
    % % For Mathematical Explanation: http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
    % INPUTS :
    % A and B are two end points of line segment
    % A: [x,y] coordinate of one end of the line segment
    % B: [x,y] coordinate of the other end of the line segment
    % C: [x,y] coordinate of center of the circle
    % R: radius of the circle
    % OUTPUT :
    % delta > 0 : no intersection
    % delta = 0 : tangent
    % delta < 0 : intersection
    
    BA = B - A; % vector from A towards B
    CA = C - A; % vector from A towards C
    proj = dot(CA, BA/norm(BA)); % length of projection of CA on BA
    % find point on the line closest to the circle
    dist=sqrt(BA(1)^2+BA(2)^2);
    if proj <= 0
        closest_point = A;
    elseif proj>dist
        closest_point = B;
    else
        closest_point = A + proj * BA / norm(BA);
    end
    D = C - closest_point; % vector from closest point on line towards C
    delta = norm(D) - R;
    % delta > 0 : no intersection
    % delta = 0 : tangent
    % delta < 0 : intersection
    if delta<=0
        flag=true;
    else
        flag=false;
    end
end