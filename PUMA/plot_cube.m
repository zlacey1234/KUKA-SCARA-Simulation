function plot_cube(Transformation_B_center_cube, ...
    X_Length, Y_Length, Z_Length, cdata)
    
    % Define the vertices of the unit cubic shape
    ver = [1 1 0;
           0 1 0;
           0 1 1;
           1 1 1;
           0 0 1;
           1 0 1;
           1 0 0;
           0 0 0];
    
    %  Define the faces of the unit cubic
    fac = [1 2 3 4;
           4 3 5 6;
           6 7 8 5;
           1 2 8 7;
           6 7 1 4;
           2 3 5 8];
    
    R = Transformation_B_center_cube(1:3, 1:3);
    p_center = Transformation_B_center_cube(1:3, 4);
    
    cube = [ver(:,1)*X_Length - X_Length/2, ...
            ver(:,2)*Y_Length - Y_Length/2, ...
            ver(:,3)*Z_Length - Z_Length/2];
    
    for i = 1:length(cube)
        cube(i,:) = (R*cube(i,:)')';
    end
    
    % Plot cube
    cube = [cube(:,1) + p_center(1), ...
            cube(:,2) + p_center(2), ...
            cube(:,3) + p_center(3)];
    patch('Faces', fac, 'Vertices', cube, 'FaceVertexCData', cdata, ...
        'FaceColor','flat');
end