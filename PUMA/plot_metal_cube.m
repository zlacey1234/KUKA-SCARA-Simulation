function plot_metal_cube(Transformation_B_Cube_Corner)
    Transformation_Cube_Corner_to_Center = [1 0 0 0.05;
                                            0 1 0 0.05;
                                            0 0 1 0.05;
                                            0 0 0 1];
                                        
    Transformation_B_to_Center = ...
        Transformation_B_Cube_Corner * ...
        Transformation_Cube_Corner_to_Center;
    
    Cube_color = [0.9 0.9 0.9];
    
    plot_cube(Transformation_B_to_Center, 0.1, 0.1, 0.1, ...
        Cube_color)
    
end