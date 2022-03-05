function plot_chip(Transformation_B_Chip)
    Transformation_Chip_Front = [1 0 0 0.0025;
                                 0 1 0 0;
                                 0 0 1 0;
                                 0 0 0 1];
    Transformation_B_Chip_Front = ...
        Transformation_B_Chip*Transformation_Chip_Front;
    
    Transformation_Chip_Back = [1 0 0 -0.0025;
                                0 1 0 0;
                                0 0 1 0;
                                0 0 0 1];
    Transformation_B_Chip_Back = ...
        Transformation_B_Chip*Transformation_Chip_Back;
    
    Chip_Front_color = [1 0 0];
    
    Chip_Back_color = [0 0 1];
    
    plot_cube(Transformation_B_Chip_Front, 0.005, 0.01, 0.002, ...
        Chip_Front_color)
    
    plot_cube(Transformation_B_Chip_Back, 0.005, 0.01, 0.002, ...
        Chip_Back_color)
    
end