function plot_feeder(Transformation_B_feeder)
    
    Length_feeder = 0.1;
    
    Transformation_Feeder_center = [1 0 0 Length_feeder/2 - 0.005;
                                    0 1 0 0;
                                    0 0 1 0;
                                    0 0 0 1];
    Transformation_B_Feeder_center = ...
        Transformation_B_feeder*Transformation_Feeder_center;
    
    Feeder_color = [0.3 0.3 0.3];
    plot_cube(Transformation_B_Feeder_center, 0.1, 0.012, -0.02, ...
        Feeder_color)
    
    plot_chip(Transformation_B_feeder)
end