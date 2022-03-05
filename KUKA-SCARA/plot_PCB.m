function plot_PCB(Transformation_B_S, Transformation_S_PCB, ...
    Transformation_S_Chip1, Transformation_S_Chip2, ...
    Transformation_S_Chip3, Transformation_S_Chip4, plot_Chip_boolean)
    
    %
    Transformation_B_PCB = Transformation_B_S*Transformation_S_PCB;
    
    %
    Transformation_B_Chip1 = Transformation_B_S*Transformation_S_Chip1;
    Transformation_B_Chip2 = Transformation_B_S*Transformation_S_Chip2;
    Transformation_B_Chip3 = Transformation_B_S*Transformation_S_Chip3;
    Transformation_B_Chip4 = Transformation_B_S*Transformation_S_Chip4;
    
    PCB_color = [0.5 0.5 0.5];
    plot_cube(Transformation_B_PCB, 0.1, 0.1,-0.002, PCB_color)
    
    if plot_Chip_boolean == 1
        plot_chip(Transformation_B_Chip1)
        plot_chip(Transformation_B_Chip2)
        plot_chip(Transformation_B_Chip3)
        plot_chip(Transformation_B_Chip4)
    end
end