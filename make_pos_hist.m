function make_pos_hist(pos_data,color)

figure;
h = histfit(pos_data,50);

h(1).FaceColor = color;
h(2).Color = [.2 .2 .2];
set(gca,'FontSize',18);
xlabel('Distance (cm)','FontSize',22);
ylabel('Frequency','FontSize',22);

end


