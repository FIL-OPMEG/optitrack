function plot_motive_translation(opti_data, angle_type)

switch angle_type
    case 'quaternion'
        figure;
        set(gcf,'Position',[1 1 1000 900]);
        subplot(3,1,1);
        plot(opti_data.time,opti_data.rigidbodies.data(:,5),'k','LineWidth',2);
        set(gca,'FontSize',12);
        ylabel('cm','FontSize',16);
        title('Rigid Body Position: X','FontSize',16);
        subplot(3,1,2);
        plot(opti_data.time,opti_data.rigidbodies.data(:,6),'k','LineWidth',2);
        set(gca,'FontSize',12);
        ylabel('cm','FontSize',16);
        title('Rigid Body Position: Y','FontSize',16);
        subplot(3,1,3);
        plot(opti_data.time,opti_data.rigidbodies.data(:,7),'k','LineWidth',2);
        set(gca,'FontSize',12);
        ylabel('cm','FontSize',16);
        title('Rigid Body Position: Z','FontSize',16);
        xlabel('Time (s)','FontSize',16);
        
    case 'euler'
        figure;
        set(gcf,'Position',[1 1 1000 900]);
        subplot(3,1,1);
        plot(opti_data.time,opti_data.rigidbodies.data(:,4),'k','LineWidth',2);
        set(gca,'FontSize',12);
        ylabel('Distance','FontSize',16);
        title('Rigid Body Position: X','FontSize',16);
        subplot(3,1,2);
        plot(opti_data.time,opti_data.rigidbodies.data(:,5),'k','LineWidth',2);
        set(gca,'FontSize',12);
        ylabel('Distance','FontSize',16);
        title('Rigid Body Position: Y','FontSize',16);
        subplot(3,1,3);
        plot(opti_data.time,opti_data.rigidbodies.data(:,6),'k','LineWidth',2);
        set(gca,'FontSize',12);
        ylabel('Distance','FontSize',16);
        title('Rigid Body Position: Z','FontSize',16);
        xlabel('Time (s)','FontSize',16);
end
