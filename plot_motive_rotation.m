function plot_motive_rotation(opti_data,angle_type)

if istable(opti_data)
    switch angle_type
        case 'quaternion'
            figure;
            set(gcf,'Position',[1 1 1000 900]);
            subplot(4,1,1);
            plot(opti_data.Time,opti_data.X_Rotation,'r','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: X','FontSize',16);
            subplot(4,1,2);
            plot(opti_data.Time,opti_data.Y_Rotation,'g','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Y','FontSize',16);
            subplot(4,1,3);
            plot(opti_data.Time,opti_data.Z_Rotation,'b','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Z','FontSize',16);
            subplot(4,1,4);
            plot(opti_data.Time,opti_data.W_Rotation,'k','LineWidth',2);
            title('Rigid Body Rotation: W','FontSize',16);
            set(gca,'FontSize',12);
            xlabel('Time (s)','FontSize',16);

        case 'euler'
            figure;
            set(gcf,'Position',[1 1 1000 900]);
            subplot(3,1,1);
            plot(opti_data.Time,opti_data.X_Rotation,'r','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: X','FontSize',16);
            subplot(3,1,2);
            plot(opti_data.Time,opti_data.Y_Rotation,'g','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Y','FontSize',16);
            subplot(3,1,3);
            plot(opti_data.Time,opti_data.Z_Rotation,'b','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Z','FontSize',16);
            xlabel('Time (s)','FontSize',16);
    end
else

    switch angle_type
        case 'quaternion'
            figure;
            set(gcf,'Position',[1 1 1000 900]);
            subplot(4,1,1);
            plot(opti_data.time,opti_data.rigidbodies.data(:,1),'k','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: X','FontSize',16);
            subplot(4,1,2);
            plot(opti_data.time,opti_data.rigidbodies.data(:,2),'k','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Y','FontSize',16);
            subplot(4,1,3);
            plot(opti_data.time,opti_data.rigidbodies.data(:,3),'k','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Z','FontSize',16);
            subplot(4,1,4);
            plot(opti_data.time,opti_data.rigidbodies.data(:,4),'k','LineWidth',2);
            title('Rigid Body Rotation: W','FontSize',16);
            set(gca,'FontSize',12);
            xlabel('Time (s)','FontSize',16);

        case 'euler'
            figure;
            set(gcf,'Position',[1 1 1000 900]);
            subplot(3,1,1);
            plot(opti_data.time,opti_data.rigidbodies.data(:,1),'k','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: X','FontSize',16);
            subplot(3,1,2);
            plot(opti_data.time,opti_data.rigidbodies.data(:,2),'k','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Y','FontSize',16);
            subplot(3,1,3);
            plot(opti_data.time,opti_data.rigidbodies.data(:,3),'k','LineWidth',2);
            set(gca,'FontSize',12);
            title('Rigid Body Rotation: Z','FontSize',16);
            xlabel('Time (s)','FontSize',16);
            print('opti_rot','-dpng','-r300');
    end
end
