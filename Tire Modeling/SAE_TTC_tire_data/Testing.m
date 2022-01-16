i=1;
for m=1:length(FZ_binvalues)
    for n=1:length(IA_binvalues)        
        [F_M{i,n,m},S_M{i,n,m}] = MagicOutput(MagicFormula([B_surf_IA_P{i}(IA_binvalues(n),FZ_binvalues(n)),...
            E_surf_IA_P{i}(IA_binvalues(n),FZ_binvalues(m))],Slip_fit),...
            Slip_fit,Mu_surf_IA_P{i}(IA_binvalues(n),FZ_binvalues(m)),...
            FZ_binvalues(m),CS_surf_IA_P{i}(IA_binvalues(n),FZ_binvalues(m)),datamode);
        if(i==1)
            if(m==1 && n==1)
                figure
                ax1 = axes;
                hold on
                if datamode == 1
                    FvsS_Title = 'F_x vs. SR';
                    yLab = 'F_x (N)'; xLab = 'SR(-)';
                else
                    FvsS_Title = 'F_y vs. SA';
                    yLab = 'F_y (N)'; xLab = 'SA(-)';
                end
                title(FvsS_Title);
                xlabel(xLab);
                ylabel(yLab);
                grid on
            end
            if(n==1)
                [Ft,SRt]= MagicOutput(F_bar_fzia{i,n,m},S_bar_fzia{i,n,m},...
                    Mu_surf_IA_P{i}(IA_binvalues(n),FZ_binvalues(m)),...
                    FZ_binvalues(m),CS_surf_IA_P{i}(IA_binvalues(n),FZ_binvalues(m)),datamode);
                
                % Plotting the Line fit and the Raw Data Points
                    color = [(linspace(0,1,5))' (linspace(1,0,5))' ([0 .5 1 0.85 0])'];
                    % Matrix was created to have the same color betweeen data and fit line. Change the amount of values if more colors are needed.
                plot(ax1,SRt,Ft,'.','Color',[color(m,1) (color(m,2)) color(m,3)],'HandleVisibility','off');
                FZ_binvalues_r = round(FZ_binvalues,2);
                plot(S_M{i,n,m}',F_M{i,n,m}','Color',[color(m,1) color(m,2) color(m,3)],'LineWidth',1,...
                    'DisplayName',[mat2str(FZ_binvalues_r(m)) ' N']);  
            end     
        end
    end
end

    % Creating the legend for figure. 
    h = legend(ax1,'show');
    set(h,'Location','eastoutside');
    htitle = get(h,'Title');
    set(htitle,'String','F_z (N)','FontSize',10);