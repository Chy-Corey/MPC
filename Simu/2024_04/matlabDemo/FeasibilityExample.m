function FeasibilityExample
% feasibility study on double integrator example
% 
% Thomas Besselmann, 18.08.08
% Francesco Borrelli July 05 2009
% M. Herceg, ETH Zurich, 2014
%
% requires MPT3

close all

%% printing parameters
label_font_size = 14;
tick_font_size = 10;
line_width = 0.8;
axeswidth=0.2;
% create new folder for storing figures
mkdir('figures');
figure_name1 = ['.',filesep,'figures',filesep,'Example13-1a'];
figure_name2 = ['.',filesep,'figures',filesep,'Example13-1c'];
figure_name3 = ['.',filesep,'figures',filesep,'Example13-1b'];


%% core code
% system dynamics
A=[1 1; 0 1];
B = [0;1];
model = LTISystem('A',A,'B',B);
model.x.min = [-5;-5];
model.x.max = [ 5; 5];
model.u.min=-0.5;
model.u.max=0.5;

% penalties
model.x.penalty = QuadFunction(eye(2));
model.u.penalty = QuadFunction(10);
model.x.with('terminalPenalty');
model.x.terminalPenalty = model.x.penalty;

% online controller
horizon = 3;
ctrl = MPCController(model, horizon);

% plot the openloop trajectory for two initial points
x0 = [-4.5, -4.5;
         2, 3];

fig1=figure;
hold on
for kk=1:2
    x = x0(:,kk);
    X = x;
    for j=1:16
        [u, feasible, openloop] = ctrl.evaluate(x);
        if feasible
            plot(openloop.X(1,:), openloop.X(2,:), 'k*-.','LineWidth',line_width)
            x = openloop.X(:,2);
            X = [X, x];
        end
    end
    if kk==1
        plot(X(1,:), X(2,:), 'ks-','LineWidth',line_width,'MarkerSize',10); 
    else
        plot(X(1,:), X(2,:), 'ko-','LineWidth',line_width,'MarkerSize',10);
    end
end
grid on
axis([-5 5 -0.5 3.5])

set(gca,'LineWidth',axeswidth)
set(gca,'FontSize', tick_font_size);

title('1')
hx=xlabel('$x_{1}$');
set(hx, 'FontSize', label_font_size);
hy=ylabel('$x_{2}$');
set(hy, 'FontSize', label_font_size);

xl = transpose(-5:1:5);
yl = transpose(-5:1:5);
set(gca,'XTick',xl);
set(gca,'YTick',yl);
set(gca,'XTickLabel',num2str(xl));
set(gca,'YTickLabel',num2str(yl));

% print
disp('Hit any key to continue.');
% pause

  
% Compute O_inf and X0 for given controller and plot
c = ctrl.toExplicit();
invCtrl = c.toInvariant();
fig2=figure;
hold on;
plot(c.optimizer, 'color','white','linewidth',line_width);
plot(invCtrl.optimizer, 'color',[0.5 0.5 0.5],'edgecolor',[0.5 0.5 0.5],'linewidth',line_width);
grid on

axis([model.x.min(1),model.x.max(1),model.x.min(2),model.x.max(2)]);

set(gca,'LineWidth',axeswidth)
set(gca,'FontSize', tick_font_size);

title('2')
hx=xlabel('$x_{1}$');
set(hx, 'FontSize', label_font_size);
hy=ylabel('$x_{2}$');
set(hy, 'FontSize', label_font_size);

xl = transpose(-5:1:5);
yl = transpose(-5:1:5);
set(gca,'XTick',xl);
set(gca,'YTick',yl);
set(gca,'XTickLabel',num2str(xl));
set(gca,'YTickLabel',num2str(yl));

% print
disp('Hit any key to continue.');
% pause


% Compute feasibility by gridding
cl = ClosedLoop(ctrl, model);
fig3=figure;
hold on;
for i=model.x.min(1):0.5:model.x.max(1)
    for j=model.x.min(2):0.5:model.x.max(2)
        x0=[i;j];
        data = cl.simulate(x0,30);
        if size(data.X,2)==31
            % feasible
            plot(i,j,'ks','MarkerFaceColor','k','MarkerSize',10)
        else
            % infeasible
            plot(i,j,'ko','MarkerSize',10)
        end
    end
end
hp=plot(invCtrl.optimizer.convexHull, 'wire', true,'linestyle','--','linewidth',line_width);
set(hp,'FaceColor','none');

grid off


axis([model.x.min(1),model.x.max(1),model.x.min(2),model.x.max(2)]);
set(gca,'LineWidth',axeswidth)
set(gca,'FontSize', tick_font_size);

title('3')
hx=xlabel('$x_{1}$');
set(hx, 'FontSize', label_font_size);
hy=ylabel('$x_{2}$');
set(hy, 'FontSize', label_font_size);

xl = transpose(-5:1:5);
yl = transpose(-5:1:5);
set(gca,'XTick',xl);
set(gca,'YTick',yl);
set(gca,'XTickLabel',num2str(xl));
set(gca,'YTickLabel',num2str(yl));

% print
disp('Hit any key to continue.');
% pause


%% save book figures
saveas(fig1, [figure_name1,'_matlab'], 'fig');

% laprint(fig1, [figure_name1,'_tex'],'scalefonts','off');

saveas(fig2, [figure_name2,'_matlab'], 'fig');

% laprint(fig2, [figure_name2,'_tex'],'scalefonts','off');

saveas(fig3, [figure_name3,'_matlab'], 'fig');

% laprint(fig3, [figure_name3,'_tex'],'scalefonts','off');



end

