function FeasibilityAndTuning
% tuning example
%
% Stefan Richter, 27th August 2008
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
width = 10;
% create new folder for storing figures
mkdir('figures');
fig_str1 = ['.',filesep,'figures',filesep,'Example13-2'];
fig_str2 = ['.',filesep,'figures',filesep,'Example13-2d'];


%% core code
A = [2  1; 0  0.5];
B = [1;0];
model = LTISystem('A',A,'B',B);
model.x.min    =   [-10; -10];
model.x.max    =   [10; 10];
model.u.min    =   -1;
model.u.max    =   1;

% penalty on states
model.x.penalty = QuadFunction(eye(2));

% Compute controllers for 3 different parameter settings +  plot their
% corresponding closed loop trajectories starting from same x(0)

% input penalty
R = {10, 2, 1};
% horizon
N = { 2, 3, 4};

c = cell(1,length(R));
for i=1:length(R)
    model.u.penalty = QuadFunction(R{i});
    ctrl = MPCController(model, N{i});
    c{i} = ctrl.toExplicit();
end

% generate a set of initial states
% idea: sample uniformly from a polytope being the Minkowski Sum of C_inf
% and the unitbox
% C_inf
interv_length = 1.5;
C_inf = model.invariantSet();
sample_box = C_inf + Polyhedron('lb',[-0.5;-0.5],'ub',[0.5; 0.5]);

% generate samples over state space
X01 = model.x.min(1):interv_length:model.x.max(1);
X02 = model.x.max(2):-interv_length:model.x.min(2);
X0 = [];
for i=1:length(X01)
    for j=1:length(X02)
        x0_temp = [X01(i); X02(j)];
        if sample_box.contains(x0_temp)
            X0 = [X0, x0_temp];
        end
    end
end

% simulate closed loop system
n_time_steps_sim = 10;
linespecs = {'k-', 'k-', 'k-'}; % for different settings 1,2,3
fname_suff = {'a', 'b', 'c'};

for i=1:length(R)
    figure(i)
    hold on;
    grid on
    cl = ClosedLoop(c{i}, model);
    for j=1:size(X0,2)
        data = cl.simulate(X0(:,j),n_time_steps_sim);
        % mark all trajectories leading to feasible closed-loop
        % trajectories with boxes otherwise use circles
        marker = 'o';
        if (size(data.X,2)==(n_time_steps_sim+1))
            marker = 's';
            plot(data.X(1,:), data.X(2,:), [linespecs{i}, marker],'LineWidth',line_width,'MarkerFaceColor','k','MarkerSize',10)
        else
            plot(data.X(1,:), data.X(2,:), [linespecs{i}, marker],'LineWidth',line_width,'MarkerSize',10)
        end
    end
    
    axis([-8 8 -10 10])
    set(gca,'LineWidth',axeswidth)
    set(gca,'FontSize', tick_font_size);
    
    title('')
    hx=xlabel('$x_{1}$');
    set(hx, 'FontSize', label_font_size);
    hy=ylabel('$x_{2}$');
    set(hy, 'FontSize', label_font_size);
    
    xl = transpose([-10 -8 -4 0 4 8 10]);
    yl = transpose(-10:5:10);
    plot(-4*ones(size(yl)),yl,'k--','LineWidth',line_width);
    set(gca,'XTick',xl);
    set(gca,'YTick',yl);
    set(gca,'XTickLabel',num2str(xl));
    set(gca,'YTickLabel',num2str(yl));
    
    % print
    figure_name = [fig_str1,fname_suff{i}];
    disp('Hit any key to print the figure and continue.');
    pause
    saveas(gcf, [figure_name,'_matlab'], 'fig');
    
    laprint(gcf, [figure_name,'_tex'],'scalefonts','off','width',width);
    
end


% Compute O_inf for all 3 controllers + plot
% O_inf
invCtrl = cell(1,length(R));
for i=1:length(R)
    invCtrl{i} = c{i}.toInvariant();
end

figure
hold on;

% plot C_inf
plot(C_inf, 'color','white','edgecolor','black','linewidth',line_width);

% 3rd controller: O_inf consists of 9 regions
plot(invCtrl{3}.optimizer,'color',[0.7 0.7 0.7],'edgecolor',[0.7 0.7 0.7],'linewidth',line_width);
% NOTE: By inspection, we found the following vertices 
V3 = [-7.6667,10;-6.4583,10;-4.6667,8;-3.8,7.2;1,0;7.6667,-10;...
    6.4583,-10;4.6667,-8;3.8,-7.2;-1,0;-7.6667,10];
plot(V3(:,1), V3(:,2), 'k-', 'Linewidth', line_width);

% 2nd controller: O_inf consists of 5 regions
plot(invCtrl{2}.optimizer,'color',[0.5 0.5 0.5],'edgecolor',[0.5 0.5 0.5],'linewidth',line_width);
% NOTE: By inspection, we found the following vertices 
V2 = [-6.02,8.88;-3.4167,6;-1,3;5,-6;7.6667,-10;6.8056,-10;6.02,-8.88;...
    3.4167,-6;1,-3;-7.6667,10;-6.8056,10;-6.02,8.88];
plot(V2(:,1), V2(:,2), 'k-', 'Linewidth', line_width);

% 1st controller: O_inf = {0}
plot(0, 0, ['k','.'], 'Linewidth', line_width, 'MarkerSize',15);

% plot the initial conditions [-4,7] and [-4, 8.5] that is used in the discussion of the
% example
plot(-4, 7, 'kx', 'Linewidth', line_width, 'MarkerSize',10);
ht1=text(-3, 6.6, '$x(0) = [-4, 7]$');
set(ht1,'FontSize',label_font_size);
plot(-4, 8.5, 'kx', 'Linewidth', line_width, 'MarkerSize',10);
ht2=text(-3, 8.7, '$x(0) = [-4, 8.5]$');
set(ht2,'FontSize',label_font_size);

grid on
axis tight


set(gca,'LineWidth',axeswidth)
set(gca,'FontSize', tick_font_size);

title('')
hx=xlabel('$x_{1}$');
set(hx, 'FontSize', label_font_size);
hy=ylabel('$x_{2}$');
set(hy, 'FontSize', label_font_size);

xl = transpose([-10 -8 -4 0 4 8 10]);
yl = transpose(-10:5:10);
plot(-4*ones(size(yl)),yl,'k--','LineWidth',line_width);
set(gca,'XTick',xl);
set(gca,'YTick',yl);
set(gca,'XTickLabel',num2str(xl));
set(gca,'YTickLabel',num2str(yl));

%% save book figure
% print
disp('Hit any key to print the figure and continue.');
pause
saveas(gcf, [fig_str2,'_matlab'], 'fig');

laprint(gcf, [fig_str2,'_tex'],'scalefonts','off','width',width);

end
