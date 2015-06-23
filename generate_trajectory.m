%2 link model with horizontal joints
mdl_twolink_mdh

%plot with both links at 0
%twolink.plot([0,0])     

%set link 2 position
twolink.links(2).a = 0.355

twolink.tool(1,4) = 0.22

twolink.base(3,4) = 0.54

%generate trajectory

%path = [ 1 0 0; 1 0 0; 0 0 0; 0 2 0; 1 2 0; 1 2 0; 0 1 0; 0 1 0; 1 1 0; 1 1 0];
%path = [ 1 0 0; 1 0 0; 0 0 0; 1 0 0]; 

t = linspace(0,2*pi,50);

r = 1
x = r*cos(t)
y = r*sin(t)
x = x.'
y = y.'

path = [x,y,zeros(length(x),1)]

%plot3(path(:,1), path(:,2), path(:,3), 'color', 'k', 'LineWidth', 2)

% x y z speedlimit, xyz initial position, timestep, time acceleration is applied
p = mstraj(path, [100 100 100], [], [0 0 0], 0.05, 0.2)

%scale the trajectory
Tp = transl(0.1 * p);

%transform the trajectory
Tp = homtrans( transl(0.4, 0, 0), Tp);

%calc ikine for the trajecotry  mask which dofs to use/ignore
q_out = twolink.ikine(Tp,[-0.7 1], [1 1 0 0 0 0])


%simulate the trajectory
%twolink.plot(q_out)


%conversion to encoder steps
theta_1_range_steps = 89000
theta_1_range_radian = 4.53785606
radians_per_step = theta_1_range_radian / theta_1_range_steps
q_out = q_out/radians_per_step

wave1.signals.values = [q_out(:,1)]
wave1.time = []

wave2.signals.values = [q_out(:,2)]
wave2.time = []
