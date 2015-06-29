%2 link model with horizontal joints
mdl_twolink_mdh

%plot with both links at 0
%twolink.plot([0,0])     

%set link 2 position
%distance joint 1 to joint 2
twolink.links(2).a = 0.355

%distance joint 2 to tool
twolink.tool(1,4) = 0.22

%distance floor to link 1 height (currently not really needed)
twolink.base(3,4) = 0.54

%generate trajectory

%path = [ 1 0 0; 1 0 0; 0 0 0; 0 2 0; 1 2 0; 1 2 0; 0 1 0; 0 1 0; 1 1 0; 1 1 0];
%path = [ 0 0 0; 0.1 0 0; 0.1 0 0; 0 0 0; 0.1 0 0; 0 0 0]; 

%create circle
t = linspace(0,2*pi,50);

r = 0.1
x = r*cos(t)
y = r*sin(t)
x = x.'
y = y.'

path = [x,y,zeros(length(x),1)]

%plot3(path(:,1), path(:,2), path(:,3), 'color', 'k', 'LineWidth', 2)

% x y z speedlimit, xyz initial position, timestep, time acceleration is
% applied (strange parameter as smaller values give higher acc/speed?)
%p = 
%mstraj(path, [2000 2000 2000], [], [0 0 0], 0.1, 0.9)



p = mstraj(path, [1 1 1], [], [0 0 0], 0.1, 0.1)

mstraj(path, [1 1 1], [], [0 0 0], 0.1, 0.1)

%scale the trajectory
Tp = transl(1 * p);

%transform the trajectory
Tp = homtrans( transl(0.4, 0, 0), Tp);

disp(['Calculating IK...'])

%calc ikine for the trajectory, start point,  mask which dofs to use/ignore
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

disp(['Done'])
