
%2 link model with horizontal joints
mdl_twolink_mdh

%plot with both links at 0
%twolink.plot([0,0])     

%transmission of joint 60:1
JOINT_1_TRANSMISSION = 60

%transmission of joint 51:1
JOINT_2_TRANSMISSION = 51

%transmission of joint 50:1
JOINT_3_TRANSMISSION = 50




%set link 2 position
%distance joint 1 to joint 2
twolink.links(2).a = 0.355

%distance joint 2 to tool
twolink.tool(1,4) = 0.22

%distance floor to link 1 height (currently not really needed)
twolink.base(3,4) = 0.54


valid_zero_offset = pi-pi/4

%set joint limits
twolink.qlim = [-valid_zero_offset, valid_zero_offset; -valid_zero_offset, valid_zero_offset]




%generate trajectory

%create circle
t = linspace(0,2*pi,50);

r = 0.1
x = r*cos(t)
y = r*sin(t)
x = x.'
y = y.'



%path = [x,y,zeros(length(x),1)]
%not wokring anymore no ik found: path = [ 1 0 0; 1 0 0; 0 0 0; 0 2 0; 1 2 0; 1 2 0; 0 1 0; 0 1 0; 1 1 0; 1 1 0];

%square
%path = [0.45 0 0; 0.45 0.1 0; 0.35 0.01 0; 0.35 0 0; 0.45 0 0]; 

%line
path = [0.57 0.1 0; 0.30 0.1 0; 0.57 0.1 0]; 

%plot3(path(:,1), path(:,2), path(:,3), 'color', 'k', 'LineWidth', 2)

% x y z speedlimit, empty, xyz initial position, timestep, time acceleration is
% applied (strange parameter as smaller values give higher acc/speed?)
%p = 
%mstraj(path, [2000 2000 2000], [], [0 0 0], 0.1, 0.9)

timestep = 0.007

p = mstraj(path, [0.1 0.1 0.1], [], [path(1,1) path(1,2) 0], timestep, 0.1)

%mstraj(path, [0.1 0.1 0.1], [], [path(1,1) path(1,2) 0], 0.01, 0.1)

%scale the trajectory
Tp = transl(p);



%transform the trajectory
Tp = homtrans( transl(0, 0, 0), Tp)





disp(['Calculating IK...'])

%calc ikine for the trajectory, start point,  mask which dofs to use/ignore
%q_out = twolink.ikine(Tp,[0 0], [1 1 0 0 0 0], 'plot')
[q_out,err,exitflag] = twolink.ikcon(Tp)


%simulate the trajectory
%twolink.plot(q_out)


%get rid of first value
q_out = q_out(2:end,:)

%conversion from radian angles to encoder steps
q_out(:,1) = radians_to_encoder_steps(q_out(:,1),JOINT_1_TRANSMISSION)
q_out(:,2) = radians_to_encoder_steps(q_out(:,2),JOINT_2_TRANSMISSION)

wave1.signals.values = [q_out(:,1)]
wave1.time = []

wave2.signals.values = [q_out(:,2)]
wave2.time = []

disp(['Done'])

twolink.fkine([0 0])


