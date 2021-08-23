%tspan = [0 80];

t0 = 0
tf = 800
dt = .1

times = [t0]

xinit = 10;
xdinit = 0;

yinit = 10;
ydinit = 0;

thetainit = 3.5;
thetadinit = 0;

scale = 20;
xmax = scale;
xmin = -scale;
ymax  = scale;
ymin = -scale;

rcm = [-.4, -3, 0];

global parameters;

parameters.K = [5, 5, 5, 5, 0, 0]

parameters.e = [0, 0, -15, 0, 0, 0];
parameters.eP = [parameters.e(1), parameters.e(3)];

parameters.cx = .44;
parameters.cy = 1.4;
parameters.M = 500;

parameters.boatx = [-.8, -.8, 0, 0.8, 0.8, -.8]
parameters.boaty = [-3, 1.5, 3, 1.5, -3, -3]
parameters.boatxy = [parameters.boatx', parameters.boaty']

parameters.windX = 1;
parameters.windY = -1.4;
parameters.wx = .04;
parameters.wy = .005;
parameters.wz = 70;
parameters.F = 0;
parameters.Fmax = 55*4.44;
t = linspace(0, 4*pi, tf/dt);
phi = .3*sin(t);
%phi = linspace(0, 0, tf/dt);
parameters.phi = phi(1);
parameters.rcm = rcm;
parameters.rcp = [0, .3, 0]
parameters.pLength = .5;
parameters.int = 0;
parameters.intMax = 200;

parameters.destReached = false;
parameters.goTo = true;
parameters.tol = 1.5;
parameters.destTol = 10;
parameters.ki = -1;
parameters.kp = -20;
parameters.kd = -40;

parameters.pkp = 2.5;
parameters.vpkp = .5;
parameters.turnBaseAngle = 0;
parameters.fkp = 2;

parameters.distPrev = 0;
parameters.dt = dt;

parameters.disableController = false;
parameters.disableTime = 0;
parameters.timer = 0;
parameters.time = 0;
parameters.switchTime = 40;

forceLog = [];



init = [xinit; xdinit; yinit; ydinit; thetainit; thetadinit]';
states = [init];

[phiLine] = drawPhi(states, parameters.phi);

calcRCP(states);
calcBoat(states);



figure(4)
clf;
axis equal


h = plot(states(:,1), states(:,3));
hold on;
H = plot([phiLine(1), phiLine(2)], [phiLine(3), phiLine(4)], 'color', 'r');
HH = plot(parameters.boatxyI(1,:), parameters.boatxyI(2,:), 'color', 'b');
plot(parameters.eP(1), parameters.eP(2), 'marker', 'x', 'linewidth', 10);




for i = 1:(tf/dt)
calcRCP(states(end,:));
calcBoat(states(end,:));
%disp(states);
%disp(parameters.F);
[time, state] = ode45(@eom,[t0, t0+dt],states(end,:)', odeset('reltol',1e-6,'abstol',1e-9));
t0 = dt*i;
times = [times; t0];
states = [states; state(end,:)];
parameters.time = t0;
%disp(states);
%parameters.phi = phi(i);

doController(state(end,:));

if(parameters.disableController)
   parameters.F = 0;
   parameters.phi = 0;
end

checkEnable();

phiLine = drawPhi(states(end,:), parameters.phi);


h.XData = states(:, 1);
h.YData = states(:, 3);
H.XData = phiLine(1:2);
H.YData = phiLine(3:4);
HH.XData = parameters.boatxyI(1,:);
HH.YData = parameters.boatxyI(2,:);
%disp(parameters.boatxy);
%axis([xmin xmax ymin ymax]);
axis equal
drawnow;

disp("Time = " + times(end));

forceLog = [forceLog, parameters.F];



end

figure(1);
clf;

plot(times, states(:,1), times, states(:,3))

figure(2);
clf;

plot(times, states(:,2), times, states(:, 4))

figure(3)
clf;
plot(states(:,1), states(:,3))
axis equal

figure(5)
clf;
plot(times(2:end), forceLog);





function f = eom(t,x)
    %F = 55*4.44;
    
    global parameters;
    
    if(parameters.F > parameters.Fmax)
        parameters.F = parameters.Fmax;
    elseif(parameters.F < -parameters.Fmax)
        parameters.F = -parameters.Fmax;
    end
   
    F = parameters.F;
    M = parameters.M;
    cx = parameters.cx;
    
    %disp(F)
    
    %phi = .13;
    phi = parameters.phi;
    
    wx = parameters.wx;
    wy = parameters.wy; 
    
    cy = parameters.cy;
    
    I = 1666;
    %rcm = [-.4, -3, 0];
    rcm = parameters.rcm;
    T = cross(rcm, [sin(phi)*F, cos(phi)*F, 0]);
    TW = cross(parameters.rcpb, [parameters.windX, parameters.windY, 0]);
    cz = 1.1;
    
    f = zeros(6,1);

%     f(1) = x(2);
%     f(2) = x(1)*w^2;

    f(1) = x(2);
    f(2) = (F*sin(phi)*cos(x(5)) - F*cos(phi)*sin(x(5)))/M - ((cx*abs(cos(x(5))) + cy*abs(sin(x(5))))*(x(2))) - ((wx*abs(cos(x(5))) + wy*abs(sin(x(5))))*(parameters.windX));
    f(3) = x(4);
    f(4) = (F*cos(phi)*cos(x(5)) + F*sin(phi)*sin(x(5)))/M - ((cy*abs(cos(x(5))) + cx*abs(sin(x(5))))*(x(4))) - ((wy*abs(cos(x(5))) + wx*abs(sin(x(5))))*(parameters.windY));
    f(5) = x(6);
    f(6) = (T(3)-(TW(3)*parameters.wz))/I - cz*x(6);
    %disp(TW(3))
    
end

function xx = drawPhi(s, p)
    global parameters;
    %x1 = s(end, 1) + parameters.rcm(1)*cos(s(5)) - parameters.rcm(2)*sin(s(5));
    %y1 = s(end, 3) + parameters.rcm(2)*cos(s(5)) + parameters.rcm(1)*sin(s(5));
    %x2 = x1 - sin(s(5) + p);
    %y2 = tan((pi/2 + (s(5)- p)))*(x2-x1)+y1;
    %y2 = y1 - cos(s(5) + p);
    x1 = parameters.rcm(1);
    y1 = parameters.rcm(2);
    x2 = x1 - (sin(p)*parameters.pLength);
    y2 = y1 - (cos(p)*parameters.pLength);
    M = [cos(s(5)), -sin(s(5)); sin(s(5)), cos(s(5))];
    B = [x1, x2; y1, y2];
    
    xx = M*B + [s(1); s(3)];
    xx = [xx(1,1), xx(1,2), xx(2,1), xx(2,2)];
    x = s(end,:);
    cx = .44;
    cy = .9;
    %disp(((cx*abs(cos(x(5))) + cy*abs(sin(x(5))))*(x(2))));
    %disp(((cy*abs(cos(x(5))) + cx*abs(sin(x(5))))*(x(4))));
end

function calcRCP(s)
    global parameters;
    x1 = parameters.rcp(1)*cos(s(5)) - parameters.rcp(2)*sin(s(5));
    y1 = parameters.rcp(2)*cos(s(5)) + parameters.rcp(1)*sin(s(5));
    
    parameters.rcpb = [x1, y1, 0];
    
    %disp(s(5))
end

function calcBoat(s)
    global parameters;
    M = [cos(s(5)), -sin(s(5)); sin(s(5)), cos(s(5))];
    parameters.boatxyI = M*parameters.boatxy' + [s(1); s(3)];
    
end

function doController(s)

    global parameters;
    M = [cos(s(5)), -sin(s(5)); sin(s(5)), cos(s(5))];
    motorPos = parameters.rcm;
    
    motorPosI = M*parameters.rcm(1:2)' + [s(1); s(3)];
    
    dirVec = parameters.eP - motorPosI';
    
    course = atan2(dirVec(2), dirVec(1));
    
    targetPhi = s(5) + pi/2 - course;
    
    parameters.phi = targetPhi;
       
    dist = norm(dirVec);
    
    disp("Dist = " + dist);
    %disp("Theta = " + s(5));
    
    
    if(dist < parameters.destTol)
        parameters.destReached = true;
    end
    
    
    if(parameters.destReached)
        spotController(s, course, dist);
    else
        goToController(s, course, dist);
    end
    
    
    
%     parameters.int = parameters.int + (dist-1)*parameters.ki;
%     m = (dist-parameters.distPrev)/parameters.dt;
%     
%     parameters.F = parameters.int + (dist-parameters.tol)*parameters.kp + ((dist-parameters.distPrev)/parameters.dt)*parameters.kd;
%     
%     parameters.distPrev = dist;
%     if(dist < parameters.tol)
%         %parameters.F = 0;
%         %parameters.int = 0;
%         %parameters.int = 0;
%     end
%     
%     if(dist<parameters.tol && ~parameters.destReached)
%         parameters.int = 0;
%         parameters.destReached = true;
%         parameters.F = 0;
%     end
%     
%     
%     if(parameters.int > parameters.intMax)
%         parameters.int = parameters.intMax;
%     end
%     
%     if(parameters.F > parameters.Fmax)
%         parameters.F = parameters.Fmax;
%     end
%     
%     disp(m);
   
end


function spotController(s, c, d)

    global parameters;
    
    M = [cos(s(5)), -sin(s(5)); sin(s(5)), cos(s(5))];
    
    %disp(s);
    
    targetPhi = s(5) + pi/2 - c;
    parameters.phi = targetPhi - pi;
    
    %v = norm([s(2), s(4)]);
    
    vi = [s(2); s(4)];
    
    vb = inv(M)*vi;
    
    parameters.error = s - parameters.e;
    
    parameters.int = parameters.int + (d-parameters.tol)*parameters.dt;
    
    parameters.F = (d-parameters.tol)*parameters.kp + parameters.int*parameters.ki + (sign(parameters.F))*abs(vb(2))*parameters.kd;
    
    %parameters.int = parameters.int + (d - parameters.tol)/parameters.dt;
    
    %parameters.F = -parameters.K*parameters.error' + parameters.ki*parameters.int;
    
    disp("Thrust = " + parameters.F);
    disp("Int = " + parameters.int*parameters.ki);
    disp("Deriv = " + (parameters.F/(abs(parameters.F)))*abs(vb(2))*parameters.kd);
    disp("Distance = " + d);
    
    
    if(d<.3 && ~parameters.disableController)
        parameters.F = 0;
        parameters.phi = 0;
        %disableController(20);
        %parameters.disableController = true;
        disableController();
    end
    
    if(d>1 && parameters.disableController)
        %parameters.F = 0;
        %parameters.phi = 0;
        %disableController(2);
        %parameters.disableController = false;
        enableController();
    end
    
    if(parameters.phi < -pi/2)
        while(parameters.phi < -pi/2)
            parameters.phi = parameters.phi + pi;
            parameters.F = -parameters.F;
        end
    elseif(parameters.phi > pi/2)
        while(parameters.phi > pi/2)
            parameters.phi = parameters.phi - pi;
            parameters.F = -parameters.F;
        end
    end
    
    disp("Phi = " + parameters.phi);

end

function goToController(s, c, d)
    global parameters;
    
    a = parameters.eP - [s(1), s(3)];
    angle = atan2(a(2), a(1));
    %disp(angle*180/pi);
    %disp(s(5)*180/pi);
    parameters.diffAngle = ((s(5)+ pi/2 - angle)*180/pi);
    parameters.diffAngleR = ((s(5)+ pi/2 - angle));
    
    vAngle = atan2(s(4), s(2));
    parameters.vDiffAngleR = ((vAngle - angle));
    
    %disp("Velocity angle = " + vAngle);
    
    if(parameters.goTo)   
             
        if(abs(parameters.diffAngle) < 90)
            forward = true;
        else
            forward = false;
        end
        disp(parameters.diffAngle);
        
        
        
        parameters.goTo = false;
    end
    
    if(parameters.diffAngleR > pi)
        parameters.diffAngleR = parameters.diffAngleR - 2*pi;
    end
    
    if(parameters.vDiffAngleR > pi)
        parameters.vDiffAngleR = parameters.vDiffAngleR - 2*pi;
    end
    
    
    
    %if(parameters.time < parameters.switchTime)
        parameters.phi = parameters.turnBaseAngle - parameters.diffAngleR*parameters.pkp;
        disp("Diff = " + parameters.diffAngleR);
    %else
        %parameters.phi = parameters.turnBaseAngle - parameters.vDiffAngleR*parameters.vpkp;
        %disp("vDiff = " + parameters.vDiffAngleR);
    %end
    disp("Phi = " + parameters.phi);
    
    if(parameters.phi < -pi/2)
        parameters.phi = -pi/2;
    elseif(parameters.phi > pi/2)
        parameters.phi = pi/2;
    end
    
    
    parameters.F = 100 + parameters.fkp*d - abs(parameters.phi)*100;
    
    
end


function disableControllerTime(t)

    global parameters;
    parameters.disableTime = t;
    parameters.disableController = true;
    parameters.timer = 0;
    
end

function disableController()
    global parameters;
    parameters.disableTime = inf;
    parameters.timer = 0;
    parameters.disableController = true;
end

function enableController()
    global parameters;
    parameters.disableTime = 0;
    parameters.timer = 0;
    parameters.disableController = false;
end


function checkEnable()
    global parameters;
    if(parameters.timer > parameters.disableTime && parameters.disableController)
        parameters.disableController = false;
        parameters.timer = 0;
    else
        parameters.timer = parameters.timer + parameters.dt;
    end
    
end


