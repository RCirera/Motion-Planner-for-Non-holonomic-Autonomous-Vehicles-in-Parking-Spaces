function []=plotMotion(motion,uBounds,qBounds,dims,location,sampleRate)

% The function plotMotion plots several states and inputs of the motion.
%
% The syntax is []=plotMotion(motion,uBounds,qBounds,dims,location,sampleRate)
%
% There are no function outputs.
%
% The function arguments are:
%   - motion: Time series collection containing the states and inputs of
%   the motion.
%   - uBounds: 2x2 array containing the maximum and minimum values allowed
%   for the control history inputs.
%   - qBounds: 7x1 vector containing maximum values for certain parameters to
%   ensure passenger comfort. In order, the elements are maximum
%   longitudinal acceleration, minimum longitudinal acceleration (maximum
%   deceleration/braking), maximum longitudinal jerk, maximum lateral
%   acceleration, maximum lateral jerk, maximum longitudinal velocity and
%   minimum longitudinal velocity. The units are m/s for velocities, m/s^2
%   for accelerations and m/s^3 for jerks. 
%   - dims: Array containing information on geometric properties of the
%   vehicle for which the motion is calculated.
%   - location: String defining the desired location for the figure created
%   by the function.
%   - sampleRate: Double, rate of resampling of the timeseries cllection to
%   use in the figure.


%% Visualize v, delta, a, deltaRef, aRef
motion=resample(motion,0:sampleRate:(motion.Time(end)));
figure()
movegui(location) %Change the location of the figure


% v vs time, with vMax and vMin
subplot(3,2,1)
plot(motion.v,'LineWidth',1.5)
hold on
vMax=refline([0 qBounds(6)]);
vMax.Color='r';
vMax.LineStyle='--';
vMin=refline([0 qBounds(7)]);
vMin.Color='b';
vMin.LineStyle='--';
title('')
ylabel('Velocity [m/s]')
xlabel('Time [s]')
legend('v','vMax','vMin',...
       'Location','northoutside','Orientation','horizontal')
grid on
axis([0 motion.Time(end)*1.01 -Inf Inf])
grid minor
hold off

% delta and deltaRef vs time, with delta RefMax and deltaRefMin
subplot(3,2,2)
plot(motion.delta,'LineWidth',1.5)
hold on
stairs(motion.deltaRef.Time.',permute(motion.deltaRef.Data,[3 2 1]),'LineWidth',0.75)
deltaMax=refline([0 qBounds(8)]);
deltaMax.Color='r';
deltaMax.LineStyle='--';
deltaMin=refline([0 qBounds(9)]);
deltaMin.Color='b';
deltaMin.LineStyle='--';
deltaRefMax=refline([0 uBounds(3)]);
deltaRefMax.Color='r';
deltaRefMax.LineStyle='-.';
deltaRefMin=refline([0 uBounds(4)]);
deltaRefMin.Color='b';
deltaRefMin.LineStyle='-.';
title('')
ylabel('Steering Angle [rad]')
xlabel('Time [s]')
legend('delta','deltaRef','deltaMax','deltaMin','deltaRefMax','deltaRefMin',...
       'Location','northoutside','Orientation','horizontal')
grid on
axis([0 motion.Time(end)*1.01 -Inf Inf])
grid minor
hold off

% a and aRef vs time, with aRefMax and aRefMin
subplot(3,2,3)
plot(motion.a,'LineWidth',1.5)
hold on
stairs(motion.aRef.Time.',permute(motion.aRef.Data,[3 2 1]),'LineWidth',0.75)
aMax=refline([0 qBounds(1)]);
aMax.Color='r';
aMax.LineStyle='--';
aMin=refline([0 qBounds(2)]);
aMin.Color='b';
aMin.LineStyle='--';
aRefMax=refline([0 uBounds(1)]);
aRefMax.Color='r';
aRefMax.LineStyle='-.';
aRefMin=refline([0 uBounds(2)]);
aRefMin.Color='b';
aRefMin.LineStyle='-.';
title('')
ylabel('Longitudinal Acceleration [m/s^2]')
xlabel('Time [s]')
legend('a','aRef','aMax','aMin','aRefMax','aRefMin',...
       'Location','northoutside','Orientation','horizontal')
grid on
axis([0 motion.Time(end)*1.01 -Inf Inf])
grid minor
hold off

% Calculate jLong, aLat and jLat
jLong=(motion.aRef.Data(1,:)-motion.a.Data(1,:))./dims(6); %Longitudinal jerk
aLat=(motion.v.Data(1,:).^2./(dims(1)./tan(motion.delta.Data(1,:)))); %Lateral acceleration
jLat=diff(aLat)./diff(motion.Time.'); %Lateral jerk

% jLong vs time, with jLongMax and jLongMin
subplot(3,2,5)
plot(motion.Time,jLong,'LineWidth',1.5)
hold on
jLongMax=refline([0 qBounds(3)]);
jLongMax.Color='r';
jLongMax.LineStyle='--';
jLongMin=refline([0 -qBounds(3)]);
jLongMin.Color='b';
jLongMin.LineStyle='--';
title('')
ylabel('Longitudinal Jerk [m/s^3]')
xlabel('Time [s]')
legend('j','jMax','jMin',...
       'Location','northoutside','Orientation','horizontal')
grid on
axis([0 motion.Time(end)*1.01 -Inf Inf])
grid minor
hold off

% aLat vs time, with aLatMax and aLatMin
subplot(3,2,4)
plot(motion.Time,aLat,'LineWidth',1.5)
hold on
aLatMax=refline([0 qBounds(4)]);
aLatMax.Color='r';
aLatMax.LineStyle='--';
aLatMin=refline([0 -qBounds(4)]);
aLatMin.Color='b';
aLatMin.LineStyle='--';
title('')
ylabel('Lateral Acceleratoin [m/s^2]')
xlabel('Time [s]')
legend('aLat','jLatMax','aLatMin',...
       'Location','northoutside','Orientation','horizontal')
grid on
axis([0 motion.Time(end)*1.01 -Inf Inf])
grid minor
hold off

% jLat vs time, with jLatMax and jLatMin
subplot(3,2,6)
plot(motion.Time(1:end-1,1),jLat,'LineWidth',1.5)
hold on
jLatMax=refline([0 qBounds(5)]);
jLatMax.Color='r';
jLatMax.LineStyle='--';
jLatMin=refline([0 -qBounds(5)]);
jLatMin.Color='b';
jLatMin.LineStyle='--';
title('')
ylabel('Lateral Jerk [m/s^3]')
xlabel('Time [s]')
legend('jLat','jLatMax','jLatMin',...
       'Location','northoutside','Orientation','horizontal')
grid on
axis([0 motion.Time(end)*1.01 -Inf Inf])
grid minor
hold off


end

