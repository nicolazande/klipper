% select parameters
prefix = '30xcurrent-4-100-0.045/';
load = 'no-pp';
source_type = 'square-wave';
amplitude = '10mm';
frequency = '1Hz';

% table data
%Table = readtable(['data/', source_type, '-', amplitude, '-', frequency, '.txt']);
%Table.Properties.VariableNames = ["time", "target-position", "position-demand", "actual-position", "actual-speed", "current", "torque"];
Table = readtable(['data/', prefix, load, '-', source_type, '-', amplitude, '-', frequency, '.txt']);
Table.Properties.VariableNames = ["time", "target-position", "position-demand", "actual-position", "actual-speed", "current"];

% Create plots
setpoint = 'target-position';
feedback = 'actual-position';
plot(Table.("time"), Table.(setpoint));
hold on;
plot(Table.("time"), Table.(feedback));
title([source_type, ' - ', amplitude, ' - ', frequency]);
legend('reference', 'feedback');

% iddata for as simulation input
idddata_input = iddata(0.001*Table.("target-position"), [], 0.01);