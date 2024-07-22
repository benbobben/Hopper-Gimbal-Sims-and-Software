%% Creating pulse generator 


variable_pulse_generator(1, 20)

function [onTime,offTime] = variable_pulse_generator(period, duty_cycle)
    onTime  = (period * duty_cycle / 100)
    offTime = period * (1 - duty_cycle / 100)
end